/**
\brief CoRal TDMA anchor node.

Starts in RX. cb_endFrame only branches on radio state. An ibeacon
joins if needed and always updates the next slot start. A slot-0 echo
that completes the ibeacon/echo pair raises a flag; CTE processing
runs in mote_main(). After a report TX the radio is turned off.

Report slot is assigned from EUI64 bytes [1] and [3] using
eui_report_slot_map[], defined before mote_main().

Timer0 runs at 16 MHz (prescaler 0):
  (16000000/200)     = 5 ms
  (16000000/200)*2   = 10 ms
  (16000000/200)*100 = 500 ms

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Sept 2025.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "radio_df.h"
#include "aod.h"
#include "uart.h"
#include "timer.h"
#include "eui64.h"
#include "math.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         17              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)*2+7)
#define LENGTH_SERIAL_FRAME  127

#define ENABLE_DF       1

#define DEBUG_RADIO_PIN 11

#define SLOT_DURATION     ((16000000/200)*100)  // 500 ms @ 16 MHz
#define SYNC_OFFSET       ((16000000/200)*2)    // beacon at 10 ms into the slot
#define REPORT_DELAY      SYNC_OFFSET           // 10 ms after ibeacon RX
#define NUM_SLOTS         10
// Slot map: 0 REF beacon + TGT echo; 1-8 RX report to TGT; 9 TGT position broadcast

#define SLOT_TIMER_ID     0
#define INNER_TIMER_ID    3   // do not use CC[1]: radio.c captures it on PHYEND
#define CAPTURE_ID        2

#define NODE_ID_REF       0
#define NODE_ID_TGT       1
#define NODE_POS_X        0
#define NODE_POS_Y        0
#define NEXT_IBEACON_CH   0

#define PKT_IDX_S0        0
#define PKT_IDX_ADDR      2
#define PKT_IDX_NODE_ID   8
#define PKT_IDX_SLOT      9

#define NUM_ANCHORS       8
#define CTE_PI            3.14159265358979323846f
#define CTE_TWO_PI        (2.0f * CTE_PI)

//=========================== slot assignment =================================
// Fill eui_id with board_eui[1] (high byte) and board_eui[3] (low byte).
// Example: board_eui[1]=0xA1, board_eui[3]=0xC3 -> 0xA1C3. 0x0000 = not filled.
// Node ids: REF=0, TGT=1, anchors=2-9 (report slots 1-8).
typedef struct {
    uint16_t eui_id;
    uint8_t  report_slot;
} eui_report_slot_map_t;

static const eui_report_slot_map_t eui_report_slot_map[NUM_ANCHORS] = {
    {0x04a4, 1},
    {0x788d, 2},
    {0x9707, 3},
    {0x67fe, 4},
    {0xeab2, 5},
    {0xe7be, 6},
    {0x901d, 7},
    {0x608e, 8},
};

//=========================== variables =======================================

uint16_t length = 0;

const static uint8_t ble_device_addr[6] = {
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

typedef enum {
    APP_STATE_TX          = 0x01,
    APP_STATE_RX          = 0x02,
    APP_STATE_OFF         = 0x04,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                app_state_t     state;
                bool            get_sync;
                bool            report_pending;
                bool            slot0_ibeacon_rx;
                bool            slot0_echo_rx;
     volatile   bool            cte_process_pending;

                uint16_t        my_eui_id;
                uint8_t         my_node_id;
                uint8_t         my_report_slot;

                uint8_t         slot_number;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;

                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;

                uint32_t        ibeacon_cte[NUM_SAMPLES];
                uint16_t        ibeacon_cte_len;
                uint32_t        ibeacon_cte_ts;
                bool            ibeacon_cte_valid;

                uint32_t        echo_cte[NUM_SAMPLES];
                uint16_t        echo_cte_len;
                uint32_t        echo_cte_ts;
                bool            echo_cte_valid;

                // Written only at end of process_cte_info(); read by assemble in ISR.
     volatile   float           cte_ls_intercept;
     volatile   float           cte_ls_slope;
     volatile   int32_t         cte_time_diff;
     volatile   bool            cte_result_valid;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);
void     cb_slot_inner_timer(void);
void     assemble_report_packet(uint8_t slot_number);
void     process_cte_info(void);
void     save_cte_samples(uint8_t node_id, uint32_t timestamp);
void     start_rx(void);
void     nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
static bool is_valid_coral_pdu(void);
static bool is_ref_beacon(void);
static bool is_tgt_echo(void);
static void assign_report_slot_from_eui64(void);

//=========================== main ============================================

int mote_main(void) {
    
    memset(&app_vars, 0, sizeof(app_vars_t));

    assign_report_slot_from_eui64();

    board_init();

    radio_rfOff();

    app_vars.state               = APP_STATE_OFF;
    app_vars.get_sync            = FALSE;
    app_vars.report_pending      = FALSE;
    app_vars.slot0_ibeacon_rx    = FALSE;
    app_vars.slot0_echo_rx       = FALSE;
    app_vars.cte_process_pending = FALSE;
    app_vars.cte_result_valid    = FALSE;

    nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);
#if ENABLE_DF == 1
    radio_configure_direction_finding_antenna_switch();
    radio_configure_direction_finding_manual_AoA();
#endif

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    timer_init();
    timer_start();
    timer_set_callback(SLOT_TIMER_ID,  cb_timer);
    timer_set_callback(INNER_TIMER_ID, cb_slot_inner_timer);

    start_rx();

    while (1) {
        if (app_vars.cte_process_pending == TRUE) {
            app_vars.cte_process_pending = FALSE;
            process_cte_info();
        }
        board_sleep();
    }
}

//=========================== private =========================================

static void assign_report_slot_from_eui64(void) {
    uint8_t  i;
    uint8_t  board_eui[8];

    eui64_get(board_eui);
    app_vars.my_eui_id      = ((uint16_t)board_eui[1] << 8) | board_eui[3];
    app_vars.my_report_slot = 0;
    app_vars.my_node_id     = 0;

    for (i = 0; i < NUM_ANCHORS; i++) {
        if (eui_report_slot_map[i].eui_id == 0x0000) {
            continue;
        }
        if (eui_report_slot_map[i].eui_id == app_vars.my_eui_id) {
            app_vars.my_report_slot = eui_report_slot_map[i].report_slot;
            app_vars.my_node_id     = (uint8_t)(app_vars.my_report_slot + 1);
            break;
        }
    }
}

void start_rx(void) {
    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
#if ENABLE_DF == 1
    radio_configure_direction_finding_manual_AoA();
#endif
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;
    radio_rxNow();
}

void process_cte_info(void) {
    uint16_t i;
    uint16_t n;
    uint16_t n_valid;
    int16_t  ref_i;
    int16_t  ref_q;
    int16_t  echo_i;
    int16_t  echo_q;
    float    den;
    float    div_i;
    float    div_q;
    float    phase;
    float    prev_unwrapped;
    float    delta;
    float    x;
    float    sum_x;
    float    sum_x2;
    float    sum_y;
    float    sum_xy;
    float    denom;
    float    slope;
    float    intercept;
    int32_t  time_diff;

    // Compute into locals first. Do NOT clear shared results at start:
    // cb_timer may assemble a report while this function is still running.
    time_diff = (int32_t)(app_vars.echo_cte_ts - app_vars.ibeacon_cte_ts);

    if ((app_vars.ibeacon_cte_valid == FALSE) || (app_vars.echo_cte_valid == FALSE)) {
        app_vars.cte_time_diff    = time_diff;
        app_vars.cte_result_valid = FALSE;
        return;
    }

    n = app_vars.ibeacon_cte_len;
    if (app_vars.echo_cte_len < n) {
        n = app_vars.echo_cte_len;
    }
    if (n > NUM_SAMPLES) {
        n = NUM_SAMPLES;
    }
    if (n < 2) {
        app_vars.cte_time_diff    = time_diff;
        app_vars.cte_result_valid = FALSE;
        return;
    }

    n_valid        = 0;
    prev_unwrapped = 0.0f;
    sum_x          = 0.0f;
    sum_x2         = 0.0f;
    sum_y          = 0.0f;
    sum_xy         = 0.0f;

    for (i = 0; i < n; i++) {
        ref_i  = (int16_t)(app_vars.ibeacon_cte[i] & 0xFFFF);
        ref_q  = (int16_t)((app_vars.ibeacon_cte[i] >> 16) & 0xFFFF);
        echo_i = (int16_t)(app_vars.echo_cte[i] & 0xFFFF);
        echo_q = (int16_t)((app_vars.echo_cte[i] >> 16) & 0xFFFF);

        den = (float)echo_i * (float)echo_i + (float)echo_q * (float)echo_q;
        if (den == 0.0f) {
            continue;
        }

        // (I_ref + j Q_ref) / (I_echo + j Q_echo)
        div_i = ((float)ref_i * (float)echo_i + (float)ref_q * (float)echo_q) / den;
        div_q = ((float)ref_q * (float)echo_i - (float)ref_i * (float)echo_q) / den;
        phase = atan2f(div_q, div_i);

        if (n_valid == 0) {
            prev_unwrapped = phase;
        } else {
            delta = phase - prev_unwrapped;
            while (delta > CTE_PI) {
                delta -= CTE_TWO_PI;
            }
            while (delta < -CTE_PI) {
                delta += CTE_TWO_PI;
            }
            prev_unwrapped = prev_unwrapped + delta;
        }

        x = (float)i;
        sum_x  += x;
        sum_x2 += x * x;
        sum_y  += prev_unwrapped;
        sum_xy += x * prev_unwrapped;
        n_valid++;
    }

    if (n_valid < 2) {
        app_vars.cte_time_diff    = time_diff;
        app_vars.cte_result_valid = FALSE;
        return;
    }

    denom = (float)n_valid * sum_x2 - sum_x * sum_x;
    if (denom == 0.0f) {
        app_vars.cte_time_diff    = time_diff;
        app_vars.cte_result_valid = FALSE;
        return;
    }

    slope     = ((float)n_valid * sum_xy - sum_x * sum_y) / denom;
    intercept = (sum_y - slope * sum_x) / (float)n_valid;

    // Publish as one consistent snapshot for assemble_report_packet().
    app_vars.cte_ls_slope     = slope;
    app_vars.cte_ls_intercept = intercept;
    app_vars.cte_time_diff    = time_diff;
    app_vars.cte_result_valid = TRUE;
}

void assemble_report_packet(uint8_t slot_number) {
    uint8_t  i;
    int16_t  pos_x;
    int16_t  pos_y;
    float    intercept;
    float    slope;
    int32_t  time_diff;

    i = 0;
    pos_x = NODE_POS_X;
    pos_y = NODE_POS_Y;

    // Snapshot volatile results once for a consistent PDU.
    intercept = app_vars.cte_ls_intercept;
    slope     = app_vars.cte_ls_slope;
    time_diff = app_vars.cte_time_diff;

    memset(app_vars.packet, 0x00, sizeof(app_vars.packet));

    app_vars.packet[i++]  = 0x42;
    app_vars.packet[i++]  = 0x00;
    app_vars.packet[i++]  = ble_device_addr[0];
    app_vars.packet[i++]  = ble_device_addr[1];
    app_vars.packet[i++]  = ble_device_addr[2];
    app_vars.packet[i++]  = ble_device_addr[3];
    app_vars.packet[i++]  = ble_device_addr[4];
    app_vars.packet[i++]  = ble_device_addr[5];

    app_vars.packet[i++]  = app_vars.my_node_id;
    app_vars.packet[i++]  = slot_number;
    app_vars.packet[i++]  = (uint8_t)(pos_x & 0xff);
    app_vars.packet[i++]  = (uint8_t)((pos_x >> 8) & 0xff);
    app_vars.packet[i++]  = (uint8_t)(pos_y & 0xff);
    app_vars.packet[i++]  = (uint8_t)((pos_y >> 8) & 0xff);

    memcpy(&app_vars.packet[i], &intercept, sizeof(intercept));
    i += 4;
    memcpy(&app_vars.packet[i], &slope, sizeof(slope));
    i += 4;
    memcpy(&app_vars.packet[i], &time_diff, sizeof(time_diff));
    i += 4;

    app_vars.packet[i++]  = NEXT_IBEACON_CH;

    app_vars.packet[1]    = i - 2;
}

void save_cte_samples(uint8_t node_id, uint32_t timestamp) {
    //leds_debug_toggle();
#if ENABLE_DF == 1
    if (node_id == NODE_ID_REF) {
        app_vars.ibeacon_cte_len   = radio_get_df_samples(app_vars.ibeacon_cte, NUM_SAMPLES);
        app_vars.ibeacon_cte_ts    = timestamp;
        app_vars.ibeacon_cte_valid = (app_vars.ibeacon_cte_len > 0) ? TRUE : FALSE;
    } else if (node_id == NODE_ID_TGT) {
        app_vars.echo_cte_len      = radio_get_df_samples(app_vars.echo_cte, NUM_SAMPLES);
        app_vars.echo_cte_ts       = timestamp;
        app_vars.echo_cte_valid    = (app_vars.echo_cte_len > 0) ? TRUE : FALSE;
    }
#else
    (void)node_id;
    (void)timestamp;
#endif
}

static bool is_valid_coral_pdu(void) {
    uint8_t i;

    if (app_vars.rxpk_crc == FALSE) {
        return FALSE;
    }
    if (app_vars.rxpk_packet_len < 15) {
        return FALSE;
    }
    if (app_vars.rxpk_packet[PKT_IDX_S0] != 0x42) {
        return FALSE;
    }
    for (i = 0; i < 6; i++) {
        if (app_vars.rxpk_packet[PKT_IDX_ADDR + i] != ble_device_addr[i]) {
            return FALSE;
        }
    }
    return TRUE;
}

static bool is_ref_beacon(void) {
    if (is_valid_coral_pdu() == FALSE) {
        return FALSE;
    }
    if (app_vars.rxpk_packet[PKT_IDX_NODE_ID] != NODE_ID_REF) {
        return FALSE;
    }
    return TRUE;
}

static bool is_tgt_echo(void) {
    if (is_valid_coral_pdu() == FALSE) {
        return FALSE;
    }
    if (app_vars.rxpk_packet[PKT_IDX_NODE_ID] != NODE_ID_TGT) {
        return FALSE;
    }
    return TRUE;
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    uint8_t pkt_slot;

    app_dbg.num_endFrame++;

    if (app_vars.state == APP_STATE_RX) {
        radio_getReceivedFrame(
            app_vars.rxpk_packet,
            &app_vars.rxpk_packet_len,
            sizeof(app_vars.rxpk_packet),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );

        if (is_ref_beacon() == TRUE) {
            pkt_slot = app_vars.rxpk_packet[PKT_IDX_SLOT];

            if (app_vars.get_sync == FALSE) {
                app_vars.get_sync    = TRUE;
                app_vars.slot_number = pkt_slot;
            } else {
                app_vars.slot_number = pkt_slot;
            }

            app_vars.time_slotStartAt = timestamp - SYNC_OFFSET;
            timer_schedule(SLOT_TIMER_ID, app_vars.time_slotStartAt + SLOT_DURATION);

            if (pkt_slot == 0) {
                save_cte_samples(NODE_ID_REF, timestamp);
                app_vars.slot0_ibeacon_rx = TRUE;
                start_rx();
                return;
            }

            start_rx();
            return;
        }

        if (is_tgt_echo() == TRUE) {
            //leds_debug_toggle();
            pkt_slot = app_vars.rxpk_packet[PKT_IDX_SLOT];

            if (pkt_slot == 0) {
                save_cte_samples(NODE_ID_TGT, timestamp);
                app_vars.slot0_echo_rx = TRUE;
                if ((app_vars.slot0_ibeacon_rx == TRUE) &&
                    (app_vars.slot0_echo_rx == TRUE)) {
                    app_vars.cte_process_pending = TRUE;
                }
            }
            start_rx();
            return;
        }

        start_rx();
        return;
    }

    if (app_vars.state == APP_STATE_TX) {
        radio_rfOff();
        app_vars.state = APP_STATE_OFF;
        return;
    }
}

void cb_timer(void) {
    leds_error_toggle();
    app_dbg.num_timer++;

    if (app_vars.get_sync == FALSE) {
        return;
    }

    app_vars.time_slotStartAt += SLOT_DURATION;
    timer_schedule(SLOT_TIMER_ID, app_vars.time_slotStartAt + SLOT_DURATION);
    app_vars.slot_number = (uint8_t)((app_vars.slot_number + 1) % NUM_SLOTS);
    app_vars.report_pending = FALSE;

    if (app_vars.slot_number == 0) {
        app_vars.slot0_ibeacon_rx  = FALSE;
        app_vars.slot0_echo_rx     = FALSE;
        app_vars.ibeacon_cte_valid = FALSE;
        app_vars.echo_cte_valid    = FALSE;
        app_vars.cte_result_valid  = FALSE;
        start_rx();
        return;
    }

    // Own report slot: only TX if this round's CTE fit succeeded.
    // Missing echo / failed LS leaves cte_result_valid FALSE (cleared at slot 0).
    if ((app_vars.my_report_slot != 0) &&
        (app_vars.slot_number == app_vars.my_report_slot) &&
        (app_vars.cte_result_valid == TRUE)) {
        assemble_report_packet(app_vars.slot_number);
        radio_rfOff();
        radio_rfOn();
        radio_setFrequency(CHANNEL, FREQ_TX);
#if ENABLE_DF == 1
        radio_configure_direction_finding_manual_AoA();
#endif
        radio_loadPacket(app_vars.packet, LENGTH_PACKET);
        radio_txEnable();
        app_vars.state          = APP_STATE_TX;
        app_vars.report_pending = TRUE;
        timer_schedule(INNER_TIMER_ID, app_vars.time_slotStartAt + SYNC_OFFSET + REPORT_DELAY);
        return;
    }

    start_rx();
}

void cb_slot_inner_timer(void) {
    if (app_vars.report_pending == FALSE) {
        return;
    }
    //leds_debug_toggle();
    app_vars.report_pending = FALSE;
    radio_txNow();
}
