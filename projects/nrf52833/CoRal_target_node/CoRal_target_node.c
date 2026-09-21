/**
\brief CoRal TDMA target node.

Starts in RX. cb_endFrame branches on radio state. On a valid
reference beacon it joins the 500 ms slot grid. In slot 0 it replies
with an echo (node id = 1). In report slots 1-8 it stays in RX and
stores each anchor report (id, position, intercept, slope, time diff).
When all eight reports are present, report_data_process() runs from
mote_main().

Timer0 runs at 16 MHz (prescaler 0):
  (16000000/200)     = 5 ms
  (16000000/200)*2   = 10 ms
  (16000000/200)*100 = 500 ms
  (16000000/5000)    = 200 us

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Sept 2026.
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
#define ECHO_DELAY        (16000000/5000)       // 200 us after beacon RX
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
#define PKT_IDX_POS_X     10
#define PKT_IDX_POS_Y     12
#define PKT_IDX_INTERCEPT 14
#define PKT_IDX_SLOPE     18
#define PKT_IDX_TIME_DIFF 22
#define PKT_IDX_NEXT_CH   26
#define REPORT_PKT_MIN_LEN 27

#define NUM_ANCHORS       8
#define ANCHOR_ID_MIN     2
#define ANCHOR_ID_MAX     9

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
    bool     valid;
    uint8_t  anchor_id;
    int16_t  pos_x;
    int16_t  pos_y;
    float    intercept;
    float    slope;
    int32_t  time_diff;
    uint32_t intercept_bits; // raw LE bits from PDU, for debug
    uint32_t slope_bits;
} anchor_report_t;

typedef struct {
                app_state_t     state;
                bool            get_sync;
                bool            echo_pending;
     volatile   bool            report_process_pending;
                bool            report_process_done;

                uint8_t         slot_number;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;

                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;

                anchor_report_t reports[NUM_ANCHORS];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);
void     cb_slot_inner_timer(void);
void     assemble_echo_packet(uint8_t slot_number);
void     save_anchor_report(void);
void     report_data_process(void);
static bool all_reports_received(void);
void     start_rx(void);
void     nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
static bool is_valid_coral_pdu(void);
static bool is_ref_beacon(void);
static bool is_anchor_report(void);

//=========================== main ============================================

int mote_main(void) {

    memset(&app_vars, 0, sizeof(app_vars_t));

    board_init();

    radio_rfOff();
    app_vars.state                 = APP_STATE_OFF;
    app_vars.get_sync              = FALSE;
    app_vars.echo_pending          = FALSE;
    app_vars.report_process_pending = FALSE;
    app_vars.report_process_done   = FALSE;

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
        if (app_vars.report_process_pending == TRUE) {
            app_vars.report_process_pending = FALSE;
            report_data_process();
        }
        board_sleep();
    }
}

//=========================== private =========================================

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

void assemble_echo_packet(uint8_t slot_number) {

    uint8_t i;
    int16_t pos_x;
    int16_t pos_y;

    i = 0;
    pos_x = NODE_POS_X;
    pos_y = NODE_POS_Y;

    memset(app_vars.packet, 0x00, sizeof(app_vars.packet));

    app_vars.packet[i++]  = 0x42;
    app_vars.packet[i++]  = 0x00;
    app_vars.packet[i++]  = ble_device_addr[0];
    app_vars.packet[i++]  = ble_device_addr[1];
    app_vars.packet[i++]  = ble_device_addr[2];
    app_vars.packet[i++]  = ble_device_addr[3];
    app_vars.packet[i++]  = ble_device_addr[4];
    app_vars.packet[i++]  = ble_device_addr[5];

    app_vars.packet[i++]  = NODE_ID_TGT;
    app_vars.packet[i++]  = slot_number;
    app_vars.packet[i++]  = (uint8_t)(pos_x & 0xff);
    app_vars.packet[i++]  = (uint8_t)((pos_x >> 8) & 0xff);
    app_vars.packet[i++]  = (uint8_t)(pos_y & 0xff);
    app_vars.packet[i++]  = (uint8_t)((pos_y >> 8) & 0xff);
    app_vars.packet[i++]  = NEXT_IBEACON_CH;

    app_vars.packet[1]    = i - 2;
}

static uint32_t get_u32_le(uint8_t *p) {
    return  ((uint32_t)p[0]      ) |
            ((uint32_t)p[1] <<  8) |
            ((uint32_t)p[2] << 16) |
            ((uint32_t)p[3] << 24);
}

static float get_f32_le(uint8_t *p) {
    union {
        float    f;
        uint32_t u;
    } conv;

    conv.u = get_u32_le(p);
    return conv.f;
}

static int16_t get_i16_le(uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool all_reports_received(void) {
    uint8_t i;

    for (i = 0; i < NUM_ANCHORS; i++) {
        if (app_vars.reports[i].valid == FALSE) {
            return FALSE;
        }
    }
    return TRUE;
}

void report_data_process(void) {
    // Placeholder: fuse app_vars.reports[0..7] into a position estimate.
}

void save_anchor_report(void) {
    uint8_t         anchor_id;
    uint8_t         idx;
    anchor_report_t *rpt;

    anchor_id = app_vars.rxpk_packet[PKT_IDX_NODE_ID];
    if ((anchor_id < ANCHOR_ID_MIN) || (anchor_id > ANCHOR_ID_MAX)) {
        return;
    }

    idx = (uint8_t)(anchor_id - ANCHOR_ID_MIN);
    rpt = &app_vars.reports[idx];

    rpt->valid     = TRUE;
    rpt->anchor_id = anchor_id;
    rpt->pos_x     = get_i16_le(&app_vars.rxpk_packet[PKT_IDX_POS_X]);
    rpt->pos_y     = get_i16_le(&app_vars.rxpk_packet[PKT_IDX_POS_Y]);

    memcpy(&rpt->intercept, &app_vars.rxpk_packet[PKT_IDX_INTERCEPT], sizeof(float));
    memcpy(&rpt->slope,     &app_vars.rxpk_packet[PKT_IDX_SLOPE],     sizeof(float));
    memcpy(&rpt->time_diff, &app_vars.rxpk_packet[PKT_IDX_TIME_DIFF], sizeof(int32_t));

    rpt->intercept_bits = get_u32_le(&app_vars.rxpk_packet[PKT_IDX_INTERCEPT]);
    rpt->slope_bits     = get_u32_le(&app_vars.rxpk_packet[PKT_IDX_SLOPE]);

    if ((app_vars.report_process_done == FALSE) &&
        (all_reports_received() == TRUE)) {
        app_vars.report_process_pending = TRUE;
        app_vars.report_process_done    = TRUE;
    }
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

static bool is_anchor_report(void) {
    uint8_t node_id;

    if (is_valid_coral_pdu() == FALSE) {
        return FALSE;
    }
    if (app_vars.rxpk_packet_len < REPORT_PKT_MIN_LEN) {
        return FALSE;
    }
    node_id = app_vars.rxpk_packet[PKT_IDX_NODE_ID];
    if ((node_id < ANCHOR_ID_MIN) || (node_id > ANCHOR_ID_MAX)) {
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
                timer_schedule(INNER_TIMER_ID, timestamp + ECHO_DELAY);
                assemble_echo_packet(0);
                //radio_rfOn();
                radio_setFrequency(CHANNEL, FREQ_TX);
#if ENABLE_DF == 1
                radio_configure_direction_finding_manual_AoA();
#endif
                radio_loadPacket(app_vars.packet, LENGTH_PACKET);
                radio_txEnable();
                app_vars.state        = APP_STATE_TX;
                app_vars.echo_pending = TRUE;
                return;
            }

            app_vars.echo_pending = FALSE;
            start_rx();
            return;
        }

        if (is_anchor_report() == TRUE) {
            save_anchor_report();
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

    switch (app_vars.slot_number) {
    case 0:
        memset(app_vars.reports, 0, sizeof(app_vars.reports));
        app_vars.report_process_pending = FALSE;
        app_vars.report_process_done    = FALSE;
        start_rx();
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
        // Report slots: receive RX -> TGT reports.
        start_rx();
        break;
    case 9:
        // TGT position broadcast (not implemented yet): stay in RX.
        start_rx();
        break;
    default:
        start_rx();
        break;
    }
}

void cb_slot_inner_timer(void) {
    if (app_vars.echo_pending == FALSE) {
        return;
    }
    app_vars.echo_pending = FALSE;
    radio_txNow();
    //leds_debug_toggle();
}
