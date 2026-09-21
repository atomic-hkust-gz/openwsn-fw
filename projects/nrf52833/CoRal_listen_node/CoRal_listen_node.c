/**
\brief CoRal TDMA listen node.

Stays in RX permanently. Parses the same report PDU as the target and
prints each received report over UART (ASCII).

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
#define LENGTH_PACKET   125+LENGTH_BLE_CRC
#define CHANNEL         17

#define ENABLE_DF       1
#define DEBUG_RADIO_PIN 11

#define NODE_ID_REF       0
#define NODE_ID_TGT       1

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

#define ANCHOR_ID_MIN     2
#define ANCHOR_ID_MAX     9

#define UART_LINE_MAX     160

//=========================== variables =======================================

const static uint8_t ble_device_addr[6] = {
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

typedef enum {
    APP_STATE_RX          = 0x02,
    APP_STATE_OFF         = 0x04,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
    bool     valid;
    uint8_t  anchor_id;
    uint8_t  slot_number;
    int16_t  pos_x;
    int16_t  pos_y;
    float    intercept;
    float    slope;
    int32_t  time_diff;
    uint32_t intercept_bits;
    uint32_t slope_bits;
    uint8_t  next_ch;
    int8_t   rssi;
} anchor_report_t;

typedef struct {
                app_state_t     state;

                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;

                anchor_report_t pending_report;
     volatile   bool            uart_pending;

                uint8_t         uart_line[UART_LINE_MAX];
                uint16_t        uart_line_len;
                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);
void     start_rx(void);
void     save_anchor_report(void);
void     uart_send_pending_report(void);
void     nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
static bool is_valid_coral_pdu(void);
static bool is_anchor_report(void);
static uint32_t get_u32_le(uint8_t *p);
static int16_t  get_i16_le(uint8_t *p);
static uint16_t uart_append_str(uint8_t *dst, uint16_t pos, uint16_t max, const char *s);
static uint16_t uart_append_u32(uint8_t *dst, uint16_t pos, uint16_t max, uint32_t v);
static uint16_t uart_append_i32(uint8_t *dst, uint16_t pos, uint16_t max, int32_t v);
static uint16_t uart_append_hex32(uint8_t *dst, uint16_t pos, uint16_t max, uint32_t v);

//=========================== main ============================================

int mote_main(void) {

    memset(&app_vars, 0, sizeof(app_vars_t));

    board_init();

    radio_rfOff();
    app_vars.state        = APP_STATE_OFF;
    app_vars.uart_pending = FALSE;
    app_vars.uartDone     = 1;

    nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);
#if ENABLE_DF == 1
    radio_configure_direction_finding_antenna_switch();
    radio_configure_direction_finding_manual_AoA();
#endif

    uart_setCallbacks(cb_uartTxDone, cb_uartRxCb);
    uart_enableInterrupts();

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    start_rx();

    while (1) {
        if (app_vars.uart_pending == TRUE) {
            app_vars.uart_pending = FALSE;
            uart_send_pending_report();
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

static uint32_t get_u32_le(uint8_t *p) {
    return  ((uint32_t)p[0]      ) |
            ((uint32_t)p[1] <<  8) |
            ((uint32_t)p[2] << 16) |
            ((uint32_t)p[3] << 24);
}

static int16_t get_i16_le(uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t uart_append_str(uint8_t *dst, uint16_t pos, uint16_t max, const char *s) {
    while ((*s != 0) && (pos + 1 < max)) {
        dst[pos++] = (uint8_t)(*s++);
    }
    return pos;
}

static uint16_t uart_append_u32(uint8_t *dst, uint16_t pos, uint16_t max, uint32_t v) {
    char     tmp[11];
    uint8_t  n;
    uint8_t  i;

    if (v == 0) {
        return uart_append_str(dst, pos, max, "0");
    }

    n = 0;
    while ((v > 0) && (n < sizeof(tmp))) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    for (i = n; i > 0; i--) {
        if (pos + 1 >= max) {
            break;
        }
        dst[pos++] = (uint8_t)tmp[i - 1];
    }
    return pos;
}

static uint16_t uart_append_i32(uint8_t *dst, uint16_t pos, uint16_t max, int32_t v) {
    if (v < 0) {
        pos = uart_append_str(dst, pos, max, "-");
        return uart_append_u32(dst, pos, max, (uint32_t)(-v));
    }
    return uart_append_u32(dst, pos, max, (uint32_t)v);
}

static uint16_t uart_append_hex32(uint8_t *dst, uint16_t pos, uint16_t max, uint32_t v) {
    const char *hex = "0123456789ABCDEF";
    int8_t      i;

    pos = uart_append_str(dst, pos, max, "0x");
    for (i = 7; i >= 0; i--) {
        if (pos + 1 >= max) {
            break;
        }
        dst[pos++] = (uint8_t)hex[(v >> (4 * i)) & 0xF];
    }
    return pos;
}

void save_anchor_report(void) {
    uint8_t         anchor_id;
    anchor_report_t *rpt;

    anchor_id = app_vars.rxpk_packet[PKT_IDX_NODE_ID];
    if ((anchor_id < ANCHOR_ID_MIN) || (anchor_id > ANCHOR_ID_MAX)) {
        return;
    }

    rpt = &app_vars.pending_report;
    memset(rpt, 0, sizeof(anchor_report_t));

    rpt->valid       = TRUE;
    rpt->anchor_id   = anchor_id;
    rpt->slot_number = app_vars.rxpk_packet[PKT_IDX_SLOT];
    rpt->pos_x       = get_i16_le(&app_vars.rxpk_packet[PKT_IDX_POS_X]);
    rpt->pos_y       = get_i16_le(&app_vars.rxpk_packet[PKT_IDX_POS_Y]);
    memcpy(&rpt->intercept, &app_vars.rxpk_packet[PKT_IDX_INTERCEPT], sizeof(float));
    memcpy(&rpt->slope,     &app_vars.rxpk_packet[PKT_IDX_SLOPE],     sizeof(float));
    memcpy(&rpt->time_diff, &app_vars.rxpk_packet[PKT_IDX_TIME_DIFF], sizeof(int32_t));
    rpt->intercept_bits = get_u32_le(&app_vars.rxpk_packet[PKT_IDX_INTERCEPT]);
    rpt->slope_bits     = get_u32_le(&app_vars.rxpk_packet[PKT_IDX_SLOPE]);
    rpt->next_ch        = app_vars.rxpk_packet[PKT_IDX_NEXT_CH];
    rpt->rssi           = app_vars.rxpk_rssi;

    app_vars.uart_pending = TRUE;
}

void uart_send_pending_report(void) {
    uint16_t         pos;
    anchor_report_t *rpt;

    if (app_vars.pending_report.valid == FALSE) {
        return;
    }

    rpt = &app_vars.pending_report;
    pos = 0;

    // Example:
    // RPT id=2 slot=1 x=0 y=0 intercept=0x3F800000 slope=0x00000000 td=3200 rssi=-45
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, "RPT id=");
    pos = uart_append_u32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->anchor_id);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " slot=");
    pos = uart_append_u32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->slot_number);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " x=");
    pos = uart_append_i32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->pos_x);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " y=");
    pos = uart_append_i32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->pos_y);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " intercept=");
    pos = uart_append_hex32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->intercept_bits);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " slope=");
    pos = uart_append_hex32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->slope_bits);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " td=");
    pos = uart_append_i32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->time_diff);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, " rssi=");
    pos = uart_append_i32(app_vars.uart_line, pos, UART_LINE_MAX, rpt->rssi);
    pos = uart_append_str(app_vars.uart_line, pos, UART_LINE_MAX, "\r\n");

    app_vars.uart_line_len         = pos;
    app_vars.uart_lastTxByteIndex  = 0;
    app_vars.uartDone              = 0;
    uart_writeByte(app_vars.uart_line[0]);
    while (app_vars.uartDone == 0) {
        board_sleep();
    }

    rpt->valid = FALSE;
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
    app_dbg.num_endFrame++;

    if (app_vars.state != APP_STATE_RX) {
        start_rx();
        return;
    }

    radio_getReceivedFrame(
        app_vars.rxpk_packet,
        &app_vars.rxpk_packet_len,
        sizeof(app_vars.rxpk_packet),
        &app_vars.rxpk_rssi,
        &app_vars.rxpk_lqi,
        &app_vars.rxpk_crc
    );

    if (is_anchor_report() == TRUE) {
        save_anchor_report();
        leds_error_toggle();
    }

    start_rx();
}

void cb_uartTxDone(void) {
    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex < app_vars.uart_line_len) {
        uart_writeByte(app_vars.uart_line[app_vars.uart_lastTxByteIndex]);
    } else {
        app_vars.uartDone = 1;
    }
}

uint8_t cb_uartRxCb(void) {
    (void)uart_readByte();
    return 0;
}
