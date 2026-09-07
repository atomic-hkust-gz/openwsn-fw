/**
\brief CoRal TDMA reference node.

Owns a 500 ms slot grid (SLOT_DURATION) and transmits one BLE beacon
at t_slot = 10 ms of every slot so other nodes can keep time sync.

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

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         17              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
//#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)*2+7)
#define LENGTH_SERIAL_FRAME  127            // length of the serial frame

#define ENABLE_DF       1

uint16_t length = 0;

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

#define DEBUG_RADIO_PIN 11

#define SLOT_DURATION     ((16000000/200)*100)  // 500 ms @ 16 MHz
#define SYNC_OFFSET       ((16000000/200)*2)    // 10 ms into the slot

#define SLOT_TIMER_ID     0
#define INNER_TIMER_ID    1
#define CAPTURE_ID        2

#define NODE_ID_REF       0
#define NODE_POS_X        0
#define NODE_POS_Y        0
#define NEXT_IBEACON_CH   0                 // hopping later; stay on channel 0 for now

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
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
 
                 uint8_t         pkt_sqn;
                 uint32_t        time_slotStartAt;
 
                 uint8_t         packet[LENGTH_PACKET];
                 uint8_t         packet_len;
 
                 uint8_t         tx_now;
 } app_vars_t;
 
 app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_timer(void);
void     cb_slot_inner_timer(void);
void     assemble_ibeacon_packet(uint8_t);
void     nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    radio_rfOff();
    app_vars.state = APP_STATE_OFF;

    nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);
#if ENABLE_DF == 1
    //antenna_CHW_rx_switch_init();
    radio_configure_direction_finding_antenna_switch();
    radio_configure_direction_finding_manual_AoA();
    //set_antenna_CHW_switches();
#endif

    // add radio callback functions
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    //sctimer_set_callback(0, cb_timer);
    //timer_capture_now(0);
    //app_vars.time_slotStartAt = sctimer_readCounter() + SEND_DURATION;
    //sctimer_setCompare(0, app_vars.time_slotStartAt);

    timer_init();
    timer_start();

    timer_set_callback(SLOT_TIMER_ID,  cb_timer);
    timer_set_callback(INNER_TIMER_ID, cb_slot_inner_timer);
    timer_capture_now(CAPTURE_ID);
    app_vars.time_slotStartAt = timer_getCapturedValue(CAPTURE_ID);
    timer_schedule(SLOT_TIMER_ID,  app_vars.time_slotStartAt + SLOT_DURATION);
    timer_schedule(INNER_TIMER_ID, app_vars.time_slotStartAt + SYNC_OFFSET);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_TX);
    app_vars.packet_len = sizeof(app_vars.packet);
    assemble_ibeacon_packet(app_vars.pkt_sqn);
    radio_loadPacket(app_vars.packet, LENGTH_PACKET);

    radio_txEnable();
    app_vars.state = APP_STATE_TX;



    while(1) {
        board_sleep();
    }
}

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t slot_number) {

     uint8_t i;
     int16_t pos_x;
     int16_t pos_y;

     i=0;
     pos_x = NODE_POS_X;
     pos_y = NODE_POS_Y;

     memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

     app_vars.packet[i++]  = 0x42;               // S0: BLE ADV_NONCONN_IND (filter)
     app_vars.packet[i++]  = 0x00;               // Payload length, filled after payload
     app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0 (filter)
     app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
     app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
     app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
     app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
     app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

     app_vars.packet[i++]  = NODE_ID_REF;        // node id, reference = 0
     app_vars.packet[i++]  = slot_number;        // slot number
     app_vars.packet[i++]  = (uint8_t)(pos_x & 0xff);        // node x, little-endian int16
     app_vars.packet[i++]  = (uint8_t)((pos_x >> 8) & 0xff);
     app_vars.packet[i++]  = (uint8_t)(pos_y & 0xff);        // node y, little-endian int16
     app_vars.packet[i++]  = (uint8_t)((pos_y >> 8) & 0xff);
     app_vars.packet[i++]  = NEXT_IBEACON_CH;    // next beacon channel

     app_vars.packet[1]    = i - 2;              // LENGTH = payload bytes after S0/Length
}
//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_START_FRAME;

    //leds_sync_on();
    // update debug stats
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    app_dbg.num_endFrame++;

    if (app_vars.state == APP_STATE_TX) {
        // Next TX time is kept on the 500 ms slot grid in cb_timer / cb_slot_inner_timer.
        // Do not re-arm from the radio end timestamp, or the period drifts by airtime.
        app_vars.pkt_sqn++;
        assemble_ibeacon_packet(app_vars.pkt_sqn);
        radio_loadPacket(app_vars.packet, LENGTH_PACKET);

        radio_txEnable();
        app_vars.state = APP_STATE_TX;
    }
}

void cb_timer(void) {
    leds_error_toggle();
    app_dbg.num_timer++;

    app_vars.time_slotStartAt += SLOT_DURATION;
    timer_schedule(SLOT_TIMER_ID,  app_vars.time_slotStartAt + SLOT_DURATION);
    timer_schedule(INNER_TIMER_ID, app_vars.time_slotStartAt + SYNC_OFFSET);
}

void cb_slot_inner_timer(void) {
    radio_txNow();
}
