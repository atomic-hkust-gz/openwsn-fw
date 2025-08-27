/**
\brief This program is for AoD anchor
The AoD anchor will preiodicly send a packet to AoD nodes.

Use one timer to schedule a periodic slot.
Use another timer to schedule a short time which used for control
when send the packet in one slot.

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Nov. 2024.
*/

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio_df.h"
#include "radio_CHW_df.h"
#include "timer.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         0              ///< 0~39

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LENGTH_SERIAL_FRAME  127              // length of the serial frame

#define ENABLE_DF       1

const static uint8_t ble_device_addr[6] = { 
   0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
   0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
   0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
   0xbd, 0x65, 0x3c, 0x73
};

#define SEND_DURATION     (16000000/200)*100        //5ms@ (16000000/200)
#define SEND_OFFSET       (16000000/5000)*0.3        //200us @ (16000000/5000)

//define debug GPIO
#define DEBUG_PORT           1
#define DEBUG_PIN0           10
#define DEBUG_RADIO_PIN      11 

//=========================== variables =======================================
typedef enum {
   APP_STATE_TX         = 0x01,
   APP_STATE_RX         = 0x02,
   APP_STATE_OFF        = 0x04,
} app_state_t;

typedef struct {
   uint8_t              num_startFrame;
   uint8_t              num_endFrame;
   uint8_t              num_timer;
   uint8_t              num_slot;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {

                uint8_t         slot_timerId;
                uint8_t         inner_timerId;
                app_state_t     state;

                uint8_t         slot_offset;
                uint8_t         pkt_sqn;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;

                uint8_t         rx_doneAt;
                uint8_t         tx_now;

                uint32_t       start_timestamp;
                uint32_t       end_timestamp;
                uint32_t       time_interval;

                bool           isTargetPkt;

} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_timer(void);

void     assemble_ibeacon_packet(uint8_t);
void nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
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

    // turn radio off
    radio_rfOff();
    app_vars.state = APP_STATE_OFF;

    #if ENABLE_DF == 1
    antenna_CHW_tx_switch_init();
    radio_configure_direction_finding_manual_AoD();
    #endif

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    //initial debugs GPIO
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_PIN0);
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_RADIO_PIN);


    timer0_init();
    timer0_start();
    timer0_set_callback(0, cb_timer);

    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_TX);
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;
    radio_rxNow();

    //sctimer_enable(app_vars.slot_timerId);

    // sleep
    while (1){
        app_vars.tx_now = 0;
        while (app_vars.tx_now == 0) {
            board_sleep();
        }

        timer0_capture_now(0);
        app_vars.start_timestamp = timer0_getCapturedValue(0);

        radio_txEnable();
        app_vars.state = APP_STATE_TX;

        radio_txNow();
        //continue;
    }
     
}
 

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t sqn) {

    uint8_t i;
    i=0;

    memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

    app_vars.packet[i++]  = 0x42;               // BLE ADV_NONCONN_IND (this is a must)
    app_vars.packet[i++]  = 0x21;               // Payload length
    app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
    app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
    app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
    app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
    app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
    app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

    app_vars.packet[i++]  = 0x1a;
    app_vars.packet[i++]  = 0xff;
    app_vars.packet[i++]  = 0x4c;
    app_vars.packet[i++]  = 0x00;

    app_vars.packet[i++]  = 0x02;
    app_vars.packet[i++]  = 0x15;
    memcpy(&app_vars.packet[i], &ble_uuid[0], 16);
    i                    += 16;
    app_vars.packet[i++]  = 0x00;               // major
    app_vars.packet[i++]  = 0xff;
    app_vars.packet[i++]  = 0x00;               //  minor
    app_vars.packet[i++]  = sqn;                //  34 byte
    app_vars.packet[i++]  = 0x02;               //  tx2 id
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {

    app_dbg.num_endFrame++;

    timer0_capture_now(0);

    if (app_vars.state == APP_STATE_RX) {
        
        app_vars.isTargetPkt = FALSE;

        radio_getReceivedFrame(
            app_vars.rxpk_packet,
            &app_vars.rxpk_packet_len,
            sizeof(app_vars.rxpk_packet),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );
        
        if (app_vars.rxpk_packet[0] == 0x42 & app_vars.rxpk_packet[1] == 0x21) {
            app_vars.isTargetPkt = TRUE;      //Check if received packet is a legal plast system packet
        }

        if (app_vars.isTargetPkt) {
            uint32_t endframe_timestamp = timer0_getCapturedValue(0);
            app_vars.time_slotStartAt = endframe_timestamp + SEND_OFFSET;
            timer0_schedule(0, app_vars.time_slotStartAt);

            app_vars.pkt_sqn = app_vars.rxpk_packet[33];
            app_vars.packet_len = sizeof(app_vars.packet);
            assemble_ibeacon_packet(app_vars.pkt_sqn);
            radio_setFrequency(CHANNEL, FREQ_TX);
            radio_loadPacket(app_vars.packet, LENGTH_PACKET);
        } else {
            radio_rxEnable();
            radio_rxNow();
        }

    }


    if (app_vars.state == APP_STATE_TX) {
        radio_rxEnable();
        app_vars.state = APP_STATE_RX;
        radio_rxNow();
    }
}

void cb_timer(void) {
    leds_error_toggle();
    app_dbg.num_timer++;
    radio_txEnable();
    app_vars.state = APP_STATE_TX;
    radio_txNow();
    //app_vars.tx_now = 1;
}