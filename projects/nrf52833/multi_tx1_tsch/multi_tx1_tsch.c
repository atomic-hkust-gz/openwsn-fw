/**
\brief This program is the tx1 for AMUNA
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
#define DEBUG_SLOT_PIN 12

#define NUM_SLOTS                 5
#define SLOT_DURATION             (16000000/200)*100                              //5ms   @ (16000000/200)
#define TX_ONE_SEND_OFFSET        (16000000/200)*2                                //5ms   @ (16000000/200)      start transmit pkt at 10ms 
#define TX_TWO_SEND_OFFSET        (16000000/200)*2 + (16000000/5000)*0.9            //200us @ (16000000/5000)     start transmit pkt at 10ms + 200us
#define ONE_MS                    (16000000/1000)                                 //1ms          
#define CALIBRATE_PKT_NUM         4     // The number of calibration pkt is value + 1
 
#define  TRANSMITTER_ONE_LAST_EUI       0x85
#define  TRANSMITTER_TWO_LAST_EUI       0x7e
//=========================== variables =======================================

typedef enum {
    BOARD_ROLE_TX_ONE   = 0x01,
    BOARD_ROLE_TX_TWO   = 0x02,
} board_role;

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
                bool            get_sync;
                board_role      role;
                
                uint8_t         slot_timerId;
                uint8_t         inner_rxtimerId;
                uint8_t         inner_txtimerId;

                app_state_t     state;

                uint8_t         slot_offset;
                uint8_t         pkt_sqn;
                uint8_t         inner_pkt_id;
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

void     cb_slot_timer(void);
void     cb_inner_slot_rxtimer(void);
void     cb_inner_slot_txtimer(void);

void     assemble_ibeacon_packet(uint8_t, board_role);
void     nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {

    uint8_t board_eui[8];
    eui64_get(board_eui);

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));
    
    if (board_eui[7] == TRANSMITTER_ONE_LAST_EUI) {
        app_vars.role = BOARD_ROLE_TX_ONE;
    } else if (board_eui[7] == TRANSMITTER_TWO_LAST_EUI) {
        app_vars.role = BOARD_ROLE_TX_TWO;
    }

    uint16_t i;

    // initialize board
    board_init();

    radio_rfOff();
    app_vars.state = APP_STATE_OFF;

    nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);
    nrf_gpio_cfg_output(0, DEBUG_SLOT_PIN);
#if ENABLE_DF == 1
    //antenna_CHW_rx_switch_init();
    radio_configure_direction_finding_antenna_switch();
    radio_configure_direction_finding_manual_AoA();
    //set_antenna_CHW_switches();
#endif

    // add radio callback functions
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    app_vars.slot_timerId = 0;
    app_vars.inner_rxtimerId = 1;
    app_vars.inner_txtimerId = 2;
    app_vars.inner_pkt_id = 0;
    app_vars.get_sync = FALSE;
    app_vars.slot_offset = NUM_SLOTS-1;
    
    timer_set_callback(app_vars.slot_timerId, cb_slot_timer);
    
    if (app_vars.role == BOARD_ROLE_TX_ONE) {
        timer_set_callback(app_vars.inner_txtimerId, cb_inner_slot_txtimer);
    }
    
    if (app_vars.role == BOARD_ROLE_TX_TWO) {
        timer_set_callback(app_vars.inner_txtimerId, cb_inner_slot_txtimer);
        timer_set_callback(app_vars.inner_rxtimerId, cb_inner_slot_rxtimer);
    }



    timer_init();
    timer_start();
    
    if (app_vars.role == BOARD_ROLE_TX_ONE) {
        timer_capture_now(0);
        NRF_P0->OUTSET =  1 << DEBUG_SLOT_PIN;
        NRF_P0->OUTCLR =  1 << DEBUG_SLOT_PIN;
        app_vars.time_slotStartAt = timer_getCapturedValue(0) + SLOT_DURATION;
        timer_schedule(app_vars.slot_timerId, app_vars.time_slotStartAt);
    }

    if (app_vars.role == BOARD_ROLE_TX_TWO) {
        radio_rfOn();
        radio_setFrequency(CHANNEL, FREQ_RX);
        radio_rxEnable();
        app_vars.state = APP_STATE_RX;
        radio_rxNow();
    }


    while(1) {
        board_sleep();
    }
}

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t sqn, board_role role) {

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
    app_vars.packet[i++]  = 0x00;                        // major
    app_vars.packet[i++]  = 0xff;
    if (app_vars.role == BOARD_ROLE_TX_ONE) {
        app_vars.packet[i++]  = app_vars.inner_pkt_id;    // minor
    } else if (app_vars.role == BOARD_ROLE_TX_TWO) {
        app_vars.packet[i++]  = 5;
    }
    app_vars.packet[i++]  = sqn;                         // 34 byte
    app_vars.packet[i++]  = role;                        // tx id
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

    if (app_vars.state == APP_STATE_RX) {
        uint32_t time_stamp;
        timer_capture_now(app_vars.inner_txtimerId);
        time_stamp = timer_getCapturedValue(app_vars.inner_txtimerId);
        //received a BLE packet, only tx2 will go to here
        app_vars.isTargetPkt = FALSE;

        radio_rfOff();
        app_vars.state = APP_STATE_OFF;

        radio_getReceivedFrame(
            app_vars.rxpk_packet,
            &app_vars.rxpk_packet_len,
            sizeof(app_vars.rxpk_packet),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );
        
        // check is received packet is a legal system packet
        if (app_vars.rxpk_packet[0] == 0x42 & app_vars.rxpk_packet[1] == 0x21) {
            app_vars.isTargetPkt = TRUE;
        }

        if (app_vars.isTargetPkt & app_vars.rxpk_packet[32] == CALIBRATE_PKT_NUM) {
            // is the target packet is the first one, calibrate the slot sync
            //uint32_t time_stamp;
            //timer_capture_now(app_vars.slot_timerId);
            //time_stamp = timer_getCapturedValue(app_vars.slot_timerId);
            app_vars.time_slotStartAt = timestamp + SLOT_DURATION - TX_ONE_SEND_OFFSET - 3*ONE_MS;
            timer_schedule(app_vars.slot_timerId, app_vars.time_slotStartAt);
            
            app_vars.pkt_sqn = app_vars.rxpk_packet[33];
            timer_schedule(app_vars.inner_txtimerId, time_stamp + (16000000/5000)*0.9);
            app_vars.packet_len = sizeof(app_vars.packet);
            assemble_ibeacon_packet(app_vars.pkt_sqn, app_vars.role);

            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_TX);
            radio_loadPacket(app_vars.packet, LENGTH_PACKET);
            radio_txEnable();
            app_vars.state = APP_STATE_TX;
            return;

        }   else {
            // is not target packet, keep receiving
            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_RX);
            radio_rxEnable();
            app_vars.state = APP_STATE_RX;
            radio_rxNow();

            return;
        }
/*
        if (app_vars.slot_offset == 0) {

            if (app_vars.rxpk_packet[0] == 0x42 & app_vars.rxpk_packet[1] == 0x21) {
                app_vars.isTargetPkt = TRUE;
            }
            
            if (app_vars.isTargetPkt) {
                if (app_vars.get_sync == FALSE) {
                    app_vars.slot_offset = 1;
                }
                app_vars.pkt_sqn = app_vars.rxpk_packet[33]+1;
                uint32_t time_stamp;
                timer_capture_now(app_vars.slot_timerId);
                time_stamp = timer_getCapturedValue(app_vars.slot_timerId);
                app_vars.time_slotStartAt = time_stamp + SLOT_DURATION - TX_ONE_SEND_OFFSET;
                timer_schedule(app_vars.slot_timerId, app_vars.time_slotStartAt);
                return;
            }
        } else {
            if (app_vars.rxpk_packet[0] == 0x42 & app_vars.rxpk_packet[1] == 0x21 & app_vars.rxpk_packet[33] == CALIBRATE_PKT_NUM) {
                app_vars.isTargetPkt = TRUE;      //Check if received packet is a legal plast system packet
            }
            if (app_vars.isTargetPkt) {
                app_vars.isTargetPkt = TRUE;
                //app_vars.get_sync = TRUE;
                if (app_vars.get_sync == FALSE) {
                    app_vars.slot_offset = 1;
                }
                //app_vars.slot_offset = 1;
                app_vars.pkt_sqn = app_vars.rxpk_packet[33]+1;
                uint32_t time_stamp;
                timer_capture_now(app_vars.slot_timerId);
                time_stamp = timer_getCapturedValue(app_vars.slot_timerId);
                app_vars.time_slotStartAt = time_stamp + SLOT_DURATION - TX_ONE_SEND_OFFSET;
                timer_schedule(app_vars.slot_timerId, app_vars.time_slotStartAt);

                return;
            }          
        }

*/
        if (app_vars.isTargetPkt == FALSE) {
            // is not target packet, keep receiving
            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_RX);
            radio_rxEnable();
            app_vars.state = APP_STATE_RX;
            radio_rxNow();

            return;
        }    

    }

    if (app_vars.state == APP_STATE_TX & app_vars.role == BOARD_ROLE_TX_ONE) {

        uint32_t time_stamp;
        timer_capture_now(app_vars.inner_txtimerId);
        time_stamp = timer_getCapturedValue(app_vars.inner_txtimerId);
        radio_rfOff();
        app_vars.state = APP_STATE_OFF;
        //return;

        if (app_vars.inner_pkt_id < CALIBRATE_PKT_NUM) {
            app_vars.inner_pkt_id = app_vars.inner_pkt_id+1;
            timer_schedule(app_vars.inner_txtimerId, time_stamp + (16000000/5000)*0.9);
            app_vars.packet_len = sizeof(app_vars.packet);
            assemble_ibeacon_packet(app_vars.pkt_sqn, app_vars.role);

            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_TX);
            radio_loadPacket(app_vars.packet, LENGTH_PACKET);
            radio_txEnable();
            app_vars.state = APP_STATE_TX;

            return;
        } else {
            app_vars.inner_pkt_id = 0;
            return;
        }

    }

    if (app_vars.state == APP_STATE_TX & app_vars.role == BOARD_ROLE_TX_TWO) {
        radio_rfOff();
        app_vars.state = APP_STATE_OFF;
        return;        
    }

}

//void cb_timer(void) {
//    leds_error_toggle();
//    app_dbg.num_timer++;
//    radio_txNow();
//}

void cb_slot_timer(void) {
    leds_error_toggle();
    NRF_P0->OUTSET =  1 << DEBUG_SLOT_PIN;
    NRF_P0->OUTCLR =  1 << DEBUG_SLOT_PIN;
    app_vars.slot_offset = (app_vars.slot_offset+1)%NUM_SLOTS;

    app_vars.time_slotStartAt += SLOT_DURATION;
    timer_schedule(app_vars.slot_timerId, app_vars.time_slotStartAt);

    switch(app_vars.slot_offset) {
    default:
        if (app_vars.role == BOARD_ROLE_TX_ONE) {
            timer_schedule(app_vars.inner_txtimerId, app_vars.time_slotStartAt - SLOT_DURATION + TX_ONE_SEND_OFFSET);
            
            app_vars.inner_pkt_id = 0;
            app_vars.pkt_sqn += 1;
            app_vars.packet_len = sizeof(app_vars.packet);
            assemble_ibeacon_packet(app_vars.pkt_sqn, app_vars.role);

            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_TX);
            radio_loadPacket(app_vars.packet, LENGTH_PACKET);
            radio_txEnable();
            app_vars.state = APP_STATE_TX;
        }

        if (app_vars.role == BOARD_ROLE_TX_TWO) {
            //app_vars.get_sync = FALSE;
            timer_schedule(app_vars.inner_rxtimerId, app_vars.time_slotStartAt - SLOT_DURATION + TX_ONE_SEND_OFFSET + 2*ONE_MS);
        }
    break;
    //default:
    //    // except slot 0, tx1 and tx2 works similar, send a packet at different time
    //    // the pkt_sqn of tx2 is decided by packet received in slot 0
    //    if (app_vars.role == BOARD_ROLE_TX_ONE) {
    //        timer_schedule(app_vars.inner_txtimerId, app_vars.time_slotStartAt - SLOT_DURATION + TX_ONE_SEND_OFFSET);

    //        app_vars.packet_len = sizeof(app_vars.packet);
    //        assemble_ibeacon_packet(app_vars.pkt_sqn++, app_vars.role);

    //        radio_rfOn();
    //        radio_setFrequency(CHANNEL, FREQ_TX);
    //        radio_loadPacket(app_vars.packet, LENGTH_PACKET);
    //        radio_txEnable();
    //        app_vars.state = APP_STATE_TX;       
    //    }

    //    if (app_vars.role == BOARD_ROLE_TX_TWO) {
    //        //timer_schedule(app_vars.inner_txtimerId, app_vars.time_slotStartAt - SLOT_DURATION + TX_TWO_SEND_OFFSET);

    //        //app_vars.packet_len = sizeof(app_vars.packet);
    //        //assemble_ibeacon_packet(app_vars.pkt_sqn++, app_vars.role);

    //        radio_rfOn();
    //        radio_setFrequency(CHANNEL, FREQ_RX);
    //        //radio_loadPacket(app_vars.packet, LENGTH_PACKET);
    //        radio_rxEnable();
    //        app_vars.state = APP_STATE_RX;
    //        radio_rxNow();       
    //    }
    //break;
    }
}

//void cb_inner_slot_rxtimer(void) {

//}

void cb_inner_slot_txtimer(void) {
    leds_error_toggle();
    app_dbg.num_timer++;
    radio_txNow();
}

void cb_inner_slot_rxtimer(void) {
    leds_error_toggle();
    app_dbg.num_timer++;

    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;

    radio_rxNow();
}