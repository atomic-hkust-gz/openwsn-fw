/**
\brief This program is for AoD node
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
#include "aod.h"
#include "math.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         0              ///< 0~39
#define BEACON_CHANNEL  20

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

#define NUM_SLOTS       5
#define SLOT_DURATION   (32768/200)*20  // 5ms@ (32768/200)
#define SENDING_OFFSET  (32768/1000)*20 // 1ms@ (32768/1000)
#define TURNON_OFFSET   (32768/2000)*20 // 0.5ms@ (32768/2000)

//define debug GPIO
#define DEBUG_PORT           1
#define DEBUG_SLOT_PIN       7
#define DEBUG_RADIO_PIN      8 

#define ONE_ANT_CONSTANT    -1.89
#define TWO_ANT_CONSTANT    -3.78

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
                bool            isSynced;
                bool            isTargetPkt;
                bool            got_sample;
                app_state_t     state;
                uint8_t         period_sqn;
                uint8_t         node_id;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;

                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint32_t        sample_buffer[NUM_SAMPLES];
                //uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                //uint16_t        uart_lastTxByteIndex;

                uint8_t         txpk_txNow;
                uint8_t         pkt_sqn;
                
                uint8_t         slot_timerId;
                uint8_t         inner_timerId;
                uint8_t         slot_offset;
                uint32_t        time_slotStartAt;

                uint32_t        capture_time;
                uint8_t         angle;

                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];

                int8_t          estimate_angle;
                uint8_t         antenna_array_id;

                uint8_t         current_time;

} app_vars_t;

app_vars_t app_vars;


//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_slot_timer(void);
void     cb_inner_slot_timer(void);

void     assemble_ibeacon_packet(uint8_t, int8_t, uint8_t);

Complex* update_steering_vector1(Complex* steer_vector_array1);
Complex* update_steering_vector2(Complex* steer_vector_array2);
uint16_t cal_angle(sample_array_int_t sample_array_int, Complex* steer_vector_array1, Complex* steer_vector_array2);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;

    uint8_t current_time;
    
    Complex steer_vector_array1[180];
    Complex steer_vector_array2[180];
    
    memcpy(steer_vector_array1, update_steering_vector1(steer_vector_array1), 180);
    memcpy(steer_vector_array1, update_steering_vector1(steer_vector_array1), 180);

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));
    app_vars.got_sample = FALSE;
    //set slot offset to any value untill sync
    app_vars.slot_offset = 10;
    app_vars.node_id = 2;
    
    sample_array_int_t sample_array_int;
    // initialize board
    board_init();

    // turn radio off
    radio_rfOff();

#if ENABLE_DF == 1
    radio_configure_direction_finding_antenna_switch();
    radio_configure_direction_finding_manual_AoD();
#endif
    
    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);

    app_vars.slot_timerId = 0;
    app_vars.inner_timerId = 1;
    
    //initial debugs GPIO
    //nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_SLOT_PIN);
    //nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_RADIO_PIN);
    // start sctimer
    sctimer_set_callback(app_vars.slot_timerId, cb_slot_timer);
    sctimer_set_callback(app_vars.inner_timerId, cb_inner_slot_timer);
    
    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;
    radio_rxNow();
    
    memset( &sample_array_int, 0, sizeof(sample_array_int) );

    // sleep
    //clocks_stop();
    while (1){
        
        //if (app_vars.slot_offset == 0 | app_vars.slot_offset == 2) {
        //    NRF_P1->OUTSET =  1 << DEBUG_SLOT_PIN;
        //} else {
        //    NRF_P1->OUTCLR =  1 << DEBUG_SLOT_PIN;
        //}

        //if (app_vars.state == APP_STATE_OFF) {
        //    NRF_P1->OUTCLR =  1 << DEBUG_RADIO_PIN;
        //} else {
        //    NRF_P1->OUTSET =  1 << DEBUG_RADIO_PIN;
        //}

        if (app_vars.got_sample == TRUE) {

            uint8_t i=0;
            for (i;i<8;i++) {
                sample_array_int.ref_Q[i] = (int16_t)((app_vars.sample_buffer[i] >> 16) & 0x0000FFFF);
                sample_array_int.ref_I[i] = (int16_t)(app_vars.sample_buffer[i]) & 0x0000FFFF;
            }
    
            for (i = 0;(9+i*8)<NUM_SAMPLES;i++) {
                sample_array_int.ant0_Q[i] = (int16_t)(app_vars.sample_buffer[9+i*8] >> 16) & 0x0000FFFF;
                sample_array_int.ant0_I[i] = (int16_t)(app_vars.sample_buffer[9+i*8]) & 0x0000FFFF;
            }

            for (i = 0;(11+i*8)<NUM_SAMPLES;i++) {
                sample_array_int.ant1_Q[i] = (int16_t)(app_vars.sample_buffer[11+i*8] >> 16) & 0x0000FFFF;
                sample_array_int.ant1_I[i] = (int16_t)(app_vars.sample_buffer[11+i*8]) & 0x0000FFFF;
            }

            for (i = 0;(13+i*8)<NUM_SAMPLES;i++) {
                sample_array_int.ant2_Q[i] = (int16_t)(app_vars.sample_buffer[13+i*8] >> 16) & 0x0000FFFF;
                sample_array_int.ant2_I[i] = (int16_t)(app_vars.sample_buffer[13+i*8]) & 0x0000FFFF;
            }

            for (i = 0;(15+i*8)<NUM_SAMPLES;i++) {
                sample_array_int.ant3_Q[i] = (int16_t)(app_vars.sample_buffer[15+i*8] >> 16) & 0x0000FFFF;
                sample_array_int.ant3_I[i] = (int16_t)(app_vars.sample_buffer[15+i*8]) & 0x0000FFFF;
            }

            app_vars.estimate_angle = cal_angle(sample_array_int, steer_vector_array1, steer_vector_array2);
            app_vars.got_sample = FALSE;
        }
        board_sleep();
        //clocks_stop();
        //app_vars.current_time = sctimer_readCounter();
    }
}

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t node_id, int8_t estimate_angle, uint8_t antenna_array_id) {

    uint8_t i;
    uint8_t sign;
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
    //app_vars.packet[i++]  = 0x00;               // minor
    app_vars.packet[i++]  = node_id;
    
    app_vars.packet[i++]  = estimate_angle;
    app_vars.packet[i++]  = antenna_array_id;
    app_vars.packet[i++]  = 0x00;               // power level
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    //only capture timestamp in rx start of frame
    if (app_vars.state == APP_STATE_RX) {
        app_vars.capture_time = timestamp;
    
    }

    app_dbg.num_startFrame++;
}


void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    
    app_dbg.num_endFrame++;

    if (app_vars.state == APP_STATE_RX) {
        // receive radio packet
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
        clocks_stop();
        app_vars.num_samples = radio_get_df_samples(app_vars.sample_buffer,NUM_SAMPLES);
        app_vars.antenna_array_id = app_vars.rxpk_packet[34];

        //sample_array_int = get_sample_array(sample_array_int, app_vars.sample_buffer, NUM_SAMPLES);

        // check if the frame is target one
        if (app_vars.rxpk_packet[0] == 0x42 & app_vars.rxpk_packet[1] == 0x21)
            app_vars.isTargetPkt = TRUE;
            app_vars.got_sample = TRUE;

        
        // is is target pkt, check if synced. if not, sync
        if (app_vars.isTargetPkt) {
            if (app_vars.isSynced) {
                app_vars.time_slotStartAt = app_vars.capture_time + SLOT_DURATION - SENDING_OFFSET;
                sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);
                //app_vars.estimate_angle = cal_angle(sample_array_int);
            } else {
        
                app_vars.slot_offset = 0;
                app_vars.time_slotStartAt = app_vars.capture_time + SLOT_DURATION - SENDING_OFFSET;
                sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);
                app_vars.isSynced = TRUE;
                //app_vars.estimate_angle = cal_angle(sample_array_int);
            }
            
            return;
        }

        // if not target pkt and not sync, keep listening. If not target pkt but synced, do nothing
        if (app_vars.isSynced == FALSE) {
            radio_rfOn();
            app_vars.state = APP_STATE_RX;
            radio_rxEnable();
            radio_rxNow();
        }
    }

    // if Radio in Tx state, do nothing
    if (app_vars.state == APP_STATE_TX) {
        radio_rfOff();
        app_vars.state = APP_STATE_OFF;
        clocks_stop();

        return;
    }
}

void cb_slot_timer(void) {

    leds_error_toggle();

    // update slot offset
    app_vars.slot_offset = (app_vars.slot_offset+1)%NUM_SLOTS;

    // schedule next slot

    app_vars.time_slotStartAt += SLOT_DURATION;
    sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);
    
   
    // check which slotoffset is right now

    switch(app_vars.slot_offset) {
    case 0:
   
        // set when to turn on the radio
        //clocks_start();
        sctimer_setCompare(app_vars.inner_timerId, app_vars.time_slotStartAt - SLOT_DURATION + SENDING_OFFSET-TURNON_OFFSET);
        //clocks_stop();
        
    break;
    //case 2:
        // send packet 
        // prepare to send
        
        // prepare packet
        //app_vars.packet_len = sizeof(app_vars.packet);
        //assemble_ibeacon_packet(app_vars.node_id, app_vars.estimate_angle, app_vars.antenna_array_id);

        ////prepare radio
        //radio_rfOn();
        //app_vars.state = APP_STATE_TX;
        //radio_setFrequency(BEACON_CHANNEL, FREQ_RX);

        //radio_loadPacket(app_vars.packet,LENGTH_PACKET);
        //radio_txEnable();

        //// no need to schedule a time
        //radio_txNow();

    //break;
    default:
        // choose slot to send the position information arrcording to node id
        if (app_vars.slot_offset == app_vars.node_id + 1) {
            //clocks_start();
            app_vars.packet_len = sizeof(app_vars.packet);
            assemble_ibeacon_packet(app_vars.node_id, app_vars.estimate_angle, app_vars.antenna_array_id);

            //prepare radio
            radio_rfOn();
            app_vars.state = APP_STATE_TX;
            radio_setFrequency(BEACON_CHANNEL, FREQ_RX);

            radio_loadPacket(app_vars.packet,LENGTH_PACKET);
            radio_txEnable();

            // no need to schedule a time
            radio_txNow();
        } else {
            //if not correct slot, turn off the radio and sleep
            radio_rfOff();
            app_vars.state = APP_STATE_OFF;
        }
       
    break;
    }
}

void cb_inner_slot_timer(void) {
    
    app_dbg.num_timer++;

    radio_rfOn();
    app_vars.state = APP_STATE_RX;
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    radio_rxNow();
}

