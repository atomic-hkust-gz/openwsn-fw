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

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         17              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
//#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+12)
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

#define SLOT_DURATION     (16000000/200)*100    //5ms@ (16000000/200)
#define SEND_OFFSET       (16000000/200)*10      //50ms
#define PKT_INTERVAL       (16000000/5000)*1        //200us @ (16000000/5000)

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
                
                uint8_t         channel;
                uint8_t         pkt_sqn;
                uint8_t         rx_packet_sqn;
                uint32_t        time_slotStartAt;
                uint8_t         slot_timerid;
                uint8_t         inner_timerid;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];

                uint16_t        uart_lastTxByteIndex;
      volatile  uint8_t         uartDone;
                uint8_t         rxpk_done;
                uint8_t         rxpk_buf[LENGTH_PACKET];
                uint8_t         rxpk_freq_offset;
                uint8_t         rxpk_len;
                uint8_t         rxpk_num;
                uint8_t         tx_now;

                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;

                uint32_t        tx_done_timestamp;
                uint32_t        rx_done_timestamp;
                uint32_t        time_interval;

                bool            isTargetPkt;

                uint32_t        tx1_sample_buffer[NUM_SAMPLES];
} app_vars_t;
 
app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_slot_timer(void);
void     cb_inner_slot_timer(void);

void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);

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

    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

    app_vars.channel = 0;

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
    
    app_vars.slot_timerid = 0;
    app_vars.inner_timerid = 1;

    timer_set_callback(app_vars.slot_timerid, cb_slot_timer);
    timer_set_callback(app_vars.inner_timerid, cb_inner_slot_timer);

    timer_capture_now(app_vars.slot_timerid);
    app_vars.time_slotStartAt = timer_getCapturedValue(app_vars.slot_timerid) + SLOT_DURATION;
    timer_schedule(app_vars.slot_timerid, app_vars.time_slotStartAt);
    timer_schedule(app_vars.inner_timerid, app_vars.time_slotStartAt + SEND_OFFSET);

    while(1) {
        app_vars.rxpk_done = 0;
        while (app_vars.rxpk_done == 0) {
            continue;
        }

        // if I get here, I just received target packet
        for (i=0;i<app_vars.num_samples;i++) {
            app_vars.uart_buffer_to_send[4*i+0] = (app_vars.tx1_sample_buffer[i] >>24) & 0x000000ff;
            app_vars.uart_buffer_to_send[4*i+1] = (app_vars.tx1_sample_buffer[i] >>16) & 0x000000ff;
            app_vars.uart_buffer_to_send[4*i+2] = (app_vars.tx1_sample_buffer[i] >> 8) & 0x000000ff;
            app_vars.uart_buffer_to_send[4*i+3] = (app_vars.tx1_sample_buffer[i] >> 0) & 0x000000ff;
        }
        app_vars.time_interval = app_vars.rx_done_timestamp - app_vars.tx_done_timestamp;

        app_vars.uart_buffer_to_send[352] = (app_vars.time_interval >> 24) & 0x000000ff;
        app_vars.uart_buffer_to_send[353] = (app_vars.time_interval >> 16) & 0x000000ff;
        app_vars.uart_buffer_to_send[354] = (app_vars.time_interval >>  8) & 0x000000ff;
        app_vars.uart_buffer_to_send[355] = (app_vars.time_interval >>  0) & 0x000000ff;
        
        app_vars.uart_buffer_to_send[356] = app_vars.rx_packet_sqn;
        app_vars.uart_buffer_to_send[357] = app_vars.channel;

        app_vars.uart_buffer_to_send[358]     = 0xff;
        app_vars.uart_buffer_to_send[359]     = 0xff; 
        app_vars.uart_buffer_to_send[360]     = 0xff;
        app_vars.uart_buffer_to_send[361]     = 0xff;
        app_vars.uart_buffer_to_send[362]     = 0xff; 
        app_vars.uart_buffer_to_send[363]     = 0xff;

        app_vars.uart_lastTxByteIndex = 0;
        uart_writeByte(app_vars.uart_buffer_to_send[0]);
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
     app_vars.packet[i++]  = (app_vars.channel+1)%20;   // next packet channel,     only use 0-20 BLE channel
     app_vars.packet[i++]  = sqn;                // 34 byte
     app_vars.packet[i++]  = 0x01;               // tx id
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
        app_vars.rx_done_timestamp = timestamp;
        app_vars.isTargetPkt = FALSE;

        radio_getReceivedFrame(
            app_vars.rxpk_buf,
            &app_vars.rxpk_len,
            sizeof(app_vars.rxpk_buf),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );
        
        app_vars.num_samples = radio_get_df_samples(app_vars.tx1_sample_buffer,NUM_SAMPLES);
        if (app_vars.rxpk_buf[0] == 0x42 && app_vars.rxpk_buf[1] == 0x21) {
            app_vars.isTargetPkt = TRUE;      //Check if received packet is a legal plast system packet
        }
        
        if (app_vars.isTargetPkt == TRUE) {
            app_vars.rx_packet_sqn = app_vars.rxpk_buf[33];
            app_vars.rxpk_done = 1;

            //only change next channel when received echo from rx
            app_vars.channel += 1;
            if (app_vars.channel > 19) {
                app_vars.channel = 0;
            }
        }
        radio_rfOff();
        app_vars.state = APP_STATE_OFF;
        
    
    }

    if (app_vars.state == APP_STATE_TX) {
        app_vars.tx_done_timestamp = timestamp;
        
        radio_setFrequency(app_vars.channel, FREQ_RX);
        radio_rxEnable();
        app_vars.state = APP_STATE_RX;
        radio_rxNow();
    }

}

void cb_slot_timer(void) {
    radio_rfOff();
    app_dbg.num_timer++;
    leds_error_toggle();
    app_vars.time_slotStartAt +=SLOT_DURATION;
    timer_schedule(app_vars.slot_timerid, app_vars.time_slotStartAt);


    radio_rfOn();

    
    radio_setFrequency(app_vars.channel, FREQ_TX);
   
    app_vars.packet_len = sizeof(app_vars.packet);
    app_vars.pkt_sqn++;
    assemble_ibeacon_packet(app_vars.pkt_sqn);
    radio_loadPacket(app_vars.packet, LENGTH_PACKET);

    radio_txEnable();
    app_vars.state = APP_STATE_TX;
}

void cb_inner_slot_timer(void) {
    //schedule next transmit 
    timer_schedule(app_vars.inner_timerid, app_vars.time_slotStartAt + SEND_OFFSET);

    //trigger transmit
    radio_txNow();
}

void cb_uartTxDone(void) {

   app_vars.uart_lastTxByteIndex++;
   if (app_vars.uart_lastTxByteIndex<LEN_UART_BUFFER) {
      uart_writeByte(app_vars.uart_buffer_to_send[app_vars.uart_lastTxByteIndex]);
   } else {
      app_vars.uartDone = 1;
   }
}

uint8_t cb_uartRxCb(void) {
   uint8_t byte;
   
   // read received byte
   byte = uart_readByte();
   
   // echo that byte over serial
   uart_writeByte(byte);
   
   return 0;
}