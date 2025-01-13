/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

The board running this program will send a packet on channel CHANNEL every
TIMER_PERIOD ticks. The packet contains LENGTH_PACKET bytes. The first byte
is the packet number, which increments for each transmitted packet. The
remainder of the packet contains an incrementing bytes.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
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

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         0              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
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

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                uint8_t         flags;
                app_state_t     state;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint32_t        sample_buffer[NUM_SAMPLES];
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
                uint8_t         rxpk_done;
                uint8_t         rxpk_buf[LENGTH_PACKET];
                uint8_t         rxpk_freq_offset;
                uint8_t         rxpk_len;
                uint8_t         rxpk_num;

                int8_t         estimate_angle;



                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);
void     send_string(const char* str);


void     assemble_ibeacon_packet(void);

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
    uint8_t antenna_id;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    Complex steer_vector_array1[180];
    Complex steer_vector_array2[180];
    
    memcpy(steer_vector_array1, update_steering_vector1(steer_vector_array1), 180);
    memcpy(steer_vector_array1, update_steering_vector1(steer_vector_array1), 180);

    // initialize board
    board_init();
    
#if ENABLE_DF == 1
    //antenna_CHW_rx_switch_init();
    radio_configure_direction_finding_antenna_switch();
    //set_antenna_CHW_switches();
#endif

    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

    // add radio callback functions
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    app_vars.packet_len = sizeof(app_vars.packet);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_TX);

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual_AoD();
#endif

    // switch in RX by default
    radio_rxEnable();
    radio_rxNow();


    while(1) {

        // wait for timer to elapse
        app_vars.rxpk_done = 0;
        while (app_vars.rxpk_done==0) {
            board_sleep();
        }

        leds_error_toggle();
        if (app_vars.rxpk_crc && ENABLE_DF) {
            
            app_vars.num_samples = radio_get_df_samples(app_vars.sample_buffer,NUM_SAMPLES);
            
            sample_array_int_t sample_array_int;

            memset( &sample_array_int, 0, sizeof(sample_array_int) );

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

            // record the samples
            for (i=0;i<app_vars.num_samples;i++) {
                app_vars.uart_buffer_to_send[4*i+0] = (app_vars.sample_buffer[i] >>24) & 0x000000ff;
                app_vars.uart_buffer_to_send[4*i+1] = (app_vars.sample_buffer[i] >>16) & 0x000000ff;
                app_vars.uart_buffer_to_send[4*i+2] = (app_vars.sample_buffer[i] >> 8) & 0x000000ff;
                app_vars.uart_buffer_to_send[4*i+3] = (app_vars.sample_buffer[i] >> 0) & 0x000000ff;
            }

            // recoard rssi

            sign = (app_vars.rxpk_rssi & 0x80) >> 7;
                if (sign){
                    read = 0xff - (uint8_t)(app_vars.rxpk_rssi) + 1;
                } else {
                    read = app_vars.rxpk_rssi;
                }

                if (sign) {
                    app_vars.uart_buffer_to_send[4*i+0] = '-';
                } else {
                    app_vars.uart_buffer_to_send[4*i+0] = '+';
                }
            app_vars.uart_buffer_to_send[4*i+1] = '0'+read/100;
            app_vars.uart_buffer_to_send[4*i+2] = '0'+read/10;
            app_vars.uart_buffer_to_send[4*i+3] = '0'+read%10;

            //app_vars.uart_buffer_to_send[4*i+0]     = app_vars.rxpk_rssi;
            // record scum settings for transmitting
            //app_vars.uart_buffer_to_send[4*i+1]     = app_vars.packet[3];
            //app_vars.uart_buffer_to_send[4*i+2]     = app_vars.packet[4];
            //app_vars.uart_buffer_to_send[4*i+3]     = app_vars.packet[5];
            // frame split identifier
            app_vars.uart_buffer_to_send[4*i+4]     = app_vars.rxpk_buf[33];
            app_vars.uart_buffer_to_send[4*i+5]     = 0xff;
            app_vars.uart_buffer_to_send[4*i+6]     = 0xff;
            app_vars.uart_buffer_to_send[4*i+7]     = 0xff;

            app_vars.uart_lastTxByteIndex = 0;
            uart_writeByte(app_vars.uart_buffer_to_send[0]);

        }

    }
}

//=========================== private =========================================

void assemble_ibeacon_packet(void) {

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
    app_vars.packet[i++]  = 0x00;               // minor
    app_vars.packet[i++]  = 0x0f;
    app_vars.packet[i++]  = TXPOWER;            // power level
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
    bool     expectedFrame;
    uint8_t  i;

    // set flag
    //app_vars.flags |= APP_FLAG_END_FRAME;

    // update debug stats
    app_dbg.num_endFrame++;

    memset(&app_vars.rxpk_buf[0],0,LENGTH_PACKET);
    
    app_vars.rxpk_len = sizeof(app_vars.rxpk_buf);

    radio_getReceivedFrame(
        app_vars.rxpk_buf,
        &app_vars.rxpk_len,
        sizeof(app_vars.rxpk_buf),
        &app_vars.rxpk_rssi,
        &app_vars.rxpk_lqi,
        &app_vars.rxpk_crc
    );

    // check the frame is sent by radio_tx project
    expectedFrame = TRUE;

    if (app_vars.rxpk_len>LENGTH_PACKET){
        expectedFrame = FALSE;
    } else {

        if(app_vars.rxpk_buf[0]!=0x42){
            expectedFrame = FALSE;
        }
    }
    
    if (expectedFrame){
        app_vars.rxpk_done = 1;
    }
    //leds_debug_toggle();

    // keep listening (needed for at86rf215 radio)
    radio_rxEnable();
    radio_rxNow();

    // led
    //leds_sync_off();

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