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
#include "timer.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         17              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
//#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LEN_UART_BUFFER (NUM_SAMPLES*4+10)
#define LENGTH_SERIAL_FRAME  127            // length of the serial frame

#define ENABLE_DF       1

#define DEBUG_RADIO_PIN 11

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
                
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;

                uint8_t         prob;
                
                uint8_t         pkt_sqn;
                uint8_t         pkt_inner_sqn;


                uint32_t        inner0_pkt_sample_buffer[NUM_SAMPLES];
                uint32_t        inner1_pkt_sample_buffer[NUM_SAMPLES];
                uint32_t        inner2_pkt_sample_buffer[NUM_SAMPLES];
                uint32_t        inner3_pkt_sample_buffer[NUM_SAMPLES];
                uint32_t        inner4_pkt_sample_buffer[NUM_SAMPLES];
                uint32_t        tx2_pkt_sample_buffer[NUM_SAMPLES];
                
                uint32_t        inner0_pkt_done_timestamp;
                uint32_t        inner1_pkt_done_timestamp;
                uint32_t        inner2_pkt_done_timestamp;
                uint32_t        inner3_pkt_done_timestamp;
                uint32_t        inner4_pkt_done_timestamp;
                uint32_t        tx2_pkt_done_timestamp;

                uint8_t         inner0_pkt_sqn;
                uint8_t         inner1_pkt_sqn;
                uint8_t         inner2_pkt_sqn;
                uint8_t         inner3_pkt_sqn;
                uint8_t         inner4_pkt_sqn;
                uint8_t         tx2_pkt_sqn;


                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];

                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;

                uint8_t         rxpk_done;

                uint8_t         rxpk_buf[LENGTH_PACKET];
                uint8_t         rxpk_freq_offset;
                uint8_t         rxpk_len;
                uint8_t         rxpk_num;

                bool            tx_done;
                uint8_t         tx_packet_sqn;
                uint32_t        tx_done_timestamp;


                //uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void      cb_startFrame(PORT_TIMER_WIDTH timestamp);
void      cb_endFrame(PORT_TIMER_WIDTH timestamp);

void      cb_uartTxDone(void);
uint8_t   cb_uartRxCb(void);


uint32_t phase_linear_regression_correction(const uint32_t *array, size_t start_index, size_t end_index)

void nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
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

    app_vars.prob = 1;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();
    
    timer_init();
    timer_start();
    
#if ENABLE_DF == 1
    //antenna_CHW_rx_switch_init();
    radio_configure_direction_finding_antenna_switch();
    //set_antenna_CHW_switches();
#endif

    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

    nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);

    // add radio callback functions
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_TX);

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual_AoA();
#endif

    // switch in RX by default
    radio_rxEnable();
    radio_rxNow();


    while(1) {

        // wait for timer to elapse
        app_vars.rxpk_done = 0;
        while (app_vars.rxpk_done==0) {
            continue;
        }

        leds_error_toggle();
        if (app_vars.rxpk_crc && ENABLE_DF) {

            app_vars.uartDone = 0;
            trans_sample_serials(app_vars.inner0_pkt_sample_buffer, app_vars.inner0_pkt_done_timestamp, app_vars.inner0_pkt_sqn, 0);

            while (app_vars.uartDone == 0) {
                continue;
            }
            
            app_vars.uartDone = 0;
            trans_sample_serials(app_vars.inner1_pkt_sample_buffer, app_vars.inner1_pkt_done_timestamp, app_vars.inner1_pkt_sqn, 1);
            while (app_vars.uartDone == 0) {
                continue;
            }
            
            app_vars.uartDone = 0;
            trans_sample_serials(app_vars.inner2_pkt_sample_buffer, app_vars.inner2_pkt_done_timestamp, app_vars.inner2_pkt_sqn, 2);
            while (app_vars.uartDone == 0) {
                continue;
            }

            app_vars.uartDone = 0;
            trans_sample_serials(app_vars.inner3_pkt_sample_buffer, app_vars.inner3_pkt_done_timestamp, app_vars.inner3_pkt_sqn, 3);
            while (app_vars.uartDone == 0) {
                continue;
            }
            
            app_vars.uartDone = 0;
            trans_sample_serials(app_vars.inner4_pkt_sample_buffer, app_vars.inner4_pkt_done_timestamp, app_vars.inner4_pkt_sqn, 4);
            while (app_vars.uartDone == 0) {
                continue;
            }

            trans_sample_serials(app_vars.tx2_pkt_sample_buffer, app_vars.tx2_pkt_done_timestamp, app_vars.tx2_pkt_sqn, 5);
        }

    }
}

//=========================== private =========================================


//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_START_FRAME;

    //leds_sync_on();
    // update debug stats
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {

    //NRF_P0->OUTSET =  1 << DEBUG_RADIO_PIN;
    //NRF_P0->OUTCLR =  1 << DEBUG_RADIO_PIN;

    bool     expectedFrame;
    //uint8_t  i;

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

        //app_vars.tx_done_timestamp = timestamp;
        //app_vars.pkt_sqn = app_vars.rxpk_buf[33];
        app_vars.pkt_inner_sqn = app_vars.rxpk_buf[32];

        switch (app_vars.pkt_inner_sqn) {
        case 0:
            app_vars.inner0_pkt_done_timestamp = timestamp;
            app_vars.num_samples = radio_get_df_samples(app_vars.inner0_pkt_sample_buffer,NUM_SAMPLES);
            app_vars.inner0_pkt_sqn = app_vars.rxpk_buf[33];

            radio_rxEnable();
            radio_rxNow();
            return;


        case 1:
            app_vars.inner1_pkt_done_timestamp = timestamp;
            app_vars.num_samples = radio_get_df_samples(app_vars.inner1_pkt_sample_buffer,NUM_SAMPLES);
            app_vars.inner1_pkt_sqn = app_vars.rxpk_buf[33];

            radio_rxEnable();
            radio_rxNow();
            return;

        case 2:
            app_vars.inner2_pkt_done_timestamp = timestamp;
            app_vars.num_samples = radio_get_df_samples(app_vars.inner2_pkt_sample_buffer,NUM_SAMPLES);
            app_vars.inner2_pkt_sqn = app_vars.rxpk_buf[33];


            radio_rxEnable();
            radio_rxNow();
            return;

        case 3:
            app_vars.inner3_pkt_done_timestamp = timestamp;
            app_vars.num_samples = radio_get_df_samples(app_vars.inner3_pkt_sample_buffer,NUM_SAMPLES);
            app_vars.inner3_pkt_sqn = app_vars.rxpk_buf[33];

            radio_rxEnable();
            radio_rxNow();
            return;

        case 4:
            app_vars.inner4_pkt_done_timestamp = timestamp;
            app_vars.num_samples = radio_get_df_samples(app_vars.inner4_pkt_sample_buffer,NUM_SAMPLES);
            app_vars.inner4_pkt_sqn = app_vars.rxpk_buf[33];
            
            app_vars.rxpk_done = 1;

            radio_rxEnable();
            radio_rxNow();
            return;

        case 5:
            app_vars.tx2_pkt_done_timestamp = timestamp;
            app_vars.num_samples = radio_get_df_samples(app_vars.tx2_pkt_sample_buffer,NUM_SAMPLES);
            app_vars.tx2_pkt_sqn = app_vars.rxpk_buf[33];

            app_vars.rxpk_done = 6;         // when received pkt from tx2, start sending data to PC

            radio_rxEnable();
            radio_rxNow();
            return;

        default:
            radio_rxEnable();
            radio_rxNow();
            return;
        }

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


void trans_sample_serials(uint32_t sample_buffer[NUM_SAMPLES], uint32_t done_timestamp, uint8_t pkt_sqn, uint8_t pkt_inner_sqn) {
    
    uint16_t i;
    
    for (i=0;i<app_vars.num_samples;i++) {
        app_vars.uart_buffer_to_send[4*i+0] = (sample_buffer[i] >>24) & 0x000000ff;
        app_vars.uart_buffer_to_send[4*i+1] = (sample_buffer[i] >>16) & 0x000000ff;
        app_vars.uart_buffer_to_send[4*i+2] = (sample_buffer[i] >> 8) & 0x000000ff;
        app_vars.uart_buffer_to_send[4*i+3] = (sample_buffer[i] >> 0) & 0x000000ff;
    }

    app_vars.uart_buffer_to_send[352] = (done_timestamp >> 24) & 0x000000ff;
    app_vars.uart_buffer_to_send[353] = (done_timestamp >> 16) & 0x000000ff;
    app_vars.uart_buffer_to_send[354] = (done_timestamp >>  8) & 0x000000ff;
    app_vars.uart_buffer_to_send[355] = (done_timestamp >>  0) & 0x000000ff;

    app_vars.uart_buffer_to_send[356] = pkt_sqn;
    app_vars.uart_buffer_to_send[357] = pkt_inner_sqn;

    app_vars.uart_buffer_to_send[358]     = 0xff;
    app_vars.uart_buffer_to_send[359]     = 0xff; 
    app_vars.uart_buffer_to_send[360]     = 0xff;
    app_vars.uart_buffer_to_send[361]     = 0xff;
    app_vars.uart_lastTxByteIndex = 0;
            
    leds_debug_toggle();
    uart_writeByte(app_vars.uart_buffer_to_send[app_vars.uart_lastTxByteIndex]);
}