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
#define CHANNEL         0              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
//#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LEN_UART_BUFFER (7)
#define LENGTH_SERIAL_FRAME  127            // length of the serial frame

#define ENABLE_DF       1

#define SLOT_DURATION   (16000000/5000)
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


//define debug GPIO
#define DEBUG_PORT           1
#define DEBUG_PIN0           8
#define DEBUG_RADIO_PIN      11 


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

                uint32_t        tx1_sample_buffer[NUM_SAMPLES];
                uint32_t        tx2_sample_buffer[NUM_SAMPLES];
                
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];

                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
                uint8_t         rxpk_done;
                uint8_t         rxpk_buf[LENGTH_PACKET];
                uint8_t         rxpk_freq_offset;
                uint8_t         rxpk_len;
                uint8_t         rxpk_num;

                bool            tx1_done;
                uint8_t         tx1_packet_sqn;
                uint32_t        tx1_done_timestamp;

                bool            tx2_done;
                uint8_t         tx2_packet_sqn;
                uint32_t        tx2_done_timestamp;

                uint32_t        time_interval;
                uint32_t        last_rx_sof_timestamp;
                uint32_t        rx_sof_timestamp;

                uint32_t        startat;

                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_timer(void);

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

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();
    
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_PIN0);
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_RADIO_PIN);
    
    //NRF_P1->OUTSET = 1 << DEBUG_PIN0;

    timer_init();
    timer_start();
    
    timer_set_callback(0, cb_timer);
    timer_capture_now(0);
    app_vars.startat = timer_getCapturedValue(0) + SLOT_DURATION;
    timer_schedule(0, app_vars.startat);

    while(1) {
        board_sleep();
    }
}

//=========================== private =========================================


//=========================== callbacks =======================================

void cb_timer(void) {

    app_vars.startat = app_vars.startat + SLOT_DURATION;
    timer_schedule(0, app_vars.startat);

    app_dbg.num_timer++;

    //timer_capture_now(0);

    //NRF_P1->OUTCLR = 1 << DEBUG_PIN0;

}