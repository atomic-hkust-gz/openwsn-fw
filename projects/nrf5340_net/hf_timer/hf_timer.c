/**
\brief This program is for multi-node aoa project.
Tx1 node will periodicly send a packet with CTE to help measure the time drift between two receiver.

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

#define SEND_DURATION     (16000000/5000)//(16000000/200)*100    //5ms@ (16000000/200)

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
                app_state_t     state;

                uint8_t         pkt_sqn;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;

                uint8_t         tx_now;

                uint32_t        interval;
                uint32_t        start_time;
                uint32_t        end_time;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================
void      cb_timer(void);


void      nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
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

    //initial debugs GPIO
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_PIN0);
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_RADIO_PIN);



    // start timer
    timer0_init();
    timer_start(NRF_TIMER0_NS);
    timer0_set_callback(0, cb_timer);
    
    timer_capture_now(NRF_TIMER0_NS, 0);
    app_vars.time_slotStartAt = timer_getCapturedValue(NRF_TIMER0_NS, 0) + SEND_DURATION;
    timer_schedule(NRF_TIMER0_NS, 0, app_vars.time_slotStartAt);

    // sleep
    while (1){
        board_sleep();
    }
}


//=========================== private =========================================



//=========================== callbacks =======================================

void cb_timer(void) {

      leds_error_toggle();
      app_dbg.num_timer++;

      app_vars.time_slotStartAt = app_vars.time_slotStartAt + SEND_DURATION;
      timer_schedule(NRF_TIMER0_NS, 0, app_vars.time_slotStartAt);

      //NRF_P1_NS->OUTSET = 1 << DEBUG_PIN0;
      //NRF_P1_NS->OUTCLR = 1 << DEBUG_PIN0;

}

