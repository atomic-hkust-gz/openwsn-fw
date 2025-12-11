/**
\brief nRF5340_network-specific definition of the "board" bsp module.

\author: Tengfei Chang <tengfei.chang@inria.fr> August 2020
*/

#include "nRF5340_network.h"
#include "board.h"
// bsp modules
#include "debugpins.h"
#include "leds.h"
#include "uart.h"
#include "sctimer.h"
#include "radio.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

void clocks_start(void);
void clocks_stop(void);

//=========================== main ============================================

extern int mote_main(void);

int main(void) {
   return mote_main();
}

//=========================== public ==========================================

void board_init(void) {

    //clocks_start();

    // initialize bsp modules
    debugpins_init();
    leds_init();
    uart_init();
    radio_init();
    sctimer_init();

    //enable_dcdc();
}

void board_sleep(void) {
    // todo
    __WFE();
    __WFE();
}

void board_reset(void) {
    // todo
    NVIC_SystemReset();
}

//=========================== private =========================================

void clocks_start( void ){

    // Start HFCLK and wait for it to start.
    NRF_CLOCK_NS->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK_NS->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK_NS->EVENTS_HFCLKSTARTED == 0);
}

void clocks_stop( void ){

    // Stop HFCLK and wait for it to stop.
    NRF_CLOCK_NS->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK_NS->TASKS_HFCLKSTOP = 1;
    while (NRF_CLOCK_NS->EVENTS_HFCLKSTARTED == 1);
}

//void enable_dcdc(void) {

//    uint32_t status; 

//    status = NRF_POWER_NS->MAINREGSTATUS;

//    if (status == 0) {

//        while (NRF_POWER_NS->DCDCEN == 0){
//            // in normal voltage mode: PS1.2, page 59
//            NRF_POWER_NS->DCDCEN = (uint32_t)1;
//        }
//    }
//}