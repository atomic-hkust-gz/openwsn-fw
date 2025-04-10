/**
 * Author: Lan HUANG (YelloooBlue@Outlook.com)
 * Date:   Apr 2024
 * Description: nRF51-dk-specific definition of the "board" bsp module.
 */

#include "nrf51.h"
#include "board.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

extern int mote_main(void);

int main(void) {
    return mote_main();
}


//=========================== public ==========================================

void board_init(void) {

    // start hfclock
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    leds_init();

    uart_init();
    sctimer_init();
    radio_init(); //Temp BLE

}

/**
 * Puts the board to sleep
 */
void board_sleep(void) {

    __WFE();
    __WFE();
}

/**
 * Resets the board
 */
void board_reset(void) {

    NVIC_SystemReset();
}

//=========================== private =========================================


//=========================== interrupt handlers ==============================
