/**
 * Author: Tamas Harczos (tamas.harczos@imms.de)
 * Date:   Apr 2018
 * Description: nRF52840-specific definition of the "board" bsp module.
 */

#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "board.h"
#include "leds.h"
#include "sctimer.h"
#include "debugpins.h"
#include "uart.h"
#include "radio.h"
#include "spi.h"
#include "radio.h"
#include "sensors.h"
#include "i2c.h"
#include "spi.h"
#include "gpio_irq.h"


//=========================== variables =======================================

//=========================== prototypes ======================================

void enable_dcdc(void);

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
    debugpins_init();
    uart_init();
    sctimer_init();
    radio_init();

    i2c_init();
    spi_init();

    gpio_irq_init(); 

    // configure dcdc
    enable_dcdc();
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

void enable_dcdc(void) {

    uint32_t status; 

    status = NRF_POWER->MAINREGSTATUS;

    if (status == 0) {

        while (NRF_POWER->DCDCEN == 0){
            // in normal voltage mode: PS1.2, page 59
            NRF_POWER->DCDCEN = (uint32_t)1;
        }
    }
}

//=========================== prototype =======================================

void nrf_gpio_cfg_input(uint32_t pin_number) {

    NRF_GPIO_Type* NRF_Px_port;
    uint32_t       nrf_pin_number;

    if (pin_number < 32) {

        NRF_Px_port     = NRF_P0;
        nrf_pin_number  = pin_number;
    } else {

        NRF_Px_port = NRF_P1;
        nrf_pin_number  = pin_number & 0x1f;
    }
    
    NRF_Px_port->PIN_CNF[nrf_pin_number]  = \
            ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
        | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
        | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}


void nrf_gpio_cfg_output(uint32_t pin_number) {
    NRF_GPIO_Type* port = (pin_number < 32) ? NRF_P0 : NRF_P1;
    uint32_t pin = pin_number & 0x1F;

    port->PIN_CNF[pin] =
          (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos)
        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

//=========================== interrupt handlers ==============================
