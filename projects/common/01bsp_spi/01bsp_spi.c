/**
\brief This program shows the use of the "spi" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

This program was written to communicate with the AT86RF231 radio chip. It will
run regardless of your radio, but might not return anything useful.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include "stdint.h"
#include "board.h"
#include "nrf52840.h"
#include "spi.h"

//=========================== defines =========================================
#define LLCC68_RESET_PIN NRF_GPIO_PIN_MAP(0, 8)

#define LLCC68_CMD_GET_STATUS  0xC0

//=========================== variables =======================================

//=========================== prototypes ======================================

// Delay function (basic for-loop delay)
void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < (16000 * ms); i++) {
        __NOP();
    }
}

void llcc68_reset(void) {
    // Configure RESET pin as output
    nrf_gpio_cfg_output(LLCC68_RESET_PIN);

    // Drive low to reset
    NRF_P0->OUTCLR = (1UL << 10);
    delay_ms(10);

    // Drive high to end reset
    NRF_P0->OUTSET = (1UL << 10);
    delay_ms(10);
}

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

 // memset(&app_vars,0,sizeof(app_vars));
   
  // initialize 
  board_init();
  spi_init();
  llcc68_reset();

  // Send GET_STATUS command
  tx_buf[0] = LLCC68_CMD_GET_STATUS;
  tx_buf[1] = 0x00; // Dummy byte for reading response

  rx_buf[0] = 0x00;
  rx_buf[1] = 0x00;
  spi_transfer(tx_buf, rx_buf, 2);
  while (1) {
    __WFE();
    } 
}
