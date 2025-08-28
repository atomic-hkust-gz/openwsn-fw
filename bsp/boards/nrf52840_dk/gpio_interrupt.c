/**
 * Author: Jacob Louie (jlouie475@connect.hkust-gz.edu.cn)
 * Date:   Aug 2025
 * Description: nRF52840-specific definition of the gpio irq module.
 */

#include "nrf52840.h"
#include "board_info.h"
#include "nrf52840_bitfields.h"
#include "gpio_interrupt.h"


//=========================== defines =========================================
#define IRQ_PRIORITY  0     // adjust based on system design 
                            // 0 = highest priority
                            // 7 = lowest priority

#define channel       0

//=========================== variables =======================================

typedef struct {
    gpioIrq_cbt         cb;
    uint32_t            pin_number;
} gpio_irq_vars_t;

gpio_irq_vars_t gpio_irq_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================


/* 
  pin_number  NRF_GPIO_PIN_MAP(port,pin)
  polarity    GPIOTE_POLARITY_LTHT, GPIOTE_POLARITY_HTOL, 
              or GPIOTE_POLARITY_TOGGLE
  pull        GPIOTE_PULL_NONE, GPIOTE_PULL_DOWN, or GPIOTE_PULL_UP
*/
void gpio_irq_init( 
      uint32_t pin_number, 
      gpiote_polarity_t polarity, 
      gpiote_pull_t pull) {

   NRF_GPIO_Type* NRF_Px_port = (pin_number < 32) ? NRF_P0 : NRF_P1;
   uint8_t nrf_pin = pin_number & 0x1f;

   memset(&gpio_irq_vars, 0, sizeof(gpio_irq_vars_t));
   
   gpio_irq_vars.pin_number     = pin_number;

   // Configure pin as input with pull and sense
   NRF_Px_port->PIN_CNF[nrf_pin]  =
            ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
        | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
        | ((uint32_t)pull << GPIO_PIN_CNF_PULL_Pos)
        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
   
   // Configure GPIOTE channel 0 to generate an event on toggle
   NRF_GPIOTE->CONFIG[channel] =
         ((uint32_t)GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
       | ((uint32_t)pin_number << GPIOTE_CONFIG_PSEL_Pos)
       | ((uint32_t)polarity << GPIOTE_CONFIG_POLARITY_Pos);

   // Clear and enable interrupt
   NRF_GPIOTE->EVENTS_IN[channel] = 0;
   NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos);

   NVIC->IP[GPIOTE_IRQn] = (uint8_t)((IRQ_PRIORITY << 
                            (8 - __NVIC_PRIO_BITS)) & 0xFF);
   NVIC->ISER[GPIOTE_IRQn >> 5] = (uint32_t)(1 << (GPIOTE_IRQn & 0x1F));
}


void gpio_irq_set_callback(gpioIrq_cbt cb) {
    gpio_irq_vars.cb= cb;
}

void gpio_irq_enable(void) {
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
}

void gpio_irq_disable(void) {
    NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_IN0_Msk;
}

//=========================== interrupt handler ===============================

void GPIOTE_IRQHandler(void) {
    // check if the event was triggered by "channel"
    if (NRF_GPIOTE->EVENTS_IN[channel] != 0) {
        NRF_GPIOTE->EVENTS_IN[channel] = 0; // clear the event flag

        if (gpio_irq_vars.cb != NULL) {
            gpio_irq_vars.cb();
        }
    }
}