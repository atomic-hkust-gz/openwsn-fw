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
    uint32_t            pin_mask;
    uint8_t             port;      // 0 for P0, 1 for P1
} gpio_irq_vars_t;

gpio_irq_vars_t gpio_irq_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================


/* 
  port      GPIO port (0 or 1) P0/P1
  pin       Pin number (0–31)
  sense     NRF_GPIO_PIN_SENSE_HIGH or NRF_GPIO_PIN_SENSE_LOW
  pull      NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_PULLUP, or NRF_GPIO_PIN_PULLDOWN
*/
void gpio_irq_init(uint8_t port, uint8_t pin, uint32_t sense, uint32_t pull) {
   memset(&gpio_irq_vars, 0, sizeof(gpio_irq_vars_t));

   gpio_irq_vars.port     = port;
   gpio_irq_vars.pin_mask = (1UL << pin);

   // Configure pin as input with pull and sense

   if (port == 0) {
       NRF_P0->PIN_CNF[pin] = 
       (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
           ((uint32_t)pull << GPIO_PIN_CNF_PULL_Pos) |
           ((uint32_t)sense << GPIO_PIN_CNF_SENSE_Pos);
   }
   else {
       NRF_P1->PIN_CNF[pin] = 
           (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
           ((uint32_t)pull << GPIO_PIN_CNF_PULL_Pos) |
           ((uint32_t)sense << GPIO_PIN_CNF_SENSE_Pos);
   }
   
   // Configure GPIOTE channel 0 to generate an event on toggle
   NRF_GPIOTE->CONFIG[channel] = 
       (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
       (pin << GPIOTE_CONFIG_PSEL_Pos) |
       (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);

   // Clear and enable interrupt
   NRF_GPIOTE->EVENTS_IN[channel] = 0;
   NRF_GPIOTE->INTENSET     = GPIOTE_INTENSET_IN0_Msk;

   NVIC->IP[GPIOTE_IRQn]    = (uint8_t)((IRQ_PRIORITY << (8 - __NVIC_PRIO_BITS)) & 0xFF);
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


//=========================== private =========================================




//=========================== interrupt handler ===============================

void GPIOTE_IRQHandler(void) {

    if (NRF_GPIOTE->EVENTS_IN[channel] != 0) {
        NRF_GPIOTE->EVENTS_IN[channel] = 0;

        if (gpio_irq_vars.cb != NULL) {
            gpio_irq_vars.cb();
        }
    }
}