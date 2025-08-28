/**
 * Author: Jacob Louie (jlouie475@connect.hkust-gz.edu.cn)
 * Date:   Aug 2025
 * Description: nRF52840-specific definition of the gpio irq module.
 */

#include "nrf52840.h"
#include "board_info.h"
#include "nrf52840_bitfields.h"
#include "gpio_irq.h"


//=========================== defines =========================================
     // adjust based on system design 
                            // 0 = highest priority
                            // 7 = lowest priority

#define GPIO_IRQ_PORT 1
#define GPIO_IRQ_PIN  6
#define GPIO_INT_PIN  NRF_GPIO_PIN_MAP(GPIO_IRQ_PORT,GPIO_IRQ_PIN)

#define GPIO_IQR_CHANNEL   0

//=========================== variables =======================================

typedef struct {
    gpioIrq_cbt         cb;
} gpio_irq_vars_t;

gpio_irq_vars_t gpio_irq_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================


void gpio_irq_init(void) {

   NRF_GPIO_Type* NRF_Px_port = (GPIO_INT_PIN < 32) ? NRF_P0 : NRF_P1;

   memset(&gpio_irq_vars, 0, sizeof(gpio_irq_vars_t));

   // Configure pin as input with pull and sense
   NRF_Px_port->PIN_CNF[GPIO_IRQ_PIN]  =
            ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
        | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
        | ((uint32_t)GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)
        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
   
   // Configure GPIOTE channel 0 to generate an event on toggle
   NRF_GPIOTE->CONFIG[GPIO_IQR_CHANNEL] =
         ((uint32_t)GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
       | ((uint32_t)GPIO_INT_PIN << GPIOTE_CONFIG_PSEL_Pos)
       | ((uint32_t)GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

   // Clear and enable interrupt
   NRF_GPIOTE->EVENTS_IN[GPIO_IQR_CHANNEL] = 0;
   NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos);

   NVIC->IP[GPIOTE_IRQn] = (uint8_t)((GPIO_PRIORITY << 
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
    if (NRF_GPIOTE->EVENTS_IN[GPIO_IQR_CHANNEL] != 0) {

        NRF_GPIOTE->EVENTS_IN[GPIO_IQR_CHANNEL] = 0; // clear the event flag

        if (gpio_irq_vars.cb != NULL) {
            gpio_irq_vars.cb();
        }
    }
}