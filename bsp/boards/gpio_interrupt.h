#ifndef __GPIO_INTERRUPT_H
#define __GPIO_INTERRUPT_H

#include "stdint.h"
#include "nrf52840_bitfields.h"

//=========================== typedef =========================================
typedef void  (*gpioIrq_cbt)(void);


typedef enum {
    GPIOTE_POLARITY_LTHT    = GPIOTE_CONFIG_POLARITY_LoToHi, // low-to-high (rising edge)
    GPIOTE_POLARITY_HTOL    = GPIOTE_CONFIG_POLARITY_HiToLo, // high-to-low (falling edge)
    GPIOTE_POLARITY_TOGGLE  = GPIOTE_CONFIG_POLARITY_Toggle, // any change in state
} gpiote_polarity_t;

typedef enum {
    GPIOTE_PULL_NONE        = GPIO_PIN_CNF_PULL_Disabled, 
    GPIOTE_PULL_DOWN        = GPIO_PIN_CNF_PULL_Pulldown,
    GPIOTE_PULL_UP          = GPIO_PIN_CNF_PULL_Pullup,
} gpiote_pull_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void gpio_irq_init(uint32_t pin_number, gpiote_polarity_t polarity, gpiote_pull_t pull);
void gpio_irq_set_callback(gpioIrq_cbt cb);
void gpio_irq_enable(void);
void gpio_irq_disable(void);


// interrupt handler
void GPIOTE_IRQHandler(void);



#endif
