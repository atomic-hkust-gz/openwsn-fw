#ifndef __GPIO_INTERRUPT_H
#define __GPIO_INTERRUPT_H

#include "stdint.h"
#include "nrf52840_bitfields.h"

//=========================== typedef =========================================

typedef void  (*gpioIrq_cbt)(void);

typedef enum {
    GPIOTE_LOTOHI = GPIOTE_CONFIG_POLARITY_LoToHi,
    GPIOTE_HITOLO = GPIOTE_CONFIG_POLARITY_HiToLo,
    GPIOTE_TOGGLE = GPIOTE_CONFIG_POLARITY_Toggle,
} gpio_irq_polarity_t;

typedef enum {
    PORT0 = 0,
    PORT1 = 1,
} gpio_port_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void gpio_irq_init(void);
int gpio_irq_config(uint8_t channel_number, 
                      gpio_port_t port, 
                      uint8_t pin, 
                      gpio_irq_polarity_t polarity, 
                      gpioIrq_cbt cb);
void gpio_irq_enable(uint8_t channel_number);
void gpio_irq_disable(uint8_t channel_number);


// interrupt handler
void GPIOTE_IRQHandler(void);

#endif
