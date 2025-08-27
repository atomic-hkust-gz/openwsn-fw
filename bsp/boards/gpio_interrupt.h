#ifndef __GPIO_INTERRUPT_H
#define __GPIO_INTERRUPT_H

#include "stdint.h"

//=========================== typedef =========================================
typedef void  (*gpioIrq_cbt)(void);

//=========================== variables =======================================

//=========================== prototypes ======================================


void gpio_irq_init(uint8_t port, uint8_t pin, uint32_t sense, uint32_t pull);
void gpio_irq_set_callback(gpioIrq_cbt cb);
void gpio_irq_enable(void);
void gpio_irq_disable(void);


// interrupt handler
void GPIOTE_IRQHandler(void);



#endif
