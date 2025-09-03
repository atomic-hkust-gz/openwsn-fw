#ifndef __GPIO_INTERRUPT_H
#define __GPIO_INTERRUPT_H

#include "stdint.h"
#include "nrf52840_bitfields.h"

//=========================== typedef =========================================

typedef void  (*gpioIrq_cbt)(void);

//=========================== variables =======================================

//=========================== prototypes ======================================

void gpio_irq_init(void);
void gpio_irq_set_callback(gpioIrq_cbt cb);
void gpio_irq_enable(void);
void gpio_irq_disable(void);


// interrupt handler
void GPIOTE_IRQHandler(void);



#endif
