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
#define IRQ_PRIORITY          7     // adjust based on system design 
                                    // 0 = highest priority
                                    // 7 = lowest priority
#define MAX_GPIOTE_CHANNELS   8
#define MAX_PIN_NUMBER        32

//=========================== variables =======================================

typedef struct {
    gpioIrq_cbt         cb;
    uint32_t            pinNumber;
    uint32_t            portNumber;
    uint8_t             configured;
} gpio_irq_vars_t;

gpio_irq_vars_t gpio_irq_vars[MAX_GPIOTE_CHANNELS];

//=========================== prototypes ======================================

//=========================== public ==========================================


void gpio_irq_init(void) {

    uint8_t channel;
    memset(gpio_irq_vars, 0, sizeof(gpio_irq_vars));
    
    NVIC_SetPriority(GPIOTE_IRQn, IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}


int gpio_irq_config(uint8_t channel, 
                      gpio_port_t port, 
                      uint8_t pin, 
                      gpio_irq_polarity_t polarity, 
                      gpioIrq_cbt cb) {

    uint32_t gpioPin;

    if (channel >= MAX_GPIOTE_CHANNELS)   return -1;
    if (pin >= MAX_PIN_NUMBER)            return -2;

    // set callback
    gpio_irq_vars[channel].cb= cb;
    // save settings
    gpio_irq_vars[channel].pinNumber  = pin;
    gpio_irq_vars[channel].portNumber = port;
    gpio_irq_vars[channel].configured = 1;

    // configure port/pin assignment
    gpioPin = NRF_GPIO_PIN_MAP(port, pin);
    NRF_GPIO_Type* NRF_Px_port = (gpioPin < 32) ? NRF_P0 : NRF_P1;


    // Configure pin as input
    NRF_Px_port->PIN_CNF[pin] =
          ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
        | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
        | ((uint32_t)GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)
        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    // Configure GPIOTE channel
    NRF_GPIOTE->CONFIG[channel] =
          ((uint32_t)GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
        | ((uint32_t)gpioPin << GPIOTE_CONFIG_PSEL_Pos);

    // set trigger polarity
    // hi to low, low to high, or any    
    switch (polarity) {
        case GPIOTE_LOTOHI:
            NRF_GPIOTE->CONFIG[channel] |= (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
            break;
        case GPIOTE_HITOLO:
            NRF_GPIOTE->CONFIG[channel] |= (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);
            break;
        case GPIOTE_TOGGLE:
            NRF_GPIOTE->CONFIG[channel] |= (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);
            break;
    }

    // Clear old event
    NRF_GPIOTE->EVENTS_IN[channel] = 0;


    return 0;
}


void gpio_irq_enable(uint8_t channel) {
    /*
    enable channel 1 example 
    bit value        |0|0|0|0|0|0|1|0|
    channel number   |7|6|5|4|3|2|1|0|
    */
    if (channel >= MAX_GPIOTE_CHANNELS) {
      // invalid channel number
    }
    else {
      NRF_GPIOTE->INTENSET = 0x1UL << channel;
      // channel 1 equivalent to GPIOTE_INTENSET_IN1_Pos
    }
}

void gpio_irq_disable(uint8_t channel) {
    /*
    disable channel 0 example 
    bit value        |0|0|0|0|0|0|0|1|
    channel number   |7|6|5|4|3|2|1|0|
    */
    if (channel >= MAX_GPIOTE_CHANNELS) {
      // invalid channel number
    }
    else {
      NRF_GPIOTE->INTENCLR = 0x1UL << channel;
      // channel 0 equivalent to GPIOTE_INTENSET_IN0_Pos
    }
}

//=========================== interrupt handler ===============================

void GPIOTE_IRQHandler(void) {
    
    uint8_t channel;

    for (channel = 0; channel < MAX_GPIOTE_CHANNELS; channel++){
      // check if the event was triggered by "channel" & IRQ is configured
      if (gpio_irq_vars[channel].configured && 
          NRF_GPIOTE->EVENTS_IN[channel]) {

        NRF_GPIOTE->EVENTS_IN[channel] = 0; // clear the event flag

        if (gpio_irq_vars[channel].cb != NULL) {
          gpio_irq_vars[channel].cb();
        }
      }
    }
}