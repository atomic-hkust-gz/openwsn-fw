/**
\brief This is a program which shows how to use GPIO interrupt.


Program will start with all LEDs off. When interrupt is triggered
the LEDs will toggle.

\author Jacob Louie <jlouie475@connect.hkust-gz.edu.cn>, August 2025.
*/

#include "stdint.h"
#include "stdio.h"
#include "board.h"
#include "leds.h"
#include "gpio_irq.h"

#define IRQCHANNEL                0
#define IRQPORT               PORT1
#define GPIO_IRQ_PIN              6
#define RISING_EDGE   GPIOTE_LOTOHI

//=========================== variables =======================================

typedef struct {
    uint8_t     toggle_num;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_toggle(void);

//=========================== main ============================================

void mote_main(void) {

    board_init();

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    gpio_irq_config(IRQCHANNEL, 
                    IRQPORT, 
                    GPIO_IRQ_PIN, 
                    RISING_EDGE, 
                    cb_toggle);
    gpio_irq_enable(IRQCHANNEL);


    while (1) {
        board_sleep();
    }
}

void cb_toggle(void) {  

    app_vars.toggle_num++;
    leds_all_toggle();
  }