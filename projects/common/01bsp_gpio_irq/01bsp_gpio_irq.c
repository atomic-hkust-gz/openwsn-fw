/**
\brief This is a program which shows how to GPIO interrupt.


Program will start in slow LED blink rate. When interrupt is triggered
the LED blink rate will toggle between fast and slow LED blink rate.

\author Jacob Louie <jlouie475@connect.hkust-gz.edu.cn>, August 2025.
*/

#include "stdint.h"
#include "stdio.h"
#include "board.h"
#include "leds.h"
#include "gpio_irq.h"

#define SLOW_BLINK 0
#define FAST_BLINK 1

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

    gpio_irq_set_callback(cb_toggle);
    gpio_irq_enable();

    while (1) {
        board_sleep();
    }
}

void cb_toggle(void) {  

    app_vars.toggle_num++;
    leds_all_toggle();
}