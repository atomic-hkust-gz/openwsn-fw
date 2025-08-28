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
   uint8_t     state;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void short_delay(void);
void long_delay(void);
void cb_toggle(void);

//=========================== main ============================================

void mote_main(void) {
   uint8_t i;
   void (*delay_func)(void);

   board_init();

   // clear local variables
   memset(&app_vars,0,sizeof(app_vars_t));
   app_vars.state = SLOW_BLINK;//FAST_BLINK or SLOW_BLINK

   gpio_irq_set_callback(cb_toggle);
   gpio_irq_enable();

   while (1) {

     if (app_vars.state == SLOW_BLINK) {
       delay_func = long_delay;
     } 
     else {
       delay_func = short_delay;
     }

     // LEDs blink 
     leds_all_off();           
     delay_func();
     leds_all_on();       
     delay_func();
   }
}

void short_delay(void) {
   volatile uint16_t delay;
   for (delay=0xffff;delay>0;delay--);
}

void long_delay(void) {
   volatile uint32_t delay;
   for (delay=0x2fffff;delay>0;delay--);
}

void cb_toggle(void) {      
   // toggle flag
   app_vars.state ^= 0x01;
}