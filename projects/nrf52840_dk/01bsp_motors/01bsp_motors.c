/**
\brief This program shows the use of the "motors" bsp module.

\author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, November 2025.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "debugpins.h"
#include "leds.h"
#include "sctimer.h"
#include "pwm.h"
#include "motor.h"

//=========================== defines =========================================

#define SCTIMER_PERIOD     32768 // @32kHz = 1s

//=========================== variables =======================================

typedef struct {
   uint16_t num_compare;
   float    velocity;
   float    direction;
   float    rotation_speed;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_compare(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {  
   
   // initialize board. 
   board_init();
   ;

   app_vars.velocity = 100;
   app_vars.direction = 90 % 360;
   app_vars.rotation_speed = 0;

   sctimer_set_callback(cb_compare);
   sctimer_setCompare(sctimer_readCounter()+SCTIMER_PERIOD);
      
   pwm_multi_init();
   while(app_vars.num_compare==0); 
    
   pwm_start(PWM_0);  
   pwm_start(PWM_1); 
   
   while (1) {
      board_sleep();
   }
}

//=========================== callbacks =======================================

void cb_compare(void) {
   
   // toggle pin
   debugpins_frame_toggle();
   
   // toggle error led
   leds_error_toggle();
   
   // increment counter
   app_vars.num_compare++;
   
   // schedule again
   sctimer_setCompare(sctimer_readCounter()+SCTIMER_PERIOD);

   app_vars.direction += 90;
   if (app_vars.direction>=360) {
      app_vars.direction -= 360;
   }

   car_control(app_vars.velocity, app_vars.direction, app_vars.rotation_speed);
}
