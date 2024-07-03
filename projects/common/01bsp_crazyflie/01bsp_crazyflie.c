/**
\brief This is a program which provide Crazyflie Boot & Communication bsp

\author Lan HUANG <yelloooblue@outlook.com>, June 2024
*/

#include "stdint.h"
#include "stdio.h"
#include "string.h"

// bsp modules required
#include "board.h"
#include "sctimer.h"
#include "leds.h"

// cf
#include "cf_crazyflie.h"
#include "cf_systick.h"

//=========================== defines =========================================


//=========================== variables =======================================


//=========================== prototypes ======================================

void mainloop();

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
   
   // initialize the board
   board_init();

   // Crazyflie init
   crazyflieInit();

   leds_all_on();

   while(1){
      mainloop();
   }
   

}

//=========================== callbacks =======================================

int tickk;

void mainloop(){

    syslink_loop();

    tickk = systickGetTick();

    if (tickk >= 15000 && tickk < 16000) {
        test_changeThrust(20000);
    }

    if (tickk >= 16000 && tickk < 18000) {
        test_changeThrust(45000);
    }

    if (tickk >= 18000 && tickk < 20000) {
        test_changeThrust(20000);
    }

    if (tickk >= 20000) {
        test_changeThrust(0);
    }

    test_sendSetpointSyslinkPkg();

}