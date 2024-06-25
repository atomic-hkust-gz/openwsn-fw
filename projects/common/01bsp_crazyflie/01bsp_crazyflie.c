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

// crazyflie
#include "cf_pm.h"
#include "cf_systick.h"

//=========================== defines =========================================

#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
   
   // initialize the board
   board_init();

   // Crazyflie init
   pmInit();
   systickInit();

   // Crazyflie boot
   pmSetState(pmSysRunning);
   leds_all_on();

   
   while(1) {
      pmProcess();
   }
}

//=========================== callbacks =======================================
