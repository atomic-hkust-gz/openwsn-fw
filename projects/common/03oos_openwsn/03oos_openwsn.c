/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#include "config.h"
#include "board.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"

#include "cf_crazyflie.h"

int mote_main(void) {
   
   // initialize
   board_init();
   scheduler_init();
   openstack_init();

   // crazyflie
   crazyflieInit();
   
   // start
   scheduler_start();
   return 0; // this line should never be reached
}

