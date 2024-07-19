/**
\brief This is a program which provide Crazyflie Boot & Communication bsp

\author Lan HUANG <yelloooblue@outlook.com>, June 2024
*/

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

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
int mote_main(void)
{
    // initialize the board
    board_init();

    // Crazyflie init
    crazyflieInit();

    while (1)
    {
        mainloop();
    }
}

//=========================== callbacks =======================================

int tickk;

bool enHighLevel = false;

bool a = true;
bool b = true;
bool d = true;

void mainloop()
{
    tickk = systickGetTick(); //unsigned int

    if (tickk == 5000 && !enHighLevel)
    {
        high_level_enable();
        enHighLevel = true;

        leds_all_on();
    }

    if (tickk == 10000 && a)
    {
        high_level_takeoff(0.5, 1.0, 0.0);
        a = false;
    }

    if (tickk == 12000 && b)
    {
        high_level_goto(0.5, 0.0, 0.0, 0, 1.0, true);
        b = false;
    }

    if (tickk == 13000 && d)
    {
        high_level_land(0.0, 1.2, 0);
        d = false;
    }

    if (tickk >= 15000 && tickk < 23000)
    {
        EmergencyStop();
        leds_all_off();
    }

    syslinkHandle();
}