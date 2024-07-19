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

    leds_all_on();

    while (1)
    {
        mainloop();
    }
}

//=========================== callbacks =======================================

int tickk;

bool inited = false;

bool a = true;
bool aa = true;
bool aaa = true;
bool aaaa = true;

bool b = true;
bool c = true;
bool d = true;


void mainloop()
{

    tickk = systickGetTick();

    if (tickk == 3000 && !inited)
    {
        //sendNullCTRPPackage(); //延迟1s启动系统，否则可能会影响1Wire读取
        inited=true;
    }

    //if (tickk >= 15000 && (tickk % 500) == 0)
    //{
    //    test_GetTOC();
    //}

    if (tickk == 14000 && a)
    {
        test_param_write_enHL(1);
        a = false;
    }

    if (tickk == 14500 && aa)
    {
        test_param_write_stabilizer_controller(1);
        aa = false;
    }

    if (tickk == 15000 && aaa)
    {
        test_param_write_kalman_estimator(1);
        aaa = false;
    }

    if (tickk == 15500 && aaaa)
    {
        test_param_write_kalman_estimator(0);
        aaaa = false;
    }
    

    if (tickk == 16000 && b)
    {
        test_send_notify_setpoint_stop();
        b = false;
    }

    if (tickk == 16500 && c)
    {
        test_HL_SendTakeOff();
        //test_read_param(10);
        c = false;
    }

    if (tickk == 17000 && d)
    {
        //test_read_param(10);
        d = false;
    }


    if (tickk >= 21000 && tickk < 23000)
    {
        test_SendEmergencyStop();
        leds_all_off();
    }

    syslinkHandle();
}