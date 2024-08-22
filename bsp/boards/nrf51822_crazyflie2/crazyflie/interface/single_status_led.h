/**
 * 
 * @file single_status_led.h
 * @brief Because the nRF51822 on Crazyflie 2.X has only one LED, this interface is designed to control it.
 *          The status LED is used to indicate the status of the Crazyflie 2.X and the OpenWSN stack.
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date Aug 2024
 *
 */
#ifndef __SINGLE_STATUS_LED_H__
#define __SINGLE_STATUS_LED_H__

#include <stdint.h>
#include "leds.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//state machine for the status LED
typedef enum {
    LED_OFF = 0,
    LED_ON = 1,
    LED_BLINK = 2,          //default blink (1s on, 1s off)
    LED_BLINK_FAST = 3,     //（0.5s on, 0.5s off)
    LED_BLINK_SLOW = 4,     // (2s on, 2s off)
    LED_BLINK_LSS = 5,      //long-short-short (0.5s on, 0.25s off, 0.25s on, 0.25s off, 0.25s on, 0.5s off)
} led_state_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void status_led_init();
void status_led_handle();
void status_led_set(led_state_t state);
void status_led_clear();

#endif // __SINGLE_STATUS_LED_H__