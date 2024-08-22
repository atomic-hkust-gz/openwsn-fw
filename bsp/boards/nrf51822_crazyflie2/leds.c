/**
 * Author: Lan HUANG (YelloooBlue@Outlook.com)
 * Date:   Apr 2024
 * Description: nrf51822_crazyflie2-specific definition of the "leds" bsp module.
 */

#include "leds.h"
#include "nrf51.h"
#include "board_info.h"

//=========================== defines =========================================

// nrf51822_crazyflie2
#define LED_1           NRF_GPIO_PIN_MAP(0,13) //nrf51822 on crazyflie has only one BLUE_LED

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void leds_init() {
    NRF_GPIO->DIRSET = 1<<LED_1;
    leds_all_off();
}

//==== error led

void leds_error_off(void) {
    NRF_GPIO->OUTCLR = 1<<LED_1;
}

void leds_error_on(void) {
    NRF_GPIO->OUTSET = 1<<LED_1;
}

void leds_error_toggle(void) {
    if ((NRF_GPIO->OUT & (1<<LED_1))!=0) {        
        NRF_GPIO->OUTCLR = 1<<LED_1;
    } else {
        NRF_GPIO->OUTSET = 1<<LED_1;
    }
}

uint8_t leds_error_isOn(void) {
    if (NRF_GPIO->OUT & (1<<LED_1)) {
        return 0;
    } else {
        return 1;
    }
}

//==== sync led

void leds_sync_off(void){}
void leds_sync_on(void){}
void leds_sync_toggle(void){}
uint8_t leds_sync_isOn(void){}

//==== radio led

void leds_radio_off(void){}
void leds_radio_on(void){}
void leds_radio_toggle(void){}
uint8_t leds_radio_isOn(void){}

//==== debug led

void leds_debug_off(void){}
void leds_debug_on(void){}
void leds_debug_toggle(void){}
uint8_t leds_debug_isOn(void){}

//==== all leds

void leds_all_on(void) {
    leds_radio_on();
    leds_sync_on();
    leds_debug_on();
    leds_error_on();
}

void leds_all_off(void) {
    leds_radio_off();
    leds_sync_off();
    leds_debug_off();
    leds_error_off();
}

void leds_all_toggle(void) {
    leds_radio_toggle();
    leds_sync_toggle();
    leds_debug_toggle();
    leds_error_toggle();
}

void leds_error_blink(void) {
    
    uint8_t i;
    uint32_t j;

    // turn all LEDs off
    leds_all_off();

    // blink error LED for ~10s
    for (i = 0; i < 100; i++) {
        leds_error_toggle();
        for(j=0;j<0x1ffff;j++);
    }
}

void leds_circular_shift(void) {

    uint32_t led_new_value;
    uint32_t led_read;
    uint32_t shift_bit;

    led_read = NRF_GPIO->OUT & 0x0001e000;
    shift_bit = (NRF_GPIO->OUT & 0x00010000)>>3;
    led_new_value = ((led_read<<1) & 0x0001e000) | shift_bit; 
    
    NRF_GPIO->OUTSET = led_new_value;
}

void leds_increment(void) {
    
    uint32_t led_new_value;
    uint32_t led_read;

    led_read = (NRF_GPIO->OUT & 0x0001e000)>>13;
    led_new_value = ((led_read+1) & 0x0000000f)<<13;

    NRF_GPIO->OUTSET = led_new_value;
}

//=========================== private =========================================