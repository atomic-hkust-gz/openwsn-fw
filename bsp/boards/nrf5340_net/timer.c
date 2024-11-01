/**
    \brief Definition of the nrf52480 Timer driver.
    \author Manjiang Cao   <manjiang19@hkust-gz.edu.cn> Atomic. Nov, 2023
    \author Tengfei Chang  <tengfeichang@hkust-gz.edu.cn> Atomic. Nov, 2023
*/

#include "nrf5340_network.h"
#include "nrf5340_network_bitfields.h"
#include "timer.h"

//=========================== defines =========================================

#define MODE_TIMER              0
#define MODE_COUNTER            1
#define MODE_LOWPOWER_COUNTER   2

#define BITMODE_16BIT           0
#define BITMODE_0BIT            1
#define BITMODE_24BIT           2
#define BITMODE_32BIT           3

#define OFFSET_INTENSET_CC_0    16  
#define OFFSET_INTENSET_CC_1    17  
#define OFFSET_INTENSET_CC_2    18  
#define OFFSET_INTENSET_CC_3    19  
#define OFFSET_INTENSET_CC_4    20  
#define OFFSET_INTENSET_CC_5    21     

#define NUM_TIMER_COMPARE      8

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
 
    timer_cbt         cb[NUM_TIMER_COMPARE];
}timer_vars_t;

timer_vars_t timer0_vars;
timer_vars_t timer1_vars;

//=========================== prototype =======================================

//=========================== public ==========================================

void timer0_init(void) {

    NRF_TIMER0_NS->MODE    = MODE_TIMER;
    NRF_TIMER0_NS->BITMODE = BITMODE_32BIT;

    // set prescaler
    NRF_TIMER0_NS->PRESCALER = 0x00;

    // configure interrupt
    NVIC->IPR[((uint32_t)TIMER0_IRQn)] = (uint8_t)((RTC_PRIORITY << (8 - __NVIC_PRIO_BITS)) & (uint32_t)0xff);   //problem: __NVIC_PRIO_BITS will this effect the timer?
    NVIC->ISER[((uint32_t)TIMER0_IRQn)>>5] = ((uint32_t)1) << ( ((uint32_t)TIMER0_IRQn) & 0x1f);
    // set compare interrupt for timer0
    //NRF_TIMER0->INTENSET = (1<<OFFSET_INTENSET_CC_0) |\
    //                       (1<<OFFSET_INTENSET_CC_1) |\
    //                       (1<<OFFSET_INTENSET_CC_2) |\
    //                       (1<<OFFSET_INTENSET_CC_3) |\
    //                       (1<<OFFSET_INTENSET_CC_4) |\
    //                       (1<<OFFSET_INTENSET_CC_5);

                           
    NRF_TIMER0_NS->INTENSET = (1<<OFFSET_INTENSET_CC_0);
}

void timer1_init(void) {      //problem: when using two timer, the interrupt configure is the same except for TIMER1_IRQn?

    NRF_TIMER1_NS->MODE    = MODE_TIMER;
    NRF_TIMER1_NS->BITMODE = BITMODE_32BIT;

    // set prescaler
    NRF_TIMER1_NS->PRESCALER = 0x00;

    // configure interrupt
    //NVIC->IPR[((uint32_t)TIMER1_IRQn)] = (uint8_t)((RTC_PRIORITY << (8 - __NVIC_PRI0_BITS)) & (uint32_t)0xff); //__NVIC_PRI0_BITS undeclared
    NVIC->IPR[((uint32_t)TIMER1_IRQn)] = (uint8_t)((RTC_PRIORITY << (8 - 3)) & (uint32_t)0xff);
    NVIC->ISER[((uint32_t)TIMER1_IRQn)>>5] = ((uint32_t)1) << ( ((uint32_t)TIMER1_IRQn) & 0x1f);
    // set compare interrupt for timer0
    //NRF_TIMER0->INTENSET = (1<<OFFSET_INTENSET_CC_0) |\
    //                       (1<<OFFSET_INTENSET_CC_1) |\
    //                       (1<<OFFSET_INTENSET_CC_2) |\
    //                       (1<<OFFSET_INTENSET_CC_3) |\
    //                       (1<<OFFSET_INTENSET_CC_4) |\
    //                       (1<<OFFSET_INTENSET_CC_5);

                           
    NRF_TIMER1_NS->INTENSET = (1<<OFFSET_INTENSET_CC_0);
}

void timer0_set_callback(uint8_t compare_id, timer_cbt cb) {

    timer0_vars.cb[compare_id] = cb;
}

void timer1_set_callback(uint8_t compare_id, timer_cbt cb) {

    timer1_vars.cb[compare_id] = cb;
}

void timer_schedule(NRF_TIMER_Type* NRF_TIMER_NS, uint8_t compare_id, uint32_t value) {

    NRF_TIMER_NS->CC[compare_id] = value;
}

uint32_t timer_getCapturedValue(NRF_TIMER_Type* NRF_TIMER_NS, uint8_t compare_id) {

    return NRF_TIMER_NS->CC[compare_id];
}


void timer_clear(NRF_TIMER_Type* NRF_TIMER_NS) {
    NRF_TIMER_NS->TASKS_CLEAR = 1;
}


void timer_capture_now(NRF_TIMER_Type* NRF_TIMER_NS, uint8_t capture_id) {
    NRF_TIMER_NS->TASKS_CAPTURE[capture_id] = 1;
}


void timer_start(NRF_TIMER_Type* NRF_TIMER_NS) {

    NRF_TIMER_NS->TASKS_START = 1;
}

void timer_stop(NRF_TIMER_Type* NRF_TIMER_NS) {
    
    NRF_TIMER_NS->TASKS_STOP = 1;
}

//=========================== private =========================================

void TIMER0_IRQHandler(void) {

    uint8_t i;

    for(i=0;i<NUM_TIMER_COMPARE;i++) {
        
        if (NRF_TIMER0_NS->EVENTS_COMPARE[i]) {
        
            NRF_TIMER0_NS->EVENTS_COMPARE[i] = 0;
            if (timer0_vars.cb[i]!=NULL) {
                timer0_vars.cb[i]();
            }
        }
    }
    

}

void TIMER1_IRQHandler(void) {

    uint8_t i;

    for(i=0;i<NUM_TIMER_COMPARE;i++) {
        
        if (NRF_TIMER1_NS->EVENTS_COMPARE[i]) {
        
            NRF_TIMER1_NS->EVENTS_COMPARE[i] = 0;
            if (timer1_vars.cb[i]!=NULL) {
                timer1_vars.cb[i]();
            }
        }
    }
    

}