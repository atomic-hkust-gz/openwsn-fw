/**
\brief A timer module with only a single compare value.

\author: Tengfei Chang <tengfei.chang@inria.fr> August 2020
*/

#include "nRF5340_network.h"
#include "debugpins.h"
#include "sctimer.h"
#include "board.h"
#include "leds.h"

// ========================== define ==========================================

#define TIMERLOOP_THRESHOLD             0xffff
#define MINIMUM_COMPAREVALE_ADVANCE     5  // nRF52840_PS_v1.0: 2 (page 337)

// LFCLK
#define LFCLK_SRC_RC          0x1 // RC oscillator 
#define LFCLK_SRC_XTAL        0x2 // crystal oscillator
#define LFCLK_SRC_SYNTH       0x3 // synthesized from HFCLK
#define LFCLK_STATE_RUNNING_MASK   ((uint32_t)(0x01))<<16

// RTC
#define RTC_TICK_SHIFT        0
#define RTC_OVRFLW_SHIFT      1
#define RTC_COMPARE0_SHIFT    16
#define RTC_COMPARE1_SHIFT    17
#define RTC_COMPARE2_SHIFT    18
#define RTC_COMPARE3_SHIFT    19

#define RTC_NUM_COMPARE       4

// ========================== variable ========================================

typedef struct {
    sctimer_cbt         sctimer_cb[RTC_NUM_COMPARE];
} sctimer_vars_t;

sctimer_vars_t sctimer_vars;

// ========================== private ==========================================

// ========================== protocol =========================================

void sctimer_init(void){

    memset(&sctimer_vars, 0, sizeof(sctimer_vars_t));

    // stop LFCLK
    NRF_CLOCK_NS->TASKS_LFCLKSTOP     = 1;
    while (
      (NRF_CLOCK_NS->LFCLKSTAT & LFCLK_STATE_RUNNING_MASK) == 1
    );


    // configure the source
    //NRF_CLOCK_NS->LFCLKSRC = (uint32_t)LFCLK_SRC_XTAL;
    NRF_CLOCK_NS->LFCLKSRC = (uint32_t)LFCLK_SRC_RC;

    // start LFCLK
    NRF_CLOCK_NS->TASKS_LFCLKSTART = (uint32_t)1;
    while(
      (NRF_CLOCK_NS->LFCLKSTAT & LFCLK_STATE_RUNNING_MASK) == 0
    );
    
    // stop RTC timer
    NRF_RTC0_NS->TASKS_STOP = (uint32_t)1; 

    // configure compare timer interrupt
    NRF_RTC0_NS->EVTEN = 
          ((uint32_t)0)<<RTC_TICK_SHIFT
        | ((uint32_t)0)<<RTC_OVRFLW_SHIFT
        | ((uint32_t)1)<<RTC_COMPARE0_SHIFT
        | ((uint32_t)1)<<RTC_COMPARE1_SHIFT
        | ((uint32_t)1)<<RTC_COMPARE2_SHIFT
        | ((uint32_t)1)<<RTC_COMPARE3_SHIFT;

    // set priority and enable interrupt in NVIC
    NVIC->IPR[((uint32_t)RTC0_IRQn)] = 
        (uint8_t)(
            (
                RTC_PRIORITY << (8 - __NVIC_PRIO_BITS)
            ) & (uint32_t)0xff
        );
    NVIC->ISER[((uint32_t)RTC0_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RTC0_IRQn) & 0x1f);

    // set prescale
    NRF_RTC0_NS->PRESCALER = (uint32_t)0;

    // start RTC timer
    NRF_RTC0_NS->TASKS_CLEAR = (uint32_t)1;
    NRF_RTC0_NS->TASKS_START = (uint32_t)1;

}

void sctimer_set_callback(uint8_t compare_id, sctimer_cbt cb){
    sctimer_vars.sctimer_cb[compare_id] = cb;
}

void sctimer_setCompare(uint8_t compare_id, PORT_TIMER_WIDTH val){
    
    // enable interrupt
    sctimer_enable(compare_id);


    if ( NRF_RTC0_NS->COUNTER- val < TIMERLOOP_THRESHOLD) {
        // the timer is already late, schedule the ISR right now manually
        NRF_RTC0_NS->EVENTS_COMPARE[compare_id] = (uint32_t)1;
    } else {
        if (val-NRF_RTC0_NS->COUNTER<MINIMUM_COMPAREVALE_ADVANCE){
            // there is hardware limitation to schedule the timer within TIMERTHRESHOLD ticks
            // schedule ISR right now manually
            NRF_RTC0_NS->EVENTS_COMPARE[compare_id] = (uint32_t)1;
        } else {
            // schedule the timer at val
            NRF_RTC0_NS->CC[compare_id] = (uint32_t)val;
        }
    }
}

PORT_TIMER_WIDTH sctimer_readCounter(void){
    
    return NRF_RTC0_NS->COUNTER;
}

void sctimer_enable(uint8_t compare_id){

    // enable interrupt
     NRF_RTC0_NS->INTENSET |=  ((uint32_t)1)<<(compare_id+RTC_COMPARE0_SHIFT);
}

void sctimer_disable(uint8_t compare_id){

    // disable interrupt
     NRF_RTC0_NS->INTENCLR |=  ((uint32_t)1)<<(compare_id+RTC_COMPARE0_SHIFT);
}

// ========================== private =========================================


void RTC0_IRQHandler(void) {
    uint8_t i;

    for(i=0;i<RTC_NUM_COMPARE;i++) {
        
        if (NRF_RTC0_NS->EVENTS_COMPARE[i]) {
            debugpins_isr_set();
            NRF_RTC0_NS->EVENTS_COMPARE[i] = (uint32_t)0;
            if (sctimer_vars.sctimer_cb[i]!=NULL) {
                sctimer_vars.sctimer_cb[i]();
            }
            debugpins_isr_clr();
        }
    }
}

//=========================== interrupt handlers ==============================
