/**

Manjiang Cao   <manjiang19@hkust-gz.edu.cn> Atomic. 
Tengfei Chang  <tengfeichang@hkust-gz.edu.cn> Atomic. 

*/

#ifndef __TIMER_H__
#define __TIMER_H__

#include "board_info.h"
#include "nrf5340_network.h"

//=========================== define ==========================================

//=========================== typedef =========================================

typedef void  (*timer_cbt)(void);

//=========================== module variables ================================

//=========================== prototypes ======================================

void timer0_init(void);
void timer1_init(void);
void timer0_set_callback(uint8_t compare_id, timer_cbt cb);
void timer1_set_callback(uint8_t compare_id, timer_cbt cb);

void timer_start(NRF_TIMER_Type* NRF_TIMER_NS);
void timer_stop(NRF_TIMER_Type* NRF_TIMER_NS);

void timer_clear(NRF_TIMER_Type* NRF_TIMER_NS);
void timer_capture_now(NRF_TIMER_Type* NRF_TIMER_NS, uint8_t capture_id) ;

void timer_schedule(NRF_TIMER_Type* NRF_TIMER_NS, uint8_t compare_id, uint32_t value);
uint32_t timer_getCapturedValue(NRF_TIMER_Type* NRF_TIMER_NS, uint8_t compare_id);

#endif // __ADC_SENSOR_H__