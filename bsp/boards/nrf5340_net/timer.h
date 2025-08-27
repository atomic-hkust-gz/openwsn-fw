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

void timer0_start(void);
void time0r_stop(void);

void timer0_start(void);
void timer1_stop(void);

void timer0_clear(void);
void timer1_clear(void);

void timer0_capture_now(uint8_t capture_id);
void timer1_capture_now(uint8_t capture_id);

void timer0_schedule(uint8_t compare_id, uint32_t value);
void timer1_schedule(uint8_t compare_id, uint32_t value);

uint32_t timer0_getCapturedValue(uint8_t compare_id);
uint32_t timer1_getCapturedValue(uint8_t compare_id);

#endif // __ADC_SENSOR_H__