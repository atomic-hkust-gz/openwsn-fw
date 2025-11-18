#ifndef MOTOR_H
#define MOTOR_H


#include "pwm.h"



uint8_t motor_speed_set(uint8_t motor_num, int16_t speed);

void car_control(float V, float theta, float omega);

#endif
