/**
 *
 * @file cf_param.h
 * @brief Crazyflie Parameter Operation
 *    this API is migrated from the higher level code like the python client
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#ifndef __CF_PARAM_H__
#define __CF_PARAM_H__

#include <stdint.h>

//=========================== define ==========================================

#define index_deck_bcFlow2 10
#define index_commander_enHighLevel 89
#define index_kalman_resetEstimation 112
#define index_stabilizer_controller 140

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void param_set(uint16_t id, const void *value, uint8_t len);
void param_read(uint16_t id); // for debug, can not automatically read the param, need cache the port=2, channel=1 CTRP data

#endif // __CF_PARAM_H__
