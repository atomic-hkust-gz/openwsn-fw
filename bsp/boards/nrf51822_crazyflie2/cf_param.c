/**
 *
 * @file cf_param.c
 * @brief Crazyflie Parameter Operation
 *    this API is migrated from the higher level code like the python client
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// crazyflie bsp
#include "cf_param.h"
#include "cf_ctrp.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void param_set(uint16_t id, const void *value, uint8_t len)
{

  uint8_t data[CRTP_MAX_DATA_SIZE];
  data[0] = id & 0xFF;
  data[1] = (id >> 8) & 0xFF;

  memcpy(data + 2, value, len);

  CTRPSend(data, len + 2, CRTP_PORT_PARAM, 2);
}

//=========================== private =========================================

//=========================== interrup handlers ===============================
