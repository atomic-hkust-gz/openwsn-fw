/**
 *
 * @file cf_api_commander_high_level.c
 * @brief The high-level commander API for Crazyflie 2.X.
 *    this API is migrated from the higher level code like the python client.
 *    this API can be used for Crazyflie 2.X self-control via the nRF51822 on-board chip. (now for the OpenWSN project)
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

/*
 * HighLevel Coordinate System:
 *
 *              ^ X+
 *              |
 *              |
 *              |
 *    Y+ <------+
 *             (0,0)
 *
 */

#include <string.h>

// crazyflie bsp
#include "cf_api_commander_high_level.h"
#include "cf_param.h"
#include "cf_ctrp.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void _high_level_sendCommand(const enum TrajectoryCommand_e command_type, void *values, uint8_t size);

//=========================== public ==========================================

// #@#@#@# After sending a high-level command, the param 'enHighLevel' should be set to 1 !!!!!!
void high_level_enable()
{
  uint8_t data[1] = {1};
  param_set(index_commander_enHighLevel, data, 1);
}

void high_level_takeoff(float absolute_height_m, float duration_s, float yaw)
{

  high_level_enable(); // let the high level commander be enabled (necessary)

  struct data_takeoff_2 takeoffValues;
  takeoffValues.groupMask = 0x00;

  takeoffValues.height = absolute_height_m;
  takeoffValues.duration = duration_s;

  if (yaw == 0.0)
  {
    takeoffValues.yaw = 0.0;
    takeoffValues.useCurrentYaw = true;
  }
  else
  {
    takeoffValues.yaw = yaw;
    takeoffValues.useCurrentYaw = false;
  }

  _high_level_sendCommand(COMMAND_TAKEOFF_2, &takeoffValues, sizeof(takeoffValues));
}

void high_level_land(float absolute_height_m, float duration_s, float yaw)
{

  struct data_land_2 landValues;
  landValues.groupMask = 0x00;

  landValues.height = absolute_height_m;
  landValues.duration = duration_s;

  if (yaw == 0.0)
  {
    landValues.yaw = 0.0;
    landValues.useCurrentYaw = true;
  }
  else
  {
    landValues.yaw = yaw;
    landValues.useCurrentYaw = false;
  }

  _high_level_sendCommand(COMMAND_LAND_2, &landValues, sizeof(landValues));
}

void high_level_stop()
{
  struct data_stop stopValues;
  stopValues.groupMask = 0x00;

  _high_level_sendCommand(COMMAND_STOP, &stopValues, sizeof(stopValues));
}

void high_level_goto(float x, float y, float z, float yaw, float duration_s, bool relative)
{
  struct data_go_to go_toValues;
  go_toValues.groupMask = 0x00;
  go_toValues.relative = relative;
  go_toValues.x = x;
  go_toValues.y = y;
  go_toValues.z = z;
  go_toValues.yaw = yaw;
  go_toValues.duration = duration_s;

  _high_level_sendCommand(COMMAND_GO_TO, &go_toValues, sizeof(go_toValues));
}

//=========================== private =========================================

void _high_level_sendCommand(const enum TrajectoryCommand_e command_type, void *values, uint8_t size)
{
  uint8_t data[CRTP_MAX_DATA_SIZE];
  data[0] = command_type;
  memcpy(data + 1, values, size);
  data[size + 1] = 0; // TODO:这里去掉0的话飞机就会翻车，原因未知（已确定不是数据拷贝长度的问题。将data数组初始化为0也是一个选择

  CTRPSend(data, size + 1, CRTP_PORT_SETPOINT_HL, 0); // ctrp_data_len = values_len + command_type_len
}

//=========================== interrup handlers ===============================