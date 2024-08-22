/**
 * 
 * @file cf_crazyflie.h
 * @brief crazyflie basic operation, like boot
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#ifndef __CF_CRAZYFLIE_H__
#define __CF_CRAZYFLIE_H__

#include <stdint.h>

//=========================== define ==========================================

// #define V_SLOCAL_REVISION "0" //useless now
// #define V_SREVISION "beb14b38ce06"
#define V_STAG "2024.2"
#define V_MODIFIED true

#define BLE

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void crazyflieInit();
void crazyflieShutdown();
void crazyflieHandle();
//void syslinkHandle();// move into crazyflieHandle
void crazyflieEmergencyStop();

#endif // __CF_CRAZYFLIE_H__

//@deprecated
//  void notifySetpointsStop(uint32_t remainValidMillisecs);          when switching from low-level to high-level, call it.
//  void resetEstimation();                                           reset the kalman filter (set this param to 0, then set it to 1)
//  void setStabilizerController(uint8_t controller);                 set the controller of the stabilizer (0: PID, 1: Mellinger)
//  void hover(float vx, float vy, float yawrate, float zDistance);   hover at the given position, replaced by high_level commands
//  void setCommanderSetpoint(...);                                   set the low-level commander setpoint