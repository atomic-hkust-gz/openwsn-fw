/**
 * 
 * @file cf_api_commander_high_level.h
 * @brief The high-level commander API for Crazyflie 2.X.
 *    this API is migrated from the higher level code like the python client.
 *    this API can be used for Crazyflie 2.X self-control via the nRF51822 on-board chip. (now for the OpenWSN project)
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#ifndef __CF_API_COMMANDER_HIGH_LEVEL__
#define __CF_API_COMMANDER_HIGH_LEVEL__

#include <stdint.h>
#include <stdbool.h>

//=========================== define ==========================================

#define index_commander_enHighLevel 89
#define CF_HL_MOVE_VEL 0.5 // Crazyflie high level move velocity (m/s)

//=========================== typedef =========================================

enum TrajectoryCommand_e
{
  COMMAND_SET_GROUP_MASK = 0, // Deprecated (removed after Dec 2024), use parameter hlCommander.groupmask instead
  COMMAND_TAKEOFF = 1,        // Deprecated (removed after August 2023), use COMMAND_TAKEOFF_2
  COMMAND_LAND = 2,           // Deprecated (removed after August 2023), use COMMAND_LAND_2
  COMMAND_STOP = 3,
  COMMAND_GO_TO = 4,
  COMMAND_START_TRAJECTORY = 5,
  COMMAND_DEFINE_TRAJECTORY = 6,
  COMMAND_TAKEOFF_2 = 7,
  COMMAND_LAND_2 = 8,
  COMMAND_TAKEOFF_WITH_VELOCITY = 9,
  COMMAND_LAND_WITH_VELOCITY = 10,
};

// vertical takeoff from current x-y position to given height
struct data_takeoff_2
{
  uint8_t groupMask;       // mask for which CFs this should apply to
  float height;            // m (absolute)
  float yaw;               // rad
  bool useCurrentYaw;      // If true, use the current yaw (ignore the yaw parameter)
  float duration;          // s (time it should take until target height is reached)
} __attribute__((packed)); // byte:1+4+4+1+4=14

// vertical land from current x-y position to given height
struct data_land_2
{
  uint8_t groupMask;       // mask for which CFs this should apply to
  float height;            // m (absolute)
  float yaw;               // rad
  bool useCurrentYaw;      // If true, use the current yaw (ignore the yaw parameter)
  float duration;          // s (time it should take until target height is reached)
} __attribute__((packed)); // byte:1+4+4+1+4=14

// stops the current trajectory (turns off the motors)
struct data_stop
{
  uint8_t groupMask;       // mask for which CFs this should apply to
} __attribute__((packed)); // byte:1=1

// "take this much time to go here, then hover"
struct data_go_to
{
  uint8_t groupMask;       // mask for which CFs this should apply to
  uint8_t relative;        // set to true, if position/yaw are relative to current setpoint
  float x;                 // m
  float y;                 // m
  float z;                 // m
  float yaw;               // rad
  float duration;          // sec
} __attribute__((packed)); // byte:1+1+4+4+4+4+4=22

// TODO: implement the Trajectory API.

//=========================== variables =======================================

//=========================== prototypes ======================================

void high_level_enable();
void high_level_takeoff(float absolute_height_m, float duration_s, float yaw);
void high_level_land(float absolute_height_m, float duration_s, float yaw);
void high_level_stop();
void high_level_goto(float x, float y, float z, float yaw, float duration, bool relative);

#endif // __CF_API_COMMANDER_HIGH_LEVEL__
