/**
 * @file cf_crazyflie.h
 * @brief crazyflie all operation
 * @author Lan HUANG(YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#ifndef __CF_CRAZYFLIE_H__
#define __CF_CRAZYFLIE_H__

#define V_SLOCAL_REVISION "0"
#define V_SREVISION "beb14b38ce06"
#define V_STAG "2024.2"
#define V_MODIFIED true

// Generic setpoint packet ----------------------------------------
struct notifySetpointsStopPacket {
  uint32_t remainValidMillisecs;
} __attribute__((packed));  // byte:4=4

struct CommanderCrtpLegacyValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
} __attribute__((packed)); // byte:4+4+4+2=14

struct hoverPacket_s {
  float vx;           // m/s in the body frame of reference
  float vy;           // ...
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed)); // byte:4+4+4+4=16


// High level commander ----------------------------------------

struct data_go_to {
  uint8_t groupMask; // mask for which CFs this should apply to
  uint8_t relative;  // set to true, if position/yaw are relative to current setpoint
  float x; // m
  float y; // m
  float z; // m
  float yaw; // rad
  float duration; // sec
} __attribute__((packed)); // byte:1+1+4+4+4+4+4=22
struct data_takeoff_2 {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));  // byte:1+4+4+1+4=14

struct data_land_2 {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));  // byte:1+4+4+1+4=14


// Communication ----------------------------------------


#define CRTP_MAX_DATA_SIZE 30
struct CRTPPacket
{
  uint8_t size; //< Size of data
  union
  {
    struct
    {
      union
      {
        uint8_t header; //< Header selecting channel and port
        struct
        {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel : 2; //< Selected channel within port
          uint8_t reserved : 2;
          uint8_t port : 4; //< Selected port
#else
          uint8_t channel : 2;
          uint8_t port : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE + 1]; //< The full packet "raw"
  };
} __attribute__((packed));





//====================================

void syslinkHandle();
void crazyflieInit();
void sendNullCTRPPackage();

void test_SetHoverValue(float vx, float vy, float yawrate, float zDistance);
void test_SendHover();

void test_SendEmergencyStop();
void test_send_notify_setpoint_stop();

void test_HL_SetGoToValue(float x, float y, float h);
void test_HL_SendGoto();
void test_HL_SendTakeOff();
void test_HL_SendLand();

void test_GetTOC();
void test_param_write_enHL(uint8_t value);
void test_param_write_stabilizer_controller(uint8_t value);
void test_param_write_kalman_estimator(uint8_t value);
void test_param_write_deck_Flow2(uint8_t value);
void test_read_param(uint16_t index);

#endif
