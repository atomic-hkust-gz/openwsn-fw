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


//#include <stdbool.h>

//====================================

struct CommanderCrtpLegacyValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
};

#define CRTP_MAX_DATA_SIZE 30
struct CRTPPacket
{
  uint8_t size;                         //< Size of data
  union {
    struct {
      union {
        uint8_t header;                 //< Header selecting channel and port
        struct {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel     : 2;      //< Selected channel within port
          uint8_t reserved    : 2;
          uint8_t port        : 4;      //< Selected port
#else
          uint8_t channel  : 2;
          uint8_t port     : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];  //< The full packet "raw"
  };
};

//====================================



void crazyflieInit();
void test_changeThrust(int t);
void test_sendSetpointSyslinkPkg();
void syslink_loop();
static void handleSyslinkEvents(bool slReceived);

#endif
