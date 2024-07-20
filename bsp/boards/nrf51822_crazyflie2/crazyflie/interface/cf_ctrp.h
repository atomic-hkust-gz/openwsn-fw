/**
 * 
 * @file cf_ctrp.h
 * @brief Crazyflie communication CTRP
 *    this API is migrated from the higher level code like the python client
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#ifndef __CF_CTRP_H__
#define __CF_CTRP_H__

#include <stdint.h>

//=========================== define ==========================================

#define CRTP_MAX_DATA_SIZE 30

//=========================== typedef =========================================

typedef enum
{
  CRTP_PORT_CONSOLE = 0x00,
  CRTP_PORT_PARAM = 0x02,
  CRTP_PORT_SETPOINT = 0x03,
  CRTP_PORT_MEM = 0x04,
  CRTP_PORT_LOG = 0x05,
  CRTP_PORT_LOCALIZATION = 0x06,
  CRTP_PORT_SETPOINT_GENERIC = 0x07,
  CRTP_PORT_SETPOINT_HL = 0x08,
  CRTP_PORT_PLATFORM = 0x0D,
  CRTP_PORT_LINK = 0x0F,
} CRTPPort;

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

//=========================== variables =======================================

//=========================== prototypes ======================================

void CTRPSend(void *data, uint8_t size, uint8_t port, uint8_t channel);
void CTRPSend_NULL();

#endif //__CF_CTRP_H__
