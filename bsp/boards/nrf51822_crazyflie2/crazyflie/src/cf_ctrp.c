/**
 *
 * @file cf_ctrp.c
 * @brief Crazyflie communication CTRP
 *    this API is migrated from the higher level code like the python client
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

#include <string.h>

// crazyflie bsp
#include "cf_ctrp.h"
#include "cf_syslink.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void CTRPSend(void *data, uint8_t size, uint8_t port, uint8_t channel)
{
  struct CRTPPacket ctrp_pk;
  ctrp_pk.port = port;
  ctrp_pk.channel = channel;
  ctrp_pk.reserved = 0;
  ctrp_pk.size = size;
  memcpy(ctrp_pk.data, data, size);

  struct syslinkPacket syslink_pk;
  syslink_pk.type = 0x00;
  syslink_pk.length = ctrp_pk.size + 1;
  memcpy(syslink_pk.data, &ctrp_pk.raw, syslink_pk.length); // syslink_data_len = ctrp_raw_len = ctrp_data_len + ctrp_header_len

  syslinkSend(&syslink_pk);
}

// Send NULL to keep syslink alive. Port 15, channel 3
void CTRPSend_NULL()
{
  CTRPSend(NULL, 0, CRTP_PORT_LINK, 3);
}

//=========================== private =========================================

//=========================== interrup handlers ===============================