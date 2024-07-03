
#include "stdint.h"
#include "stdio.h"
#include "string.h"

// crazyflie bsp
#include "cf_pm.h"
#include "cf_systick.h"
#include "cf_uart.h"
#include "cf_syslink.h"
#include "cf_platform.h"
#include "cf_crazyflie.h"

#define BLE
#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif


// var =======================

struct CommanderCrtpLegacyValues commanderValues;
struct CRTPPacket ctrp_pk;
struct syslinkPacket syslink_pk;

static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;

// func =======================

void constructPackage();

void crazyflieInit(){
  
  // Crazyflie init
  pmInit();
  uartInit();
  systickInit();
  pm_boot_all(); //boot STM32


  // 构造Commander数据
  commanderValues.roll = 0.0;
  commanderValues.pitch = -0.0;
  commanderValues.yaw = 0.0;
  commanderValues.thrust = 0;

  constructPackage();

}

void constructPackage(){
  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.channel = 0;
  ctrp_pk.reserved = 0;
  ctrp_pk.port = 3;
  ctrp_pk.size = 14;
  memcpy(ctrp_pk.data, &commanderValues, 14);

  // 构造Syslink包
  memset(&syslink_pk, 0, sizeof(syslink_pk));
  syslink_pk.type = 0x00;
  syslink_pk.length = ctrp_pk.size + 1;
  memcpy(syslink_pk.data, &ctrp_pk.raw, 15);
}


void test_changeThrust(int t){
  commanderValues.thrust = t;
  constructPackage();
}

void test_sendSetpointSyslinkPkg(){
  syslinkReceive(&slRxPacket);
  syslinkSend(&syslink_pk);
}


// The NRF must receive packets from the STM; otherwise, the STM will not receive the packets sent by the NRf
void syslink_loop(){

  //syslinkReceive(&slRxPacket); //!!!! nrf must received syslink packet from stm

  //handleSyslinkEvents(syslinkReceive(&slRxPacket));
}


static void handleSyslinkEvents(bool slReceived)
{
  if (slReceived)
  {
    switch (slRxPacket.type)
    {
      case SYSLINK_RADIO_RAW:
        break;
      case SYSLINK_RADIO_CHANNEL:
        if(slRxPacket.length == 1)
        {
          //esbSetChannel(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_CHANNEL;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          syslinkSend(&slTxPacket);

          //debugProbeReceivedChan = true;
        }
        break;
      case SYSLINK_RADIO_DATARATE:
        if(slRxPacket.length == 1)
        {
          //esbSetDatarate(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_DATARATE;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          syslinkSend(&slTxPacket);

          //debugProbeReceivedRate = true;
        }
        break;
      case SYSLINK_RADIO_CONTWAVE:
        if(slRxPacket.length == 1) {
          //esbSetContwave(slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          syslinkSend(&slTxPacket);
        }
        break;
      case SYSLINK_RADIO_ADDRESS:
        if(slRxPacket.length == 5)
        {
          uint64_t address = 0;
          memcpy(&address, &slRxPacket.data[0], 5);
          //esbSetAddress(address);

          slTxPacket.type = SYSLINK_RADIO_ADDRESS;
          memcpy(slTxPacket.data, slRxPacket.data, 5);
          slTxPacket.length = 5;
          syslinkSend(&slTxPacket);

          //debugProbeReceivedAddress = true;
        }
        break;
      case SYSLINK_RADIO_POWER:
        if(slRxPacket.length == 1)
        {
          //esbSetTxPowerDbm((int8_t)slRxPacket.data[0]);

          slTxPacket.type = SYSLINK_RADIO_POWER;
          slTxPacket.data[0] = slRxPacket.data[0];
          slTxPacket.length = 1;
          syslinkSend(&slTxPacket);
        }
        break;
      case SYSLINK_PM_ONOFF_SWITCHOFF:
        //pmSetState(pmAllOff);
        break;
      case SYSLINK_OW_GETINFO:
      case SYSLINK_OW_READ:
      case SYSLINK_OW_SCAN:
      case SYSLINK_OW_WRITE:
        //if (memorySyslink(&slRxPacket)) {
        //  syslinkSend(&slRxPacket);
        //}
        break;
      case SYSLINK_RADIO_P2P_BROADCAST:
        // Check that bluetooth is disabled and disable it if not
        if (bleEnabled) {
          //disableBle();
        }
        // Send the P2P packet immediately without buffer
        //esbSendP2PPacket(slRxPacket.data[0],&slRxPacket.data[1],slRxPacket.length-1);
        break;
      case SYSLINK_SYS_NRF_VERSION:{
        size_t len = strlen(V_STAG);
        slTxPacket.type = SYSLINK_SYS_NRF_VERSION;

        memcpy(&slTxPacket.data[0], V_STAG, len);

        if (V_MODIFIED) {
          slTxPacket.data[len] = '*';
          len += 1;
        }
        // Add platform identifier
        slTxPacket.data[len++] = ' ';
        slTxPacket.data[len++] = '(';
        memcpy(&slTxPacket.data[len], (void*)PLATFORM_DEVICE_DATA_FLASH_POS + 2, 4);
        len += 4;
        slTxPacket.data[len++] = ')';
        slTxPacket.data[len++] = '\0';

        slTxPacket.length = len;
        syslinkSend(&slTxPacket);
      } break;
      case SYSLINK_PM_BATTERY_AUTOUPDATE:
        //syslinkEnableBatteryMessages();
        break;
      case SYSLINK_PM_SHUTDOWN_ACK:
        //shutdownReceivedAck();
        break;
      case SYSLINK_PM_LED_ON:
        //LED_ON();
        break;
      case SYSLINK_PM_LED_OFF:
        //LED_OFF();
        break;
      case SYSLINK_DEBUG_PROBE:
      {
        slTxPacket.type = SYSLINK_DEBUG_PROBE;
        //slTxPacket.data[0] = debugProbeReceivedAddress;
        //slTxPacket.data[1] = debugProbeReceivedChan;
        //slTxPacket.data[2] = debugProbeReceivedRate;
        slTxPacket.data[3] = (uint8_t)uartDropped();
        slTxPacket.data[4] = uartGetError();
        slTxPacket.data[5] = uartGetErrorCount();
        slTxPacket.data[6] = syslinkGetRxCheckSum1ErrorCnt();
        slTxPacket.data[7] = syslinkGetRxCheckSum2ErrorCnt();

        slTxPacket.length = 8;
        syslinkSend(&slTxPacket);
      }
        break;
    }
  }
}