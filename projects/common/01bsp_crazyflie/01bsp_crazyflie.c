/**
\brief This is a program which provide Crazyflie Boot & Communication bsp

\author Lan HUANG <yelloooblue@outlook.com>, June 2024
*/

#include "stdint.h"
#include "stdio.h"
#include "string.h"

// bsp modules required
#include "board.h"
#include "sctimer.h"
#include "leds.h"

// crazyflie
#include "cf_pm.h"
#include "cf_systick.h"
#include "cf_uart.h"
#include "cf_syslink.h"
#include "cf_platform.h"

//=========================== defines =========================================

#define BLE
#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif


#define V_SLOCAL_REVISION "0"
#define V_SREVISION "beb14b38ce06"
#define V_STAG "2024.2"
#define V_MODIFIED true

//=========================== variables =======================================

static void handleSyslinkEvents(bool slReceived);

//=========================== prototypes ======================================

//struct syslinkPacket pp;
static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;

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

void mainloop();
void setpoint_test();
void setUpValue();
void constructPackage();

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
   
   // initialize the board
   board_init();

   // Crazyflie init
   pmInit();
   uartInit();
   systickInit();

   //// Crazyflie boot
   //pmSetState(pmSysRunning);

   crazyflie_init();

   leds_all_on();

   setUpValue();//setpoint
   constructPackage();

   while(1){
      mainloop();
   }
   

}

//=========================== callbacks =======================================

int tickk;
bool rr;
bool sent=false;

struct CommanderCrtpLegacyValues commanderValues;
struct CRTPPacket ctrp;
struct syslinkPacket slp;


void setUpValue(){
  // 构造Commander数据
  commanderValues.roll = 0.0;
  commanderValues.pitch = -0.0;
  commanderValues.yaw = 0.0;
  commanderValues.thrust = 0;
}

void constructPackage(){
  // 构造CRTP包
  memset(&ctrp, 0, sizeof(ctrp)); // 使用 memset 函数将 pk 初始化为零
  ctrp.channel = 0;
  ctrp.reserved = 0;
  ctrp.port = 3;
  ctrp.size = 14;
  memcpy(ctrp.data, &commanderValues, 14);

  // 构造Syslink包
  memset(&slp, 0, sizeof(slp)); // 使用 memset 函数将 pk 初始化为零
  slp.type = 0x00;
  slp.length = ctrp.size + 1;
  memcpy(slp.data, &ctrp.raw, 15);
}

void mainloop(){

    handleSyslinkEvents(syslinkReceive(&slRxPacket));

    tickk = systickGetTick();

    if (tickk >= 15000 && tickk < 16000) {
        commanderValues.thrust = 1100;
        constructPackage();
        sent=true;
    }

    if (tickk >= 16000 && tickk < 17000) {
        commanderValues.thrust = 1300;
        constructPackage();
        sent=true;
    }

    if (tickk >= 17000 && tickk < 18000) {
        commanderValues.thrust = 1100;
        constructPackage();
        sent=true;
    }

    if (tickk >= 18000) {
        commanderValues.thrust = 0;
        constructPackage();
        sent=true;
    }

    setpoint_test();

}


void setpoint_test(){

    syslinkSend(&slp);
  
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