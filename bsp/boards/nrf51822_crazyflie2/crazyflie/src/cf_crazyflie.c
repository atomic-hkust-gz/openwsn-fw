/**
 *
 * @file cf_crazyflie.c
 * @brief crazyflie basic operation, like boot
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date June 2024
 *
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>

// crazyflie bsp
#include "cf_crazyflie.h"
#include "cf_platform.h"
#include "cf_pm.h"
#include "cf_systick.h"
#include "cf_memory.h"
#include "cf_ownet.h"
#include "cf_uart.h"
#include "cf_syslink.h"
#include "cf_ctrp.h"
#include "cf_multiranger.h"

#include "single_status_led.h"

//=========================== defines =========================================

#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

//=========================== variables =======================================
static bool stm_ready = false;
static int lastSyslinkSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;

// cache
static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;

//=========================== prototypes ======================================

void _syslinkHandle();

//=========================== public ==========================================

void crazyflieInit()
{
  //uartInit(); // move to pm
  systickInit();
  memoryInit();
  pmInit();
  //pm_boot_all(); // boot STM32
  pmSetState(pmSysRunning);

  status_led_init();
}

void crazyflieShutdown()
{
  struct syslinkPacket slTxPacket = {
    .type = SYSLINK_PM_SHUTDOWN_REQUEST,
  };
  syslinkSend(&slTxPacket);
  pmSetState(pmAllOff);
}

void crazyflieHandle()
{
  _syslinkHandle();
  status_led_handle();
  pmProcess();
}

void crazyflieEmergencyStop()
{
  uint8_t data[1] = {0x03}; // 0x03:EMERGENCY_STOP
  CTRPSend(data, 1, CRTP_PORT_LOCALIZATION, 1);
}

//=========================== private =========================================

void _syslinkHandle()
{

  if (syslinkReceive(&slRxPacket)) // try to receive
  {
    if (!stm_ready && slRxPacket.type == 20) // stm ready message
      stm_ready = true;

    // Process
    switch (slRxPacket.type)
    {
    case SYSLINK_RADIO_RAW:

      // Decode ctrp package
      int port = (slRxPacket.data[0] >> 4) & 0x0F;
      int channel = slRxPacket.data[0] & 0x03;
      char *data = slRxPacket.data + 1;

      // @you can debug here...

      // catch the param value read
      if (port == 2 && channel == 1)
      {
        int a;
        a++;
      }

      // catch the param value WRITE result
      if (port == 2 && channel == 2)
      {
        int a;
        a++;
      }

      // log setting result
      if (port == 5 && channel == 1)
      {
        int a;
        a++;
      }


      // message route
      if (port == 5 && channel == 2) //Log Data
      {
        //ctrp data
        uint8_t *ctrp_data = slRxPacket.data + 1;

        mutiranger_handle(ctrp_data, slRxPacket.length - 1);
      }



      break;

    case SYSLINK_RADIO_CHANNEL:
      if (slRxPacket.length == 1)
      {
        // esbSetChannel(slRxPacket.data[0]);

        slTxPacket.type = SYSLINK_RADIO_CHANNEL;
        slTxPacket.data[0] = slRxPacket.data[0];
        slTxPacket.length = 1;
        syslinkSend(&slTxPacket);

        // debugProbeReceivedChan = true;
      }
      break;
    case SYSLINK_RADIO_DATARATE:
      if (slRxPacket.length == 1)
      {
        // esbSetDatarate(slRxPacket.data[0]);

        slTxPacket.type = SYSLINK_RADIO_DATARATE;
        slTxPacket.data[0] = slRxPacket.data[0];
        slTxPacket.length = 1;
        syslinkSend(&slTxPacket);

        // debugProbeReceivedRate = true;
      }
      break;
    case SYSLINK_RADIO_CONTWAVE:
      if (slRxPacket.length == 1)
      {
        // esbSetContwave(slRxPacket.data[0]);

        slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
        slTxPacket.data[0] = slRxPacket.data[0];
        slTxPacket.length = 1;
        syslinkSend(&slTxPacket);
      }
      break;
    case SYSLINK_RADIO_ADDRESS:
      if (slRxPacket.length == 5)
      {
        uint64_t address = 0;
        memcpy(&address, &slRxPacket.data[0], 5);
        // esbSetAddress(address);

        slTxPacket.type = SYSLINK_RADIO_ADDRESS;
        memcpy(slTxPacket.data, slRxPacket.data, 5);
        slTxPacket.length = 5;
        syslinkSend(&slTxPacket);

        // debugProbeReceivedAddress = true;
      }
      break;
    case SYSLINK_RADIO_POWER:
      if (slRxPacket.length == 1)
      {
        // esbSetTxPowerDbm((int8_t)slRxPacket.data[0]);

        slTxPacket.type = SYSLINK_RADIO_POWER;
        slTxPacket.data[0] = slRxPacket.data[0];
        slTxPacket.length = 1;
        syslinkSend(&slTxPacket);
      }
      break;
    case SYSLINK_PM_ONOFF_SWITCHOFF:
      // pmSetState(pmAllOff);
      break;
    case SYSLINK_OW_GETINFO:
    case SYSLINK_OW_READ:
    case SYSLINK_OW_SCAN:
    case SYSLINK_OW_WRITE:
      if (memorySyslink(&slRxPacket))
      {
        syslinkSend(&slRxPacket);
      }
      break;
    case SYSLINK_RADIO_P2P_BROADCAST:
      // Check that bluetooth is disabled and disable it if not
      // if (bleEnabled)
      // {
      //   disableBle();
      // }
      // // Send the P2P packet immediately without buffer
      // esbSendP2PPacket(slRxPacket.data[0], &slRxPacket.data[1], slRxPacket.length - 1);
      break;
    case SYSLINK_SYS_NRF_VERSION:
    {
      size_t len = strlen(V_STAG);
      slTxPacket.type = SYSLINK_SYS_NRF_VERSION;

      memcpy(&slTxPacket.data[0], V_STAG, len);

      if (V_MODIFIED)
      {
        slTxPacket.data[len] = '*';
        len += 1;
      }
      // Add platform identifier
      slTxPacket.data[len++] = ' ';
      slTxPacket.data[len++] = '(';
      memcpy(&slTxPacket.data[len], (void *)PLATFORM_DEVICE_DATA_FLASH_POS + 2, 4);
      len += 4;
      slTxPacket.data[len++] = ')';
      slTxPacket.data[len++] = '\0';

      slTxPacket.length = len;
      syslinkSend(&slTxPacket);
    }
    break;
    case SYSLINK_PM_BATTERY_AUTOUPDATE:
      // syslinkEnableBatteryMessages();
      break;
    case SYSLINK_PM_SHUTDOWN_ACK:
      // shutdownReceivedAck();
      break;
    case SYSLINK_PM_LED_ON:
      // LED_ON();
      break;
    case SYSLINK_PM_LED_OFF:
      // LED_OFF();
      break;
    case SYSLINK_DEBUG_PROBE:
    {
      slTxPacket.type = SYSLINK_DEBUG_PROBE;
      memset(slTxPacket.data, 0, 8);
      // slTxPacket.data[0] = debugProbeReceivedAddress;
      // slTxPacket.data[1] = debugProbeReceivedChan;
      // slTxPacket.data[2] = debugProbeReceivedRate;
      // slTxPacket.data[3] = (uint8_t)uartDropped();
      // slTxPacket.data[4] = uartGetError();
      // slTxPacket.data[5] = uartGetErrorCount();
      // slTxPacket.data[6] = syslinkGetRxCheckSum1ErrorCnt();
      // slTxPacket.data[7] = syslinkGetRxCheckSum2ErrorCnt();

      slTxPacket.length = 8;
      syslinkSend(&slTxPacket);
    }
    break;
    default:
      // sendNullCTRPPackage();
      break;
    }
  }

  // Send
  // TODO:optimize it
  if (systickGetTick() - lastSyslinkSendTime > SYSLINK_SEND_PERIOD_MS)
  {
    lastSyslinkSendTime = systickGetTick();
    CTRPSend_NULL(); // just send NULL to keep syslink alive, otherwise the STM32 will blocked until a packet is received
  }

  /*
    Tips:
    1. The STM32 Only sends data when it receives a packet from the NRF51822. So, we have to send a NULL packet to keep the syslink alive.
    2. You can not send data via syslink too frequently, otherwise the NRF51822 will overflow (the LED_M3 will blink slowly).
    3. The OneWire Packet will be exchanged at the beginning, the decks on Crazyflie will be detected and initialized. [important!]
    4. It's best not to send too many packets at the beginning, otherwise the OneWire Packet might be lost.

    @Author Lan HUANG(YelloooBlue@outlook.com) June 2024
  */
}

//=========================== interrup handlers ===============================