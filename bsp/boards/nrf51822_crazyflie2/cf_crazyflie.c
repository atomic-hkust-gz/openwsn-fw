
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
#include "memory.h"
#include "ownet.h"

#define BLE
#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

// ddd =======================
#define index_deck_bcFlow2 10
#define index_commander_enHighLevel 89
#define index_kalman_resetEstimation 112
#define index_stabilizer_controller 140

// var =======================
uint16_t param_index = 0;
bool sl_cache_IsNotEmpty = false;
bool syslink_rx = false;
bool stm_ready = false;

static int lastSyslinkSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;

// struct ====================

// generic
struct notifySetpointsStopPacket notifySetpointsStopValues;
struct CommanderCrtpLegacyValues commanderValues;
struct hoverPacket_s hoveValues;

// highlevel
struct data_go_to go_toValues;
struct data_takeoff_2 takeoffValues;
struct data_land_2 landValues;

// cache
static struct CRTPPacket ctrp_pk;
static struct CRTPPacket ctrp_pk_null = {.port = 15, .channel = 3, .size = 0};
static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;

// func =======================

// Send NULL to keep syslink alive. Port 15, channel 3
void sendNullCTRPPackage()
{
  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk_null.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk_null.raw, slTxPacket.length);

  syslinkSend(&slTxPacket);
  //syslinkDeactivateUntilPacketReceived();
}

void crazyflieInit()
{

  // Crazyflie init
  pmInit();
  uartInit();
  systickInit();
  memoryInit();
  pm_boot_all(); // boot STM32

  // 构造Commander数据
  commanderValues.roll = 0.0;
  commanderValues.pitch = -0.0;
  commanderValues.yaw = 0.0;
  commanderValues.thrust = 0;

  // 构造hover数据
  hoveValues.vx = 0.0;
  hoveValues.vy = 0.0;
  hoveValues.yawrate = 0.0;
  hoveValues.zDistance = 0.0;

  // 构造go_to数据
  go_toValues.groupMask = 0;
  go_toValues.relative = true;
  go_toValues.x = 0.0;
  go_toValues.y = 0.0;
  go_toValues.z = 0.0;
  go_toValues.yaw = 0.0;
  go_toValues.duration = 0.0;

  // 构造land数据
  landValues.groupMask = 0;
  landValues.height = 0.0;
  landValues.yaw = 0.0;
  landValues.useCurrentYaw = true;
  landValues.duration = 1;

  // 构造notifySetpointsStop数据
  notifySetpointsStopValues.remainValidMillisecs = 0;

  // 构造takeoff数据
  takeoffValues.groupMask = 0x00;
  takeoffValues.yaw = 0.0;
  takeoffValues.useCurrentYaw = true;
  takeoffValues.height = 0.5;
  takeoffValues.duration = 1.0;

  // sendNullCTRPPackage(); // 向stm发出第一个包，否则两边会互相等待
}

// ================= getter and setter ==================

// High-Level GoTo
void test_HL_SetGoToValue(float x, float y, float h)
{
  go_toValues.z = h;
  go_toValues.duration = h / 0.5;
}

void test_SetHoverValue(float vx, float vy, float yawrate, float zDistance)
{
  hoveValues.zDistance = zDistance;
}

// ===================== callback ========================
void syslinkHandle()
{

  // Receive
  syslink_rx = syslinkReceive(&slRxPacket); // try to receive

  if (syslink_rx)
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

      // #=#=#=#=#=#= For Debug =#=#=##=#=#=

      if (systickGetTick() > 16500)
      {
        int ccc;
        ccc++;
      }

      // catch the TOC Item
      if (port == 2 && channel == 0)
      {
        if (data[4] == 'd' && data[6] == 'c') //
        {
          int a;
          a++;
        }
      }

      // catch the param value read
      if (port == 2 && channel == 1)
      {
        int a;
        a++;
      }

      // catch the takeof
      if (port == 2 && channel == 2)
      {
        int a;
        a++;
      }
      // #=#=#=#=#=#= Debug End =#=#=##=#=#=

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
      // 全部置0
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
  if (sl_cache_IsNotEmpty)
  {
    syslinkSend(&slTxPacket);
    sl_cache_IsNotEmpty = false;
  }
  else
  {
    // if (stm_ready)
    //   sendNullCTRPPackage();
    if (systickGetTick() - lastSyslinkSendTime > SYSLINK_SEND_PERIOD_MS)
    {
      lastSyslinkSendTime = systickGetTick();
      sendNullCTRPPackage();
    }
  }
}

// ======================================================

// !!! Emergency Stop !!!
void test_SendEmergencyStop()
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 6;
  ctrp_pk.channel = 1; // Generic localization
  ctrp_pk.reserved = 0;

  // 通过Generic localization触发EmergencyStop
  ctrp_pk.size = 1;
  ctrp_pk.data[0] = 0x03; // 0x03:EMERGENCY_STOP

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_send_notify_setpoint_stop()
{
  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 7;    // Gereric setpoint
  ctrp_pk.channel = 1; // Meta
  ctrp_pk.reserved = 0;

  // 通过Generic localization触发EmergencyStop
  ctrp_pk.size = sizeof(notifySetpointsStopValues) + 1;
  ctrp_pk.data[0] = 0x00; // 0x00:NOTIFY_SETPOINTS_STOP
  memcpy(ctrp_pk.data + 1, &notifySetpointsStopValues, sizeof(notifySetpointsStopValues));

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_SendHover()
{
  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 7;    // Gereric setpoint
  ctrp_pk.channel = 0; // Hover
  ctrp_pk.reserved = 0;

  // 通过Generic localization触发EmergencyStop
  ctrp_pk.size = sizeof(hoveValues) + 1;
  ctrp_pk.data[0] = 0x05; // 0x00:HOVER
  memcpy(ctrp_pk.data + 1, &hoveValues, sizeof(hoveValues));

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

// ## High-Level Commander

void test_HL_SendGoto()
{
  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 8;
  ctrp_pk.channel = 0; // High-Level Commander
  ctrp_pk.reserved = 0;

  // 通过High-Level Commander触发Land
  ctrp_pk.size = sizeof(go_toValues) + 1;
  ctrp_pk.data[0] = 0x04; // 0x04:GO_TO
  memcpy(ctrp_pk.data + 1, &go_toValues, sizeof(go_toValues));

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_HL_SendLand()
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 8;
  ctrp_pk.channel = 0; // High-Level Commander
  ctrp_pk.reserved = 0;

  // 通过High-Level Commander触发Land
  ctrp_pk.size = sizeof(landValues) + 1;
  ctrp_pk.data[0] = 0x08; // 0x08:COMMAND_LAND_2
  memcpy(ctrp_pk.data + 1, &landValues, sizeof(landValues));

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_HL_SendTakeOff()
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 8;
  ctrp_pk.channel = 0; // High-Level Commander
  ctrp_pk.reserved = 0;

  // 通过High-Level Commander触发Land
  ctrp_pk.size = sizeof(takeoffValues) + 1;
  ctrp_pk.data[0] = 0x07; // 0x07:COMMAND_TAKEOFF_2
  memcpy(ctrp_pk.data + 1, &takeoffValues, sizeof(takeoffValues));

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

//=================

void test_GetTOC()
{
  // Port2 Channel0

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 2;
  ctrp_pk.channel = 0;
  ctrp_pk.reserved = 0;

  // 通过
  ctrp_pk.size = 3;
  ctrp_pk.data[0] = 0x02;
  ctrp_pk.data[1] = param_index & 0xFF;
  ctrp_pk.data[2] = (param_index >> 8) & 0xFF;

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);

  param_index++;
}

void test_param_write_enHL(uint8_t value)
{
  // Port2 Channel2

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 2;
  ctrp_pk.channel = 2;
  ctrp_pk.reserved = 0;

  // 通过
  uint16_t varid = index_commander_enHighLevel;

  ctrp_pk.size = 3;

  // struct.pack('<H', varid)
  ctrp_pk.data[0] = varid & 0xFF;
  ctrp_pk.data[1] = (varid >> 8) & 0xFF;

  // pk.data += struct.pack(element.pytype, value_nr)
  ctrp_pk.data[2] = value;

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_param_write_stabilizer_controller(uint8_t value)
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 2;
  ctrp_pk.channel = 2;
  ctrp_pk.reserved = 0;

  // 通过
  uint16_t varid = index_stabilizer_controller;
  ctrp_pk.size = 3;

  // struct.pack('<H', varid)
  ctrp_pk.data[0] = varid & 0xFF;
  ctrp_pk.data[1] = (varid >> 8) & 0xFF;

  // pk.data += struct.pack(element.pytype, value_nr)
  ctrp_pk.data[2] = value;

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_param_write_kalman_estimator(uint8_t value)
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 2;
  ctrp_pk.channel = 2;
  ctrp_pk.reserved = 0;

  // 通过
  uint16_t varid = index_kalman_resetEstimation;
  ctrp_pk.size = 3;

  // struct.pack('<H', varid)
  ctrp_pk.data[0] = varid & 0xFF;
  ctrp_pk.data[1] = (varid >> 8) & 0xFF;

  // pk.data += struct.pack(element.pytype, value_nr)
  ctrp_pk.data[2] = value;

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_param_write_deck_Flow2(uint8_t value)
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 2;
  ctrp_pk.channel = 2;
  ctrp_pk.reserved = 0;

  // 通过
  uint16_t varid = index_deck_bcFlow2;
  ctrp_pk.size = 3;

  // struct.pack('<H', varid)
  ctrp_pk.data[0] = varid & 0xFF;
  ctrp_pk.data[1] = (varid >> 8) & 0xFF;

  // pk.data += struct.pack(element.pytype, value_nr)
  ctrp_pk.data[2] = value;

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}

void test_read_param(uint16_t param_index)
{

  // 构造CRTP包
  memset(&ctrp_pk, 0, sizeof(ctrp_pk));
  ctrp_pk.port = 2;
  ctrp_pk.channel = 1;
  ctrp_pk.reserved = 0;

  // 通过
  ctrp_pk.size = 2;
  ctrp_pk.data[0] = param_index & 0xFF;
  ctrp_pk.data[1] = (param_index >> 8) & 0xFF;

  // 构造Syslink包
  memset(&slTxPacket, 0, sizeof(slTxPacket));
  slTxPacket.type = 0x00; // SYSLINK_RADIO_RAW
  slTxPacket.length = ctrp_pk.size + 1;
  memcpy(slTxPacket.data, &ctrp_pk.raw, slTxPacket.length);

  // 发送Syslink包
  sl_cache_IsNotEmpty = true;
  // syslinkSend(&slTxPacket);
}
