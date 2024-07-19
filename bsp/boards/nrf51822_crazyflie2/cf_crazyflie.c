/**
 * @file cf_crazyflie.c
 * @brief crazyflie operations
 * @author Lan HUANG(YelloooBlue@outlook.com)
 * @date June 2024
 *
 */

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
#include "cf_memory.h"
#include "cf_ownet.h"

//=========================== defines =========================================

#define BLE
#ifdef BLE
int volatile bleEnabled = 1;
#else
int volatile bleEnabled = 0;
#endif

// Parameters index
#define index_deck_bcFlow2 10
#define index_commander_enHighLevel 89
#define index_kalman_resetEstimation 112
#define index_stabilizer_controller 140

// Syslink & CRTP defines
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

//=========================== variables =======================================
static bool stm_ready = false;
static int lastSyslinkSendTime = SYSLINK_STARTUP_DELAY_TIME_MS;

// cache
static struct syslinkPacket slRxPacket;
static struct syslinkPacket slTxPacket;

//=========================== prototypes ======================================
void _CTRPSend(void *data, uint8_t size, uint8_t port, uint8_t channel);
void _CTRPSend_NULL();
void _high_level_sendCommand(const enum TrajectoryCommand_e command_type, void *values, uint8_t size);

//=========================== public ==========================================

void crazyflieInit()
{
  // Crazyflie init
  pmInit();
  uartInit();
  systickInit();
  memoryInit();
  pm_boot_all(); // boot STM32
}

void syslinkHandle()
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
      // int port = (slRxPacket.data[0] >> 4) & 0x0F;
      // int channel = slRxPacket.data[0] & 0x03;
      // char *data = slRxPacket.data + 1;

      // @you can debug here...

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
    _CTRPSend_NULL(); // just send NULL to keep syslink alive, otherwise the STM32 will blocked until a packet is received
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


// Safety
void EmergencyStop()
{
  uint8_t data[1] = {0x03}; // 0x03:EMERGENCY_STOP
  _CTRPSend(data, 1, CRTP_PORT_LOCALIZATION, 1);
}



// Parameters Operation
void param_set(uint16_t id, const void *value, uint8_t len)
{

  uint8_t data[CRTP_MAX_DATA_SIZE];
  data[0] = id & 0xFF;
  data[1] = (id >> 8) & 0xFF;

  memcpy(data + 2, value, len);

  _CTRPSend(data, len + 2, CRTP_PORT_PARAM, 2);
}

// High-Level Commander
// #@#@#@# After sending a high-level command, the param 'enHighLevel' should be set to 1 !!!!!!
void high_level_enable()
{
  uint8_t data[1] = {1};
  param_set(index_commander_enHighLevel, data, 1);
}

// Trajectory
void high_level_takeoff(float absolute_height_m, float duration_s, float yaw)
{

  high_level_enable(); // let the high level commander be enabled (necessary)

  struct data_takeoff_2 takeoffValues;
  takeoffValues.groupMask = 0x00;

  takeoffValues.height = absolute_height_m;
  takeoffValues.duration = duration_s;

  if (yaw == 0.0)
  {
    takeoffValues.yaw = 0.0;
    takeoffValues.useCurrentYaw = true;
  }
  else
  {
    takeoffValues.yaw = yaw;
    takeoffValues.useCurrentYaw = false;
  }

  _high_level_sendCommand(COMMAND_TAKEOFF_2, &takeoffValues, sizeof(takeoffValues));
}

void high_level_land(float absolute_height_m, float duration_s, float yaw)
{

  struct data_land_2 landValues;
  landValues.groupMask = 0x00;

  landValues.height = absolute_height_m;
  landValues.duration = duration_s;

  if (yaw == 0.0)
  {
    landValues.yaw = 0.0;
    landValues.useCurrentYaw = true;
  }
  else
  {
    landValues.yaw = yaw;
    landValues.useCurrentYaw = false;
  }

  _high_level_sendCommand(COMMAND_LAND_2, &landValues, sizeof(landValues));
}

void high_level_stop()
{
  struct data_stop stopValues;
  stopValues.groupMask = 0x00;

  _high_level_sendCommand(COMMAND_STOP, &stopValues, sizeof(stopValues));
}

void high_level_goto(float x, float y, float z, float yaw, float duration_s, bool relative)
{
  struct data_go_to go_toValues;
  go_toValues.groupMask = 0x00;
  go_toValues.relative = relative;
  go_toValues.x = x;
  go_toValues.y = y;
  go_toValues.z = z;
  go_toValues.yaw = yaw;
  go_toValues.duration = duration_s;

  _high_level_sendCommand(COMMAND_GO_TO, &go_toValues, sizeof(go_toValues));
}

//=========================== private =========================================

// CTRP

void _CTRPSend(void *data, uint8_t size, uint8_t port, uint8_t channel)
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
void _CTRPSend_NULL()
{
  _CTRPSend(NULL, 0, CRTP_PORT_LINK, 3);
}

void _high_level_sendCommand(const enum TrajectoryCommand_e command_type, void *values, uint8_t size)
{
  uint8_t data[CRTP_MAX_DATA_SIZE];
  data[0] = command_type;
  memcpy(data + 1, values, size);
  data[size + 1] = 0; // TODO:这里去掉0的话飞机就会翻车，原因未知（已确定不是数据拷贝长度的问题。将data数组初始化为0也是一个选择

  _CTRPSend(data, size + 1, CRTP_PORT_SETPOINT_HL, 0); // ctrp_data_len = values_len + command_type_len
}

//=========================== interrup handlers ===============================