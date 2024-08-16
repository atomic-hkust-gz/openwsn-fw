/**
 *
 * @file single_status_led.h
 * @brief Because the nRF51822 on Crazyflie 2.X has only one LED, this interface is designed to control it.
 *          The status LED is used to indicate the status of the Crazyflie 2.X and the OpenWSN stack.
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date Aug 2024
 *
 */

#include "cf_multiranger.h"
#include "cf_ctrp.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

//=========================== defines =========================================

// log cmd
#define CMD_GET_ITEM 0    // original version: up to 255 entries
#define CMD_GET_INFO 1    // original version: up to 255 entries
#define CMD_GET_ITEM_V2 2 // version 2: up to 16k entries
#define CMD_GET_INFO_V2 3 // version 2: up to 16k entries

#define CONTROL_CREATE_BLOCK 0
#define CONTROL_APPEND_BLOCK 1
#define CONTROL_DELETE_BLOCK 2
#define CONTROL_START_BLOCK 3
#define CONTROL_STOP_BLOCK 4
#define CONTROL_RESET 5
#define CONTROL_CREATE_BLOCK_V2 6
#define CONTROL_APPEND_BLOCK_V2 7
#define CONTROL_START_BLOCK_V2 8

#define CHAN_TOC 0
#define CHAN_SETTINGS 1
#define CHAN_LOGDATA 2

/* Possible variable types */
#define LOG_UINT8 1
#define LOG_UINT16 2
#define LOG_UINT32 3
#define LOG_INT8 4
#define LOG_INT16 5
#define LOG_INT32 6
#define LOG_FLOAT 7
#define LOG_FP16 8

//=========================== variables =======================================

static struct mutiranger_data mutiranger_values = {0, 0, 0, 0, 0, 0};
static int variable_id[6] = {LOG_RANGER_FRONT_ID, LOG_RANGER_BACK_ID, LOG_RANGER_LEFT_ID, LOG_RANGER_RIGHT_ID, LOG_RANGER_UP_ID, LOG_RANGER_ZRANGE_ID};
static int block_id = 0;
static uint16_t isCloseThreshold = 200; // mm

//=========================== prototypes ======================================

void log_createLogBlock_v2_uint16(int block_id, struct ops_setting_v2 *ops, uint8_t ops_len);
void log_startLogBlock(int id, uint8_t period);

//=========================== public ==========================================

void mutiranger_init()
{
    //TODO: Check mutiranger deck is connected

    // create log block
    struct ops_setting_v2 ops[6];
    for (int i = 0; i < 6; i++)
    {
        ops[i].logType = LOG_UINT16;
        ops[i].id = variable_id[i];
    }
    log_createLogBlock_v2_uint16(block_id, ops, 6);

    // start log block
    log_startLogBlock(block_id, 10);  //the period will be multiplied by 10(in stm32), so 100 means 1000ms
}

void mutiranger_handle(uint8_t *data, uint8_t len)
{

    if (len < 4 + sizeof(mutiranger_values))
    {
        return;
    }

    uint8_t id = data[0];
    uint32_t timestamp = data[1] | data[2] << 8 | data[3] << 16;
    uint8_t *log_data = data + 4;

    memcpy(&mutiranger_values, log_data, sizeof(mutiranger_values));
}

// Getter
uint16_t mutiranger_get_front_mm()
{
    return mutiranger_values.front;
}

uint16_t mutiranger_get_back_mm()
{
    return mutiranger_values.back;
}

uint16_t mutiranger_get_left_mm()
{
    return mutiranger_values.left;
}

uint16_t mutiranger_get_right_mm()
{
    return mutiranger_values.right;
}

uint16_t mutiranger_get_up_mm()
{
    return mutiranger_values.up;
}

uint16_t mutiranger_get_down_mm()
{
    return mutiranger_values.down;
}

bool mutiranger_front_isClose()
{
    return mutiranger_values.front < isCloseThreshold;
}

bool mutiranger_back_isClose()
{
    return mutiranger_values.back < isCloseThreshold;
}

bool mutiranger_left_isClose()
{
    return mutiranger_values.left < isCloseThreshold;
}

bool mutiranger_right_isClose()
{
    return mutiranger_values.right < isCloseThreshold;
}

bool mutiranger_up_isClose()
{
    return mutiranger_values.up < isCloseThreshold;
}

bool mutiranger_down_isClose()
{
    return mutiranger_values.down < isCloseThreshold;
}

// Setter
void mutiranger_set_close_threshold(uint16_t threshold_mm)
{   
    //0-4m
    if (threshold_mm > 4000)
    {
        threshold_mm = 4000;
    }

    isCloseThreshold = threshold_mm;
}




//=========================== private =========================================

// LOG Create_block V2
void log_createLogBlock_v2_uint16(int block_id, struct ops_setting_v2 *ops, uint8_t ops_len)
{
    uint8_t data[CRTP_MAX_DATA_SIZE];
    uint8_t command = CONTROL_CREATE_BLOCK_V2;

    // args
    data[0] = command;  // cmd
    data[1] = block_id; // block id uint8_t

    for (int i = 0; i < ops_len; i++)
    {
        // each log item (variable)
        uint8_t index = 2 + i * 3; // TODO: this func only support uint16_t, so 3 bytes for each item

        data[index] = ops[i].logType;
        data[index + 1] = ops[i].id & 0xFF;
        data[index + 2] = (ops[i].id >> 8) & 0xFF;
    }

    CTRPSend(data, 2 + ops_len * 3, CRTP_PORT_LOG, CHAN_SETTINGS);
}

// LOG Start_block
void log_startLogBlock(int block_id, uint8_t period)
{
    uint8_t data[CRTP_MAX_DATA_SIZE];
    uint8_t command = CONTROL_START_BLOCK; // do not support v2 start now (CF21 Aug 2024)

    // args
    data[0] = command;  // cmd
    data[1] = block_id; // block id uint8_t
    data[2] = period;   // period

    CTRPSend(data, 3, CRTP_PORT_LOG, CHAN_SETTINGS);
}

//=========================== interrup handlers ===============================