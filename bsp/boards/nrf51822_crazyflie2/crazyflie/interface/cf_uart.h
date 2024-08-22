/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

/**
 * @file cf_uart.h
 * @brief UART interface for the NRF51822 on the Crazyflie 2.1
 * @author Lan HUANG(YelloooBlue@outlook.com)
 * @date June 2024
 *
 * This interface is different from the one in the OPENWSN project. 
 * It is specific to the Crazyflie 2.1 for the NRF51822 communicate with the STM32F4.
 * This file contains the UART driver for the NRF51822. It is used to send and
 * receive data over the UART interface.
 */


#ifndef __CF_UART_H__
#define __CF_UART_H__

#include <stdbool.h>

void uartInit();

void uartDeinit();

void uartPuts(char* string);

void uartSend(char* data, int len);

void uartPutc(char c);

bool uartIsDataReceived();

char uartGetc();

int uartDropped();
uint8_t uartGetError();
uint8_t uartGetErrorCount();

#endif //__CF_UART_H__
