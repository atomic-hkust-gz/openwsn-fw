# Running OpenWSN on nRF51822 on Crazyflie2.1

## Info
- Author: lan HUANG (YelloooBlue@Outlook.com)
- Date: Apr 2024

## Introduction
The goal of this project is to run OpenWSN on Crazyflie2.1.
modifying the code of nRF51822 on the Crazyflie2.1 board.
https://www.bitcraze.io/documentation/system/platform/cf2-architecture/


## Dependency
- MBS (Master Boot Switch)
https://github.com/bitcraze/crazyflie2-nrf-firmware `/bootloaders/nrf_mbs_cf21.hex`
- BootLoader
build https://github.com/bitcraze/crazyflie2-nrf-bootloader
or https://github.com/bitcraze/crazyflie2-nrf-firmware `/bootloaders/cload_nrf_cf21.hex`

- SoftDevice S130 
from nRF5 SDK v12.3.0 `/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex`


## Flashing

| Data | Flash Address |
| ------------ | ------------ |
| MBS | 0x0003F000 (252k) |
| BootLoader | 0x0003A000 (232k) |
| **Firmware(This App) ** | 0x0001B000 (108k) |
| SoftDevice & MBR | 0x00000000 |

https://www.bitcraze.io/documentation/repository/crazyflie2-nrf-firmware/master/development/architecture/


