
nRF51-DK (pca10028) nRF51422 Minimized SES Project

>Starting with v14.1.0, the nRF5 SDK supplies SEGGER Embedded Studio projects. If you are using an older version of the nRF5 SDK (for example, nRF5 SDK v12.3.0, which supports nRF51 Series devices), you must import and convert the Keil µVision projects.

This means that nRF51 does not have a segger template!

This project is being updated

## TODO：


Author Lan HUANG (YelloooBlue@Outlook.com)
Date Apr 2024

- SConscript
- UART
- Radio
- ScTimer
- ...


<!-- # Running OpenWSN on nRF52840

The nRF52840 port of OpenWSN uses SEGGER Embedded Studio as its IDE.
The current version doesn't support the L2_security yet.
To run an OpenWSN network, cjoin needs to be disabled by commenting out the 

        void openapps_init(void) {

           ...
           ...
           
           // cjoin_init();

           ...
           ...
        }
        
in [openapp.c](https://github.com/openwsn-berkeley/openwsn-fw/blob/develop/openapps/openapps.c#L40) file. -->