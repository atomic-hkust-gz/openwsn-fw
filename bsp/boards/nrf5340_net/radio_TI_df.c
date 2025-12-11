#include "nRF5340_network.h"
#include "nrf5340_network_bitfields.h"
#include "board.h"
#include "radio.h"
#include "debugpins.h"
#include "leds.h"
#include "radio_df.h"


/**
This file is driver of using TI antenna board for AoA/AoD 

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Sept 2025.
*/

//=========================== define ==========================================


#define SAMPLE_MAXCNT       (0x5c)    //0x58 == 1 us 1 sample for 88 us    0x2c0 == 1 us 8 samples for 88us       0x5c for scum
#define MAX_IQSAMPLES            0x5c //((1<<8)-1)

#define MAX_PACKET_SIZE           (255)       ///< maximal size of radio packet (one more byte at the beginning needed to store the length)



// TI BOOSTXL-AOA antenna pin mapping

//  VALUE   DIO27   DIO28   DIO29   DIO30   Antenna
//  0x2     0       1       0       0       A2.1
//  0x4     0       0       1       0       A2.2
//  0x8     0       0       0       1       A2.3
//  0x3     1       1       0       0       A1.1
//  0x5     1       0       1       0       A1.2
//  0x9     1       0       0       1       A1.3

//  0: 0.0V to 0.2V, 1: 2.5V to 5.0V


#define ANT_SWITCH_PORT           1
#define ANT_SWITCH_PIN0           6 // DIO27
#define ANT_SWITCH_PIN1           7 // DIO28
#define ANT_SWITCH_PIN2           8 // DIO29
#define ANT_SWITCH_PIN3           9 // DIO30

#define PATTERN_A2_1              0x2
#define PATTERN_A2_2              0x4
#define PATTERN_A2_3              0x8
#define PATTERN_A1_1              0x3
#define PATTERN_A1_2              0x5
#define PATTERN_A1_3              0x9


//===========================private===========================================
void nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);



//===========================public============================================
void radio_configure_direction_finding_TI_antenna_switch(int8_t array_id) {
    
    uint8_t i;

    NRF_RADIO_NS->EVENTS_DISABLED = (uint32_t)0;
    NRF_RADIO_NS->TASKS_DISABLE = (uint32_t)1;
    while(NRF_RADIO_NS->EVENTS_DISABLED == 0);

    nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN0);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN1);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN2);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN3);

    NRF_P1_NS->OUTCLR = 0x000003C0;
        
    // configure GPIO pins
    NRF_RADIO_NS->PSEL.DFEGPIO[0] = (uint32_t)(
                                        (ANT_SWITCH_PORT << 5)      |
                                        (ANT_SWITCH_PIN0 << 0)      |
                                        (0 << 31)
                                    );
    NRF_RADIO_NS->PSEL.DFEGPIO[1] = (uint32_t)(
                                        (ANT_SWITCH_PORT << 5)      |
                                        (ANT_SWITCH_PIN1 << 0)      |
                                        (0 << 31)
                                    );
    NRF_RADIO_NS->PSEL.DFEGPIO[2] = (uint32_t)(
                                        (ANT_SWITCH_PORT << 5)      |
                                        (ANT_SWITCH_PIN2 << 0)      |
                                        (0 << 31)
                                    );
    NRF_RADIO_NS->PSEL.DFEGPIO[3] = (uint32_t)(
                                        (ANT_SWITCH_PORT << 5)      |
                                        (ANT_SWITCH_PIN3 << 0)      |
                                        (0 << 31)
                                    );

    // write switch pattern

    NRF_RADIO_NS->CLEARPATTERN  = (uint32_t)1;
    if (array_id == 2){
        // use antenna array 2
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A2_2);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A2_2);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A2_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A2_3);
    } else {
        // use antenna array 1 by default
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_2);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_3);
    }
}