#include "nRF5340_network.h"
#include "nrf5340_network_bitfields.h"
#include "board.h"
#include "radio.h"
#include "debugpins.h"
#include "leds.h"
#include "radio_df.h"


/**
This file is driver of using CHW antenna board for AoA/AoD 
*/

//=========================== define ==========================================


#define SAMPLE_MAXCNT       (0x58)    //0x58 == 1 us 1 sample for 88 us    0x2c0 == 1 us 8 samples for 88us
#define MAX_IQSAMPLES            0x58 //((1<<8)-1)

#define MAX_PACKET_SIZE           (255)       ///< maximal size of radio packet (one more byte at the beginning needed to store the length)

// CHW antenna pin mapping

//have problems in mapping 

//  VALUE   Pin8    Pin7    Pin6    Pin5    Antenna
//  0x0     0       0       0       0       ANT3.2
//  0x1     0       0       0       1       ANT3.1
//  0x2     0       0       1       0       ANT4.3
//  0x3     0       0       1       1       ANT3.3
//  0x4     0       1       0       0       ANT4.1
//  0x5     0       1       0       1       ANT4.2
//  0x6     0       1       1       0       ANT4.4
//  0x7     0       1       1       1       ANT3.4
//  0x8     1       0       0       0       ANT2.4
//  0x9     1       0       0       1       ANT1.4
//  0xa     1       0       1       0       ANT1.2
//  0xb     1       0       1       1       ANT1.1
//  0xc     1       1       0       0       ANT2.3
//  0xd     1       1       0       1       ANT1.3
//  0xe     1       1       1       0       ANT2.1
//  0xf     1       1       1       1       ANT2.2

//  The antenna in the upper left corner is numbered ANT1.1
//  The antenna in the lower right corner is numbered ANT4.4
//  The first number of the antenna number represents the row and the second number represents the column
//  0: 0.0V to 0.2V, 1: 2.5V to 5.0V

#define ANT_SWITCH_PORT           1
#define ANT_SWITCH_PIN0           6 // Pin5
#define ANT_SWITCH_PIN2           7 // Pin6
#define ANT_SWITCH_PIN1           8 // Pin7
#define ANT_SWITCH_PIN3           9 // Pin8

#define ANT_DIGITAL_CONTROL_INPUT_PIN   1   //Pin9
#define ANT_ENABLE_PIN            4         //Pin10


#define PATTERN_A3_2              0x0
#define PATTERN_A3_1              0x1
#define PATTERN_A4_1              0x2
#define PATTERN_A4_2              0x3
#define PATTERN_A4_3              0x4
#define PATTERN_A3_3              0x5
#define PATTERN_A4_4              0x6
#define PATTERN_A3_4              0x7
#define PATTERN_A2_4              0x8
#define PATTERN_A1_4              0x9
#define PATTERN_A2_3              0xa
#define PATTERN_A1_3              0xb
#define PATTERN_A1_2              0xc
#define PATTERN_A1_1              0xd
#define PATTERN_A2_1              0xe
#define PATTERN_A2_2              0xf

#define LED_PORT                  0

//===========================private===========================================
void nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
//===========================public============================================

void antenna_CHW_rx_switch_init(void) {
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN0);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN1);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN2);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN3);

    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_ENABLE_PIN);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_DIGITAL_CONTROL_INPUT_PIN);

    NRF_P1_NS->OUTSET =  1 << ANT_ENABLE_PIN;
    NRF_P1_NS->OUTCLR =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;
    //NRF_P1_NS->OUTSET =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;

}

void antenna_CHW_tx_switch_init(void) {
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN0);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN1);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN2);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_SWITCH_PIN3);

    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_ENABLE_PIN);
    nrf_gpio_cfg_output(ANT_SWITCH_PORT, ANT_DIGITAL_CONTROL_INPUT_PIN);

    NRF_P1_NS->OUTSET =  1 << ANT_ENABLE_PIN;
    //NRF_P1_NS->OUTCLR =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;
    NRF_P1_NS->OUTSET =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;

}

void set_antenna_CHW_switches(void) {
    
    NRF_P1_NS->OUTCLR =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;
    uint8_t value;
    value = 0xe;
    if (value & 0x01) {
        NRF_P1_NS->OUTSET =  1 << ANT_SWITCH_PIN0;
    } else {
        NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN0;
    }

    if (value & 0x02) {
        NRF_P1_NS->OUTSET =  1 << ANT_SWITCH_PIN1;
    } else {
        NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN1;
    }

    if (value & 0x04) {
        NRF_P1_NS->OUTSET =  1 << ANT_SWITCH_PIN2;
    } else {
        NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN2;
    }

    if (value & 0x08) {
        NRF_P1_NS->OUTSET =  1 << ANT_SWITCH_PIN3;
    } else {
        NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN3;
    }
    
    //NRF_P1_NS->OUTSET =  1 << ANT_SWITCH_PIN0;
    //NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN1;
    //NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN2;
    //NRF_P1_NS->OUTCLR =  1 << ANT_SWITCH_PIN3;

    //NRF_P1_NS->OUTSET =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;
    //NRF_P1_NS->OUTCLR =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;
    //NRF_P1_NS->OUTSET =  1 << ANT_DIGITAL_CONTROL_INPUT_PIN;
}


void radio_configure_direction_finding_CHW_antenna_switch(uint8_t antenna_array_id) {
    
    uint8_t i;

    NRF_RADIO_NS->EVENTS_DISABLED = (uint32_t)0;
    NRF_RADIO_NS->TASKS_DISABLE = (uint32_t)1;
    while(NRF_RADIO_NS->EVENTS_DISABLED == 0);

    //nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN0);
    //nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN1);
    //nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN2);
    //nrf_gpio_cfg_output(ANT_SWITCH_PORT,  ANT_SWITCH_PIN3);

    //NRF_P1_NS->OUTCLR = 0x000003C0;
        
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
    
    // set radio switch pattern
    if (antenna_array_id == 1) {
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_1);

        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_2);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_3);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_4);
    
    } else if (antenna_array_id == 2) {
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_1);

        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A3_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A2_1);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_1);
    } else if (antenna_array_id == 3) {
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_2);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_2);

        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_2);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_3);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_4);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_1);
    } else if (antenna_array_id == 4) {
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A3_4);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A3_4);

        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A3_4);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A2_4);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A1_4);
        NRF_RADIO_NS->SWITCHPATTERN = (uint32_t)(PATTERN_A4_4);
    }
    
}

