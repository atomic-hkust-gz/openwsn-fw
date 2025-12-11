#ifndef __RADIO_TI_DF_H
#define __RADIO_TI_DF_H

#include "board.h"

/**
This file is driver of using TI antenna board for AoA/AoD 
*/

//=========================== define ==========================================

// set this value according to the direction finding configurations
// e.g. if 
#define SAMPLE_MAXCNT       (0x5c)  //0x5c for scum       normal 0x58


//=========================== typedef =========================================


//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void     radio_configure_direction_finding_TI_antenna_switch(uint8_t antenna_array_id);

uint16_t radio_get_df_samples(uint32_t* sample_buffer, uint16_t length);
void     radio_get_crc(uint8_t* crc24);
// return in MHz
uint32_t radio_get_frequency(void);

void antenna_TI_rx_switch_init(void);
void antenna_TI_tx_switch_init(void);

#endif