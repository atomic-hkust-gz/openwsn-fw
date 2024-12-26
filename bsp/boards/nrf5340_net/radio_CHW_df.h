#ifndef __RADIO_CHW_DF_H
#define __RADIO_CHW_DF_H

#include "board.h"

/**
This file is driver of using CHW antenna board for AoA/AoD 
*/

//=========================== define ==========================================

// set this value according to the direction finding configurations
// e.g. if 
#define SAMPLE_MAXCNT       (0x58)

//=========================== typedef =========================================


//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void     radio_configure_direction_finding_CHW_antenna_switch(uint8_t antenna_array_id);
void     radio_configure_direction_finding_CHW_manual(void);
//void     radio_configure_direction_finding_CHW_inline(void);
//void     radio_configure_switch_CHW_antenna_array(void);
//uint8_t  radio_get_antenna_array_id(void);
uint16_t radio_get_df_samples(uint32_t* sample_buffer, uint16_t length);
void     radio_get_crc(uint8_t* crc24);
// return in MHz
uint32_t radio_get_frequency(void);

void antenna_CHW_rx_switch_init(void);
void antenna_CHW_tx_switch_init(void);
void set_antenna_CHW_switches(void);
/**
\}
\}
*/

#endif
