#ifndef RADIO_WRAPPER_H
#define RADIO_WRAPPER_H

#include "config.h" 

#if (RADIO_MODE == 1)
 /**
 * \def RADIO_MODE

 * Mode 1: LoRa
 */
 #include "LoRa.h"
    


#else
 /**
 * \def RADIO_MODE

 * Mode 0: IEEE802154E
 */
  #include "IEEE802154E.h"
    
#endif

#endif // RADIO_WRAPPER_H