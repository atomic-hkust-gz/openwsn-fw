#ifndef __CCRAZYFLIE_H
#define __CCRAZYFLIE_H

/**
\addtogroup AppCoAP
\{
\addtogroup crazyflie
\{
*/

#include "config.h"
#include "coap.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    coap_resource_desc_t desc;
} ccrazyflie_vars_t;

//=========================== prototypes ======================================

void ccrazyflie_init(void);

/**
\}
\}
*/

#endif
