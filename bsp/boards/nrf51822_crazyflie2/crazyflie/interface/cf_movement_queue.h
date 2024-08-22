/**
 *
 * @file cf_movement_queue.c
 * @brief The movement queue for Crazyflie 2.X.
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date Aug 2024
 *
 */

#ifndef __CF_MOVEMENT_QUEUE__
#define __CF_MOVEMENT_QUEUE__

#include <stdint.h>
#include <stdbool.h>
#include "opendefs.h"

//=========================== define ==========================================

struct simple_trajectory_point
{
    float x;
    float y;
    float z;
    float yaw;
    float duration;
    uint32_t duration_asn; // TODO: calculate duration_asn from duration(s)
} __attribute__((packed));

struct simple_trajectory
{
    struct simple_trajectory_point *points;
    int num_steps;
    uint32_t total_duration_asn;
    bool is_closed; // TODO: support open trajectory
    bool is_relative;
} __attribute__((packed));

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void cf_movement_queue_init();
void cf_movement_queue_handle(asn_t *now_asn);

#endif // __CF_MOVEMENT_QUEUE__
