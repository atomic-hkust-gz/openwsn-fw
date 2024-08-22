/**
 *
 * @file cf_movement_queue.c
 * @brief The movement queue for Crazyflie 2.X.
 * @author Lan HUANG (YelloooBlue@outlook.com)
 * @date Aug 2024
 *
 */

#include <stdint.h>
#include "opendefs.h"

// crazyflie bsp
#include "cf_movement_queue.h"
#include "cf_api_commander_high_level.h"

//=========================== defines =========================================

#define DRONE_VEL 0.5 // Crazyflie high level move velocity (m/s)

typedef struct
{
  int drone_id;  // Drone ID
  int drone_num; // Drone number

  uint32_t movement_start_asn; // the ASN when the movement starts

  int current_step_index;          // current step index
  uint32_t current_step_start_ASN; // current step start ASN
  uint32_t current_step_end_ASN;   // current step end ASN

  struct simple_trajectory *trajectory; // trajectory
} Drone;

// === Trajectory ===

//Closed octagon
// #define NUM_STEPS 8
// #define TOTAL_DURATION_ASN 400
// struct simple_trajectory_point test_points[NUM_STEPS] = {
//    {0.75, 0.25, 0.5, 0, 1.0, 50},
//    {0.75, -0.25, 0.5, 0, 1.0, 50},

//    {0.25, -0.75, 0.5, 0, 1.0, 50},
//    {-0.25, -0.75, 0.5, 0, 1.0, 50},

//    {-0.75, -0.25, 0.5, 0, 1.0, 50},
//    {-0.75, 0.25, 0.5, 0, 1.0, 50},

//    {-0.25, 0.75, 0.5, 0, 1.0, 50},
//    {0.25, 0.75, 0.5, 0, 1.0, 50},
// };
// struct simple_trajectory test_trajectory = {test_points, NUM_STEPS, TOTAL_DURATION_ASN, true, false};

// Line
// #define NUM_STEPS 2
// #define TOTAL_DURATION_ASN 100
// struct simple_trajectory_point test_points[NUM_STEPS] = {
//     {0.5, 0.0, 0.0, 0, 1.0, 50},
//     {-0.5, -0.0, 0.0, 0, 1.0, 50},
// };
// struct simple_trajectory test_trajectory = {test_points, NUM_STEPS, TOTAL_DURATION_ASN, true, true};

// // Rectangle
// #define NUM_STEPS 4
// #define TOTAL_DURATION_ASN 200
// struct simple_trajectory_point test_points[NUM_STEPS] = {
//    {0.5, 0.0, 0.0, 0, 1.0, 50},
//    {0.0, 0.5, 0.0, 0, 1.0, 50},
//    {-0.5, 0.0, 0.0, 0, 1.0, 50},
//    {0.0, -0.5, 0.0, 0, 1.0, 50},
// };
// struct simple_trajectory test_trajectory = {test_points, NUM_STEPS, TOTAL_DURATION_ASN, true, true};

// Absolute Rectangle
// #define NUM_STEPS 4
// #define TOTAL_DURATION_ASN 200
// struct simple_trajectory_point test_points[NUM_STEPS] = {
//     {0.25, 0.25, 0.5, 0, 1.0, 50},
//     {-0.25, 0.25, 0.5, 0, 1.0, 50},
//     {-0.25, -0.25, 0.5, 0, 1.0, 50},
//     {0.25, -0.25, 0.5, 0, 1.0, 50},
// };
// struct simple_trajectory test_trajectory = {test_points, NUM_STEPS, TOTAL_DURATION_ASN, true, false};

// Absolute 3D Rectangle
#define NUM_STEPS 4
#define TOTAL_DURATION_ASN 200
struct simple_trajectory_point test_points[NUM_STEPS] = {
    {0.25, 0.25, 1.0, 0, 1.0, 50},
    {-0.25, 0.25, 0.5, 0, 1.0, 50},
    {-0.25, -0.25, 1.0, 0, 1.0, 50},
    {0.25, -0.25, 0.5, 0, 1.0, 50},
};
struct simple_trajectory test_trajectory = {test_points, NUM_STEPS, TOTAL_DURATION_ASN, true, false};

//=========================== variables =======================================

uint16_t slotDuration = 20; // ms
Drone Drone1;
uint32_t last_asn = 0;

//=========================== prototypes ======================================

int _execute_current_step(Drone *drone, uint32_t now_asn);
int _execute_next_step(Drone *drone, uint32_t now_asn);
int _roll_back_step_by_asn_diff(int32_t asn_diff, Drone *drone);
int _roll_forward_step_by_asn_diff(int32_t asn_diff, Drone *drone);
uint32_t _asn_to_uint32(asn_t *asn);
int _calculate_index_offset(int drone_id, int num_drones, int num_points);

//=========================== public ==========================================

void cf_movement_queue_init()
{
  Drone1.drone_id = 2;
  Drone1.drone_num = 4;
  Drone1.trajectory = &test_trajectory;
  Drone1.current_step_index = _calculate_index_offset(Drone1.drone_id, Drone1.drone_num, Drone1.trajectory->num_steps);
  Drone1.current_step_start_ASN = 0;
  Drone1.current_step_end_ASN = Drone1.trajectory->points[0].duration_asn;

  Drone1.movement_start_asn = 2100;
}

void cf_movement_queue_handle(asn_t *now_asn)
{
  uint32_t asn = _asn_to_uint32(now_asn);

  if (asn <= Drone1.movement_start_asn)
  {
    return;
  }

  asn -= Drone1.movement_start_asn;
  int32_t diff = asn - last_asn;

  /**
   *
   * diff = 1: normal case
   * diff < 1: roll back (abandon the missed steps)
   * diff > 1: roll forward (catch up the missed steps?)
   */

  if (diff == 1 || last_asn == 0)
  {
    if (asn >= Drone1.current_step_end_ASN)
    {
      _execute_current_step(&Drone1, asn);
    }
  }
  else if (diff < 1)
  {
    // Roll back
    int new_step_offset = _roll_back_step_by_asn_diff(diff, &Drone1); //TODO: should be tested
    _execute_current_step(&Drone1, asn);
  }
  else
  {
    // Roll forward
    int new_step_offset = _roll_forward_step_by_asn_diff(diff, &Drone1); //TODO: should be tested
    _execute_current_step(&Drone1, asn);
  }

  last_asn = asn;
}


// Getters and setters
void set_slot_duration(uint16_t duration)
{
  slotDuration = duration;
}

// void set_drone_id(int id)
// {
//   drone_id = id;
// }

// void set_start_asn(asn_t asn)
// {
//   start_asn = asn;
// }

//=========================== private =========================================

int _execute_current_step(Drone *drone, uint32_t now_asn)
{
  // GoTo
  struct simple_trajectory_point *point = &drone->trajectory->points[drone->current_step_index];
  high_level_goto(point->x, point->y, point->z, point->yaw, point->duration, drone->trajectory->is_relative);

  // Pop next step
  drone->current_step_index = (drone->current_step_index + 1) % drone->trajectory->num_steps;
  drone->current_step_start_ASN = now_asn + 1;
  uint32_t current_step_duration_asn = drone->trajectory->points[drone->current_step_index].duration_asn;
  drone->current_step_end_ASN = drone->current_step_start_ASN + current_step_duration_asn;

  return 0;
}

int _execute_next_step(Drone *drone, uint32_t now_asn)
{
  // Pop next step
  drone->current_step_index = (drone->current_step_index + 1) % drone->trajectory->num_steps;
  drone->current_step_start_ASN = now_asn;
  uint32_t current_step_duration_asn = drone->trajectory->points[drone->current_step_index].duration_asn;
  drone->current_step_end_ASN = drone->current_step_start_ASN + current_step_duration_asn;

  // GoTo
  struct simple_trajectory_point *point = &drone->trajectory->points[drone->current_step_index];
  high_level_goto(point->x, point->y, point->z, point->yaw, point->duration, drone->trajectory->is_relative);

  return 0;
}

int _roll_back_step_by_asn_diff(int32_t asn_diff, Drone *drone)
{
  struct simple_trajectory *trajectory = drone->trajectory;

  int new_step_offset = 0;
  int total_duration_asn = trajectory->total_duration_asn;

  // Convert the roll back ASN diff to an offset within a period
  int roll_back_relative_asn = asn_diff % total_duration_asn; // This value will be within the interval [0, total_duration_asn)

  while (roll_back_relative_asn < 0)
  {
    drone->current_step_index = (drone->current_step_index - 1 + drone->trajectory->num_steps) % drone->trajectory->num_steps;

    uint32_t current_step_duration_asn = trajectory->points[drone->current_step_index].duration_asn;
    drone->current_step_start_ASN -= current_step_duration_asn;

    roll_back_relative_asn += current_step_duration_asn; // Compensate for the ASN difference
    new_step_offset--;
  }

  return new_step_offset;
}

int _roll_forward_step_by_asn_diff(int32_t asn_diff, Drone *drone)
{
  struct simple_trajectory *trajectory = drone->trajectory;

  int new_step_offset = 0;
  int total_duration_asn = trajectory->total_duration_asn;

  // Convert the roll forward ASN diff to an offset within a period
  int roll_forward_relative_asn = asn_diff % total_duration_asn; // This value will be within the interval [0, total_duration_asn)

  while (roll_forward_relative_asn > 0)
  {
    drone->current_step_index = (drone->current_step_index + 1) % drone->trajectory->num_steps;

    uint32_t current_step_duration_asn = trajectory->points[drone->current_step_index].duration_asn;
    drone->current_step_start_ASN += current_step_duration_asn;

    roll_forward_relative_asn -= current_step_duration_asn; // Compensate for the ASN difference
    new_step_offset++;

    // TODO: execute the missed steps?
  }

  return new_step_offset;
}

int _calculate_index_offset(int drone_id, int num_drones, int num_points)
{
  return drone_id * num_points / num_drones;
}

uint32_t _asn_to_uint32(asn_t *asn)
{
  uint32_t test = (uint32_t)asn->bytes2and3 << 16 | (uint32_t)asn->bytes0and1;
  return test;
}


// int32_t _calculate_asn_diff_int32(asn_t *asn1, asn_t *asn2)
// {
//   if (asn1->byte4 != asn2->byte4)
//   {
//     return 0x7FFFFFFF; // max int32
//   }

//   int32_t diff = 0;

//   if (asn1->bytes2and3 == asn2->bytes2and3)
//   {
//     diff = (int32_t)asn1->bytes0and1 - (int32_t)asn2->bytes0and1;
//   }
//   else
//   {
//     diff = ((int32_t)asn1->bytes2and3 - (int32_t)asn2->bytes2and3) << 16;
//     diff += (int32_t)asn1->bytes0and1 - (int32_t)asn2->bytes0and1;
//   }

//   return diff;
// }

// float _asn_to_sec_int32(int32_t asn_diff)
// {
//   return (float)asn_diff * slotDuration / 1000.0;
// }

// int32_t _sec_to_asn_int32(float sec)
// {
//   return (int32_t)(sec * 1000.0 / slotDuration);
// }

//=========================== interrup handlers ===============================