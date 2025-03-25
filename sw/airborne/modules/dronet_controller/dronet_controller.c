/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/dronet_controller/dronet_controller.h"
#include "modules/computer_vision/dronet_image_filter.h"
#include "modules/computer_vision/dronet_floor_detector.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
// #include "dronet.h"

// Verbose settings
#define DRONET_CONTROLLER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[dronet_controller->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if DRONET_CONTROLLER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Define functions used
static uint8_t heading_from_steering(float steering_input);
static uint8_t velocity_from_collision_prob(float collision_prob);
static uint8_t chooseRandomIncrementAvoidance(void);

// Define maximum horizontal speed of the drone from airframe configuration
#ifndef V_MAX
#define V_MAX GUIDANCE_H_REF_MAX_SPEED
#endif

// DroNet parameters
#define ALPHA 0.7f
#define BETA 0.5f

// Initialize global variables
float s_k = 0.f;                        // Steering input
float p = 1.f;                          // Probability of collision
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// Define settings
float oag_floor_count_frac = 0.03f;       // floor detection threshold as a fraction of total of image
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// Define navigation states
enum navigation_state_t {
  SAFE,         // Default mode: use neural net output
  COLLISION_AVOID,    // Stop or reverse to avoid collision
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,       // Reorient and push drone back in bounds
  REENTER_ARENA
};

enum navigation_state_t nav_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine

#ifndef DRONET_CONTROLLER_VISUAL_DETECTION_ID
// #define DRONET_CONTROLLER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event dronet_image_ev;

// Callback function
static void dronet_image_cb(uint8_t __attribute__((unused)) sender_id, float steering_input, float collision_prob)
{
  s_k = steering_input;
  p = collision_prob;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
// #define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

// Initialization function
void dronet_controller_init(void) {

  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // Bind the ABI message to receive image data
  // Note: DRONET_CONTROLLER_VISUAL_DETECTION_ID corresponds to NN_OBJECT_DETECTION_ID
  AbiBindMsgNN_DETECTION(DRONET_CONTROLLER_VISUAL_DETECTION_ID, &dronet_image_ev, dronet_image_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

void dronet_controller_periodic(void) {

  // Ensure we're in GUIDED mode (required for guided control to apply)
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    // Set the navigation state into the default control mode
    enum navigation_state_t nav_state = SEARCH_FOR_SAFE_HEADING;
    VERBOSE_PRINT("Not in GUIDED mode. Controller inactive.\n");
    return;
  }

  if (s_k == 0 && p == 0) {
    VERBOSE_PRINT("No valid inference results available.\n");
    return;
  }

  // Compute current color thresholds
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // Update our safe confidence using collision probability
  if (p < 0.5f) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // more cautious if danger detected
  }

  // Bound the value between 0 and max
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);


  switch (nav_state) {
    case SAFE:

      VERBOSE_PRINT("State: SAFE.\n");

      // Check if drone is out of bounds of the obstacle zone
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        nav_state = OUT_OF_BOUNDS;
      }
      // Check if the predicted probability of collision is too high
      else if (obstacle_free_confidence == 0){
        nav_state = COLLISION_AVOID;
      }
      // If safe navigate the drone
      else {
        // heading_from_steering(s_k);
        velocity_from_collision_prob(p);

        VERBOSE_PRINT("Periodic - Steering: %.2f, Collision: %.2f\n", s_k, p);
      }
      break;

    case COLLISION_AVOID:

      VERBOSE_PRINT("State: COLLISION_AVOID.\n");

      // Emergency stop
      guidance_h_set_body_vel(0.0f, 0.0f);
      VERBOSE_PRINT("EMERGENCY STOP: collision_prob = %.2f\n", p);

      // Randomly select new search direction
      chooseRandomIncrementAvoidance();

      // Search for safe heading mode
      nav_state = SEARCH_FOR_SAFE_HEADING;

      break;

    case SEARCH_FOR_SAFE_HEADING:

      VERBOSE_PRINT("State: SEARCH_FOR_SAFE_HEADING.\n");

      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // Ensure the probability of collision is low enough before declaring the way safe
      if (obstacle_free_confidence >= 2){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        nav_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:

      VERBOSE_PRINT("State: OUT_OF_BOUNDS.\n");

      // Emergency stop
      guidance_h_set_body_vel(0.0f, 0.0f);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      nav_state = REENTER_ARENA;

      break;

    case REENTER_ARENA:

      VERBOSE_PRINT("State: REENTER_ARENA.\n");

      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold  && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        nav_state = SAFE;
      }
      break;
    default:
      break;
  }
}
  
/*
 * Updates the NAV heading based on a scaled steering input in [-1, 1]
 * using a low-pass filter to smooth the heading changes.
 */
uint8_t heading_from_steering(float steering_input)  
{
  // clamp steering input to [-1, 1]
  steering_input = fmaxf(fminf(steering_input, 1.0f), -1.0f);
                
  // Compute new filtered heading based on scaled steering input
  float new_heading = (1.0f - BETA) * stateGetNedToBodyEulers_f()->psi + BETA * ((M_PI / 2.0f) * steering_input);

  // Normalize to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // Set heading
  guidance_h_set_heading(new_heading);

  VERBOSE_PRINT("Updated heading (rad): %f, (deg): %f\n", new_heading, DegOfRad(new_heading));
  return false;
}

/*
 * Updates the NAV velocity in x and y direction based on the probability of collision with an obstacle
 * using a low-pass filter to smooth the velocity changes.
 */
uint8_t velocity_from_collision_prob(float collision_prob)
{
  // Clamp collision probability to [0, 1]
  collision_prob = fmaxf(fminf(collision_prob, 1.0f), 0.0f);

  // Compute current forward velocity
  struct EnuCoor_f *vel = stateGetSpeedEnu_f();

  float vx = vel->x;
  float vy = vel->y;
  // float vz = vel->z;

  float current_forward_velocity = sqrtf(vx * vx + vy * vy);

  // Compute low-pass filtered velocity
  float target_velocity = (1.0f - ALPHA) * current_forward_velocity + ALPHA * (1.0f - collision_prob) * V_MAX;

  // Move forward (vx), no lateral (vy)
  guidance_h_set_body_vel(target_velocity, 0.0f); 

  VERBOSE_PRINT("Updated forward velocity: %.2f m/s (collision_prob=%.2f)\n", target_velocity, collision_prob);
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}
