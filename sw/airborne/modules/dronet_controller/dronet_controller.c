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
#include "firmwares/rotorcraft/navigation.h"
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
#include "dronet.h"

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
float s_k = 0.f;          // Steering input
float p = 1.f;            // Probability of collision

// Define navigation states
enum navigation_state_t {
  NN_CONTROL,         // Default mode: use neural net output
  COLLISION_AVOID,    // Stop or reverse to avoid collision
  OUT_OF_BOUNDS       // Reorient and push drone back in bounds
};

// Set the navigation state into the default control mode
enum navigation_state_t nav_state = NN_CONTROL;

#ifndef DRONET_CONTROLLER_VISUAL_DETECTION_ID
#define DRONET_CONTROLLER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event dronet_image_ev;

// Callback function
static void dronet_image_cb(float steering_input, float collision_prob)
{
  s_k = steering_input;
  p = collision_prob;
}

// Initialization function
void dronet_controller_init(void) {

  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // Bind the ABI message to receive image data
  // Note: DRONET_CONTROLLER_VISUAL_DETECTION_ID corresponds to NN_OBJECT_DETECTION_ID
  AbiBindMsgVISUAL_DETECTION(DRONET_CONTROLLER_VISUAL_DETECTION_ID, &dronet_image_ev, dronet_image_cb);
}

void dronet_controller_periodic(void) {

  // Don’t navigate if not flying
  if (!autopilot_in_flight()) {
      return;
  }

  if (s_k == 0 && p == 0) {
    VERBOSE_PRINT("No valid inference results available.\n");
    return;
  }

  switch (nav_state) {
    case NN_CONTROL:
      // Check if drone is out of bounds of the obstacle zone
      if (!InsideObstacleZone(GetPosX(), GetPosY())) {
        nav_state = OUT_OF_BOUNDS;
        break;
      }
      // Check if the predicted probability of collision is too high
      else if (p > 0.95f) {
        nav_state = COLLISION_AVOID;
        break;
      }
      // If safe navigate the drone
      else {
        heading_from_steering(s_k);
        velocity_from_collision_prob(p);

        VERBOSE_PRINT("Periodic - Steering: %.2f, Collision: %.2f\n", s_k, p);
      }
      break;

    case COLLISION_AVOID:
      // Emergency stop
      nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
      nav.speed.x = 0.0f;
      nav.speed.y = 0.0f;

      VERBOSE_PRINT("EMERGENCY STOP: collision_prob = %.2f\n", p);

      // Randomly select new search direction
      chooseRandomIncrementAvoidance();

      // Return to control if it's safe again
      if (p < 0.5f) {
        nav_state = NN_CONTROL;
      }
      break;

    case OUT_OF_BOUNDS:
      // Reorient and gently push drone back in
      nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
      nav.heading = M_PI; // Face "opposite" direction
      nav.speed.x = 0.0f;
      nav.speed.y = -0.2f; // Drift backward

      VERBOSE_PRINT("OUT OF BOUNDS: Reorienting\n");

      if (InsideObstacleZone(GetPosX(), GetPosY())) {
        nav_state = NN_CONTROL;
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

  // Set nav heading, declared in firmwares/rotorcraft/navigation.h
  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
  nav.heading = new_heading;

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

  // Get current heading
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Apply velocity in heading direction (ENU frame)
  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
  nav.speed.x = sinf(heading) * target_velocity;
  nav.speed.y = cosf(heading) * target_velocity;
  nav.speed.z = 0.0f; // Assuming flat-plane movement

  VERBOSE_PRINT("Updated forward velocity: %.2f m/s (collision_prob=%.2f)\n", target_velocity, collision_prob);
  return false;
}

/*
 * Sets the variable 'steering_input' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    s_k = 0.05f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", steering_input);
  } else {
    s_k = -0.05f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", steering_input);
  }
  return false;
}
