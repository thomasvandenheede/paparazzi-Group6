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

// Define maximum horizontal speed of the drone from airframe configuration
#ifndef V_MAX
#define V_MAX GUIDANCE_H_REF_MAX_SPEED
#endif

#define ABI_BROADCAST 255
#define ABI_DRONET_IMAGE_MSG 1

#ifndef AbiBindMsgVISUAL_DETECTION
#define AbiBindMsgVISUAL_DETECTION(sender_id, steering_angle, collision_prob) {}
#endif

// #ifndef VERBOSE_PRINT
// #define VERBOSE_PRINT(args...) printf(args)
// #endif

extern int32_t color_count;
extern enum navigation_state_t navigation_state;
extern float obstacle_free_confidence;
extern float maxDistance;
extern float max_trajectory_confidence;
extern float heading_increment;

// Bebop camera dimensions
// #define SRC_WIDTH 640
// #define SRC_HEIGHT 480
#define DST_WIDTH 200
#define DST_HEIGHT 200

// DroNet parameters
#define ALPHA 0.7f
#define BETA 0.5f

// Define maximum horizontal speed of the drone from airframe configuration
#ifndef V_MAX
#define V_MAX GUIDANCE_H_REF_MAX_SPEED
#endif

// #define V_MAX 1.0f            // Max velocity (m/s)
// #define MAX_YAW_RATE 60.0f    // Max yaw rate (degrees per second)

// #define K_V 1.5f              // Proportional gain for velocity
// #define K_YAW 30.0f           // Proportional gain for yaw
// #define DT 0.1f               // Time step

// #define OA_COLOR_COUNT_FRAC 0.1f
// #define MAX_TRAJECTORY_CONFIDENCE 10
// #define MAX_DISTANCE 5.0f

// ABI message definition
#ifndef DRONET_IMAGE_FILTER_ID
#define DRONET_IMAGE_FILTER_ID 1
#endif

// Global variables
// static float normalized_image[DST_WIDTH * DST_HEIGHT];
// static abi_event dronet_image_ev;

// ABI callback function to receive processed image data
// static void dronet_image_cb(uint8_t __attribute__((unused)) sender_id, float *image_data) {
//   memcpy(normalized_image, image_data, sizeof(normalized_image));
// }

// // Global variables for image processing
// uint8_t raw_camera_buffer[SRC_WIDTH * SRC_HEIGHT * 2];
// uint8_t grayscale_image[DST_WIDTH * DST_HEIGHT];

// static float last_steering_angle = 0;
// static float last_collision_prob = 0;
// static float last_output = 0;

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[dronet_controller->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// int navigation_state = SAFE;
enum navigation_state_t navigation_state = SAFE;
int color_count = 0;
float oa_color_count_frac = 0.1;  // Example threshold
float obstacle_free_confidence = 0.0f;
float max_trajectory_confidence = 10.0f;
float maxDistance = 5.0f;  // Example max distance

// static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
// static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
// static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
// static uint8_t increase_nav_heading(float incrementDegrees);
// static uint8_t chooseRandomIncrementAvoidance(void);

// Define last values as static variables to retain their value between function calls
// static float last_theta_k = 0.0f;  
// static float last_velocity = 0.0f;  

static pthread_mutex_t mutex;

float s_k = 0.f;
float p = 1.f;

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
// static abi_event dronet_image_ev;
// static void dronet_image_cb(float steering_angle, float collision_prob)
// {
//   s_k = steering_angle;
//   p = collision_prob;
// }

// Initialization function
void dronet_controller_init(void) {
  pthread_mutex_init(&mutex, NULL);

  // Bind the ABI message to receive image data
  AbiBindMsgVISUAL_DETECTION(DRONET_IMAGE_FILTER_ID, &dronet_image_ev, dronet_image_cb);
}


void dronet_controller_periodic(void) {
    if (!autopilot_in_flight()) {
        return;
    }

    float steering_angle, collision_prob;

    pthread_mutex_lock(&mutex);
    steering_angle = global_output.s_k;
    collision_prob = global_output.p;
    pthread_mutex_unlock(&mutex);

    if (steering_angle == 0 && collision_prob == 0) {
      VERBOSE_PRINT("No valid inference results available.\n");
      return;
    }

    // Update heading and velocity
    heading_from_steering(steering_angle);
    velocity_from_collision_prob(collision_prob);

    VERBOSE_PRINT("Periodic - Steering: %.2f, Collision: %.2f\n", steering_angle, collision_prob);
    
    // float theta_k = (1.0f - BETA) * last_theta_k + BETA * (s_k * (M_PI / 2.0f));
    // last_theta_k = theta_k;  // Update stored value

    // // Convert steering angle to heading change
    // float steering_input = K_YAW * theta_k;

    // // Limit yaw rate
    // steering_input = fmaxf(fminf(heading_increment, MAX_YAW_RATE * DT), -MAX_YAW_RATE * DT);
    // heading_from_steering(steering_input);

    // float velocity = (1.0f - ALPHA) * last_velocity + ALPHA * (1.0f - p) * V_MAX;
    // last_velocity = velocity;  // Update stored value

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
