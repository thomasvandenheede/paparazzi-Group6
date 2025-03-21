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

#define ABI_BROADCAST 255
#define ABI_DRONET_IMAGE_MSG 1
#define AbiSendMsgDRONET_IMAGE(sender_id, image_data) {}
#define AbiBindMsgDRONET_IMAGE(sender_id, cb, callback) {}

#ifndef VERBOSE_PRINT
#define VERBOSE_PRINT(args...) printf(args)
#endif

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
#define V_MAX 1.0f            // Max velocity (m/s)
#define MAX_YAW_RATE 60.0f    // Max yaw rate (degrees per second)

#define K_V 1.5f              // Proportional gain for velocity
#define K_YAW 30.0f           // Proportional gain for yaw
#define DT 0.1f               // Time step

// #define OA_COLOR_COUNT_FRAC 0.1f
// #define MAX_TRAJECTORY_CONFIDENCE 10
// #define MAX_DISTANCE 5.0f

// ABI message definition
#ifndef DRONET_IMAGE_FILTER_ID
#define DRONET_IMAGE_FILTER_ID 1
#endif

// Global variables
static float normalized_image[DST_WIDTH * DST_HEIGHT];
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

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

// Define last values as static variables to retain their value between function calls
static float last_theta_k = 0.0f;  
static float last_velocity = 0.0f;  

// Initialization function
void dronet_controller_init(void) {
  // Bind the ABI message to receive image data
  AbiBindMsgDRONET_IMAGE(DRONET_IMAGE_FILTER_ID, &dronet_image_ev, dronet_image_cb);
}


void dronet_controller_periodic(void) {
    if (!autopilot_in_flight()) {
        return;
    }

    // Run DroNet inference
    float steering_angle, collision_prob;
    // float steering_angle[1];
    // float collision_prob[1];
    float input_tensor[1][200][200][1];

    // Convert received image data into DroNet input format
    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 200; j++) {
            input_tensor[0][i][j][0] = normalized_image[i * 200 + j];
        }
    }

    float steering_output[1][1] = {{0.0f}};
    float collision_output[1][1] = {{1.0f}}; // Assume worst-case (maximum collision probability)

    entry(input_tensor, steering_output, collision_output);

    // if (steering_angle != NULL && collision_prob != NULL) {
    //   entry(input_tensor, &steering_angle, &collision_prob);
    // } else {
    //   fprintf(stderr, "ONNX output is NULL!\n");
    // }

    steering_angle = steering_output[0][0];
    collision_prob = collision_output[0][0];

    float theta_k = (1.0f - BETA) * last_theta_k + BETA * (steering_angle * (M_PI / 2.0f));
    last_theta_k = theta_k;  // Update stored value

    // Convert steering angle to heading change
    float heading_increment = K_YAW * theta_k;

    // Limit yaw rate
    heading_increment = fmaxf(fminf(heading_increment, MAX_YAW_RATE * DT), -MAX_YAW_RATE * DT);
    increase_nav_heading(heading_increment);

    float velocity = (1.0f - ALPHA) * last_velocity + ALPHA * (1.0f - collision_prob) * V_MAX;
    last_velocity = velocity;  // Update stored value

    float move_distance = K_V * velocity * DT;
    move_distance = fminf(move_distance, V_MAX * DT);

    // Move the drone
    moveWaypointForward(WP_TRAJECTORY, move_distance);
}

// enum navigation_state_t {
//   SAFE,
//   OBSTACLE_FOUND,
//   SEARCH_FOR_SAFE_HEADING,
//   OUT_OF_BOUNDS
//   };

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
// // Function to downscale 640x480 YUV image to 200x200 grayscale
// void downscale_and_convert_gray(uint8_t *src_yuv, uint8_t *dst_gray) {
//   int x_ratio = SRC_WIDTH / DST_WIDTH;
//   int y_ratio = SRC_HEIGHT / DST_HEIGHT;

//   for (int y = 0; y < DST_HEIGHT; y++) {
//       for (int x = 0; x < DST_WIDTH; x++) {
//           int src_x = x * x_ratio;
//           int src_y = y * y_ratio;
//           int src_index = (src_y * SRC_WIDTH + src_x) * 2;  // YUV422 format (Y, U, Y, V)

//           // Extract grayscale from Y component
//           dst_gray[y * DST_WIDTH + x] = src_yuv[src_index];  // Use Y component only
//       }
//   }
// }

// void normalize_image(uint8_t *gray_img, float *normalized_img) {
//   for (int i = 0; i < DST_WIDTH * DST_HEIGHT; i++) {
//       normalized_img[i] = gray_img[i] / 255.0f;  // Normalize to [0,1]
//   }
// }


// float dronet_input[DST_WIDTH * DST_HEIGHT];  // CNN input buffer
// downscale_and_convert_gray(raw_camera_buffer, grayscale_image);
// normalize_image(grayscale_image, dronet_input);

// /*
//  * Initialisation function, setting the colour filter, random seed and heading_increment
//  */
// void dronet_controller_init(void)
// {
//   // Initialise random values
//   srand(time(NULL));
//   chooseRandomIncrementAvoidance();

//   // bind our colorfilter callbacks to receive the color filter outputs
//   // AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
// }

// /*
//  * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
//  */
// void dronet_controller_periodic(void)
// {
//   // only evaluate our state machine if we are flying
//   if(!autopilot_in_flight()){
//     return;
//   }
//     // Downscale and normalize input for DroNet
//     float dronet_input[DST_WIDTH * DST_HEIGHT];  // CNN input buffer
//     downscale_and_convert_gray(raw_camera_buffer, grayscale_image);
//     normalize_image(grayscale_image, dronet_input);

//     // Run DroNet inference
//     float steering_angle, collision_prob;

//     float input_tensor[1][200][200][1];
//     for (int i = 0; i < 200; i++) {
//         for (int j = 0; j < 200; j++) {
//             input_tensor[0][i][j][0] = dronet_input[i * 200 + j];
//         }
//     }

//     float steering_output[1][1];
//     float collision_output[1][1];

//     entry(input_tensor, steering_output, collision_output);

//     steering_angle = steering_output[0][0];
//     collision_prob = collision_output[0][0];

//     // entry(dronet_input, &steering_angle, &collision_prob);

//     if (isnan(steering_angle) || isnan(collision_prob)) {
//         VERBOSE_PRINT("Invalid DroNet output! Using last known good values.\n");
//         steering_angle = last_steering_angle;
//         collision_prob = last_collision_prob;
//     } else {
//         last_steering_angle = steering_angle;
//         last_collision_prob = collision_prob;
//     }
    
//     // Convert steering angle to heading
//     float theta_k = (1.0f - BETA) * last_output + BETA * (steering_angle * (M_PI / 2.0f));
//     float heading_increment = K_YAW * theta_k;

//     // Smooth out yaw rate changes
//     heading_increment = fmaxf(fminf(heading_increment, MAX_YAW_RATE * DT), -MAX_YAW_RATE * DT);

//     increase_nav_heading(heading_increment);

//     // Convert collision probability to velocity
//     float velocity = (1.0f - ALPHA) * last_output + ALPHA * (1.0f - collision_prob) * V_MAX;
//     float move_distance = K_V * velocity * DT;

//     // Limit maximum movement distance to avoid instability
//     move_distance = fminf(move_distance, V_MAX * DT);

//     moveWaypointForward(WP_TRAJECTORY, move_distance);

//   // compute current color thresholds
//   int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

//   VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);

//   // update our safe confidence using color threshold
//   if(color_count < color_count_threshold){
//     obstacle_free_confidence++;
//   } else {
//     obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
//   }

//   // bound obstacle_free_confidence
//   Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

//   float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

//   switch (navigation_state){
//     case SAFE:
//       // Move waypoint forward
//       moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
//       if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
//         navigation_state = OUT_OF_BOUNDS;
//       } else if (obstacle_free_confidence == 0){
//         navigation_state = OBSTACLE_FOUND;
//       } else {
//         moveWaypointForward(WP_GOAL, moveDistance);
//         moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
//       }

//       break;
//     case OBSTACLE_FOUND:
//       // stop
//       waypoint_move_here_2d(WP_GOAL);
//       waypoint_move_here_2d(WP_RETREAT);
//       waypoint_move_here_2d(WP_TRAJECTORY);

//       // randomly select new search direction
//       chooseRandomIncrementAvoidance();

//       navigation_state = SEARCH_FOR_SAFE_HEADING;

//       break;
//     case SEARCH_FOR_SAFE_HEADING:
//       increase_nav_heading(heading_increment);

//       // make sure we have a couple of good readings before declaring the way safe
//       if (obstacle_free_confidence >= 2){
//         navigation_state = SAFE;
//       }
//       break;
//     case OUT_OF_BOUNDS:
//       increase_nav_heading(heading_increment);
//       moveWaypointForward(WP_TRAJECTORY, 1.5f);
//       moveWaypointForward(WP_RETREAT, -1.0f);

//       if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
//         // add offset to head back into arena
//         increase_nav_heading(heading_increment);

//         // reset safe counter
//         obstacle_free_confidence = 0;

//         // ensure direction is safe before continuing
//         navigation_state = SEARCH_FOR_SAFE_HEADING;
//       }
//       break;
//     default:
//       break;
//   }
//   return;
// }

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

