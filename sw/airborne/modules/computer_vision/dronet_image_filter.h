#ifndef DRONET_IMAGE_FILTER_H
#define DRONET_IMAGE_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include "modules/computer_vision/lib/vision/image.h"

// Image dimensions (expected model input)
#define IMG_WIDTH  200
#define IMG_HEIGHT 200

// ABI Message ID for this detection module
#define NN_OBJECT_DETECTION_ID 1

// Output structure from the neural network
struct nn_object_t {
  float s_k;     // Steering input
  float p;       // Probability of collision
  bool updated;  // Flag to indicate new data available
};

// Expose the global prediction result
extern struct nn_object_t global_output;

// Initialization and periodic functions
extern void dronet_image_filter_init(void);
extern void dronet_image_filter_periodic(void);

// Internal helpers (optional to expose depending on use case)
void preprocess_image(struct image_t *img);
void run_model_prediction(float *steering_input, float *prob_collision);

// uint8_t heading_from_steering(float steering_input);
// uint8_t velocity_from_collision_prob(float collision_prob);

// // ABI message ID
// #ifndef DRONET_IMAGE_FILTER_ID
// #define DRONET_IMAGE_FILTER_ID 1
// #endif

// // ABI message definitions
// #define ABI_BROADCAST 255
// #define ABI_DRONET_IMAGE_MSG 1

// // ABI message function declarations
// #define AbiBindMsgVISUAL_DETECTION(steering_angle, collision_prob=]) {}
// #define AbiBindMsgVISUAL_DETECTION(sender_id, cb, callback) {}


#endif // DRONET_IMAGE_FILTER_H