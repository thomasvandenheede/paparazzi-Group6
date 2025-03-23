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

// Initialization and periodic functions
extern void dronet_image_filter_init(void);
extern void dronet_image_filter_periodic(void);

// Internal helpers (optional to expose depending on use case)
void preprocess_image(struct image_t *img);
void run_model_prediction(float *steering_input, float *prob_collision);

// // Output structure from the neural network
// struct nn_object_t {
//   float s_k;     // Steering input
//   float p;       // Probability of collision
//   bool updated;  // Flag to indicate new data available
// };

// // Expose the global prediction result
// extern struct nn_object_t global_output;

// uint8_t heading_from_steering(float steering_input);
// uint8_t velocity_from_collision_prob(float collision_prob);

#endif // DRONET_IMAGE_FILTER_H