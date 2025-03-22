#ifndef DRONET_IMAGE_FILTER_H
#define DRONET_IMAGE_FILTER_H

#include "modules/computer_vision/lib/vision/image.h"

// Define the size of the processed image
#define SRC_WIDTH  640  // Original camera resolution width
#define SRC_HEIGHT 480  // Original camera resolution height
#define DST_WIDTH  200  // Downscaled width
#define DST_HEIGHT 200  // Downscaled height

#define NN_OBJECT_DETECTION_ID 1

struct nn_object_t {
  float s_k;
  float p;
  bool updated;
};
extern struct nn_object_t global_output;

extern void dronet_image_filter_init(void);
extern void dronet_image_filter_periodic(void);

void preprocess_image(struct image_t *img);
void run_model_prediction(float *steering_angle, float *prob_collision);

uint8_t heading_from_steering(float steering_input);
uint8_t velocity_from_collision_prob(float collision_prob);

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