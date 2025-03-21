#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "state.h"
#include "modules/core/abi.h"
#include "dronet_image_filter.h"

#define SRC_WIDTH  640  // Original camera resolution width
#define SRC_HEIGHT 480  // Original camera resolution height
#define DST_WIDTH  200  // Downscaled width
#define DST_HEIGHT 200  // Downscaled height
#define DOWNSAMPLE_FACTOR 4  // Factor to downscale the image

#ifndef DRONET_IMAGE_FILTER_FPS
#define DRONET_IMAGE_FILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DRONET_IMAGE_FILTER_FPS)

// ABI message definition
#ifndef DRONET_IMAGE_FILTER_ID
#define DRONET_IMAGE_FILTER_ID 1
#endif

// Define ABI message functionality
#define ABI_BROADCAST 255
#define ABI_DRONET_IMAGE_MSG 1
// Define necessary ABI functions
#define AbiSendMsgDRONET_IMAGE(sender_id, image_data) {}
#define AbiBindMsgDRONET_IMAGE(sender_id, cb, callback) {}

// Mutex for thread safety
static pthread_mutex_t mutex;
static struct image_t downscaled_image;
static struct image_t gray_image;
static float normalized_image[DST_WIDTH * DST_HEIGHT];
static volatile bool image_updated = false;  // Flag to check if a new frame is processed

// define global variables
struct infer_model {
  double s_k;
  double p;
  bool updated;
};
struct infer_model output;


// ABI event
static abi_event dronet_image_ev __attribute__((unused));

/**
 * Process image from the front camera
 */
static struct image_t *process_image(struct image_t *img, uint8_t camera_id) {
  (void)camera_id;
  if (!img) return NULL; // Safety check

  // Create downscaled image
  image_create(&downscaled_image, DST_WIDTH, DST_HEIGHT, IMAGE_YUV422);

  // Create grayscale image
  image_create(&gray_image, DST_WIDTH, DST_HEIGHT, IMAGE_GRAYSCALE);

  // Downsample image
  image_yuv422_downsample(img, &downscaled_image, DOWNSAMPLE_FACTOR);

  // Convert downscaled YUV to grayscale
  image_to_grayscale(&downscaled_image, &gray_image);

  // Normalize grayscale image
  // uint8_t *gray_buffer = (uint8_t *)gray_image.buf;
  uint8_t *gray_buffer = gray_image.buf;
  for (int i = 0; i < DST_WIDTH * DST_HEIGHT; i++) {
      normalized_image[i] = gray_buffer[i] / 255.0f;
  }

  // DroNet inference
  float steering_out[1][1];
  float collision_out[1][1];
  entry(input_tensor, steering_out, collision_out);  // DroNet model function



  // Set flag to indicate new image data is available
  pthread_mutex_lock(&mutex);

  steering_angle = steering_out[0][0];
  collision_prob = collision_out[0][0];
  image_updated = true;
  // export results
  printf("Test\n");
  pthread_mutex_unlock(&mutex);

  // Clean up allocated images to avoid memory leaks
  image_free(&downscaled_image);
  image_free(&gray_image);
  
  return &gray_image; //&gray_image;  // Return the processed grayscale image
}

/**
 * Initialization function for the Dronet Image Filter
 */
void dronet_image_filter_init(void) {
  pthread_mutex_init(&mutex, NULL);
  
  // Register video processing callback
  cv_add_to_device(&front_camera, process_image, DRONET_IMAGE_FILTER_FPS, 0);
}

/**
 * Periodic function to send processed image data via ABI messaging
 */
void dronet_image_filter_periodic(void) {
  pthread_mutex_lock(&mutex);
  if (image_updated) {
      // Send processed image data via ABI messaging
      AbiSendMsgVISUAL_DETECTION(DRONET_IMAGE_FILTER_ID, steering_angle, collision_prob);
      image_updated = false;  // Reset flag after sending
  }
  pthread_mutex_unlock(&mutex);
}