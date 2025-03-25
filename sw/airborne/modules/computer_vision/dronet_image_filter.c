#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "state.h"
#include "modules/core/abi.h"
#include "dronet_image_filter.h"
#include "dronet.h"

#ifndef DRONET_IMAGE_FILTER_FPS
#define DRONET_IMAGE_FILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DRONET_IMAGE_FILTER_FPS)

// Mutex for thread safety
static pthread_mutex_t mutex;

// Define global variables
struct nn_object_t {
  float s_k;
  float p;
  bool updated;
};
struct nn_object_t global_output;

#define IMG_WIDTH 200
#define IMG_HEIGHT 200
#define INV_255 (1.0f / 255.0f)

// Declare tensor as 4D array: [batch][height][width][channels]
static float input_tensor[1][IMG_HEIGHT][IMG_WIDTH][1];

void preprocess_image(struct image_t *img)
{
  if (!img) {
    printf("Image invalid!\n");
    return; // Safety check
  }

  if (img->buf == NULL) {
    printf("[preprocess] Image buffer is NULL!\n");
  }

  // Safety check to guarantee consistent image dimensions
  if (img->w != IMG_WIDTH || img->h != IMG_HEIGHT) {
    printf("Unexpected image size!\n");
    return;
  }

  uint8_t *buffer = img->buf;

  for (int y = 0; y < img->h; y++) {
    for (int x = 0; x < img->w; x++) {
      // Get Y (luma) value from YUV422 buffer (GRAYSCALE IMAGE)
      uint8_t *yp; 
      yp = &buffer[y * 2 * img->w + 2 * x + 1];

      // printf("[preprocess] Raw Y (0,0): %d\n", *yp);

      // Rotate 90° counterclockwise when storing in the tensor
      int rotated_x = y;
      int rotated_y = IMG_WIDTH - 1 - x;

      // Create input tensor to the model from the given image
      input_tensor[0][rotated_y][rotated_x][0] = (*yp) * INV_255;
      // input_tensor[0][y][x][0] = (*yp) * INV_255;
    }
  }

  printf("[preprocess] Top-left (original): %f\n", input_tensor[0][0][0][0]);
  printf("[preprocess] Bottom-left (rotated): %f\n", input_tensor[0][IMG_HEIGHT - 1][0][0]);

}

void run_model_prediction(float *steering_input, float *prob_collision)
{
  // Output tensors from the model
  float tensor_dense_1[1][1];       // Output: steering input
  float tensor_activation_8[1][1];  // Output: probability of collision

  // Call model entry function
  entry(input_tensor, tensor_dense_1, tensor_activation_8);

  // Copy results to output pointers
  *steering_input = tensor_dense_1[0][0];
  *prob_collision = tensor_activation_8[0][0];

  // *steering_input = 0.0f;
  // *prob_collision = 0.1f;

}


static struct image_t *nn_object_detector(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  (void)camera_id; // Explicit that the camera_id is unused

  if (!img) return NULL; // Safety check

  // Step 1: Preprocess the image into the model input tensor
  preprocess_image(img);

  // Step 2: Run the model to get predictions
  float steering_input, collision_prob;
  run_model_prediction(&steering_input, &collision_prob);

  printf("[dronet_image_filter] Inference: s_k=%.2f, p=%.2f\n", steering_input, collision_prob);

  // Step 3: Store results in global struct safely
  pthread_mutex_lock(&mutex);
  global_output.s_k = steering_input;
  global_output.p = collision_prob;
  global_output.updated = true;
  pthread_mutex_unlock(&mutex);

  return img; // Return the original (or processed) image if needed
}


/**
 * Initialization function for the Dronet Image Filter
 */
void dronet_image_filter_init(void) {
  memset(&global_output, 0, sizeof(struct nn_object_t));    // LOOK INTO THIS
  pthread_mutex_init(&mutex, NULL);

  #ifdef NN_OBJECT_DETECTOR_CAMERA
    // Register video processing callback
    cv_add_to_device(&NN_OBJECT_DETECTOR_CAMERA, nn_object_detector, NN_OBJECT_DETECTOR_FPS, 1);
  #endif
}

/**
 * Periodic function to send processed image data via ABI messaging
 */
void dronet_image_filter_periodic(void) {

  static struct nn_object_t local_output;
  pthread_mutex_lock(&mutex);
  memcpy(&local_output, &global_output, sizeof(struct nn_object_t));
  pthread_mutex_unlock(&mutex);

  if (local_output.updated) {
      // Send processed image data via ABI messaging
      AbiSendMsgNN_DETECTION(NN_OBJECT_DETECTION_ID, local_output.s_k, local_output.p);
      local_output.updated = false;  // Reset flag after sending
  }
}