#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "state.h"

#define SRC_WIDTH  640  // Original camera resolution width
#define SRC_HEIGHT 480  // Original camera resolution height
#define DST_WIDTH  200  // Downscaled width
#define DST_HEIGHT 200  // Downscaled height
#define DOWNSAMPLE_FACTOR 4  // Factor to downscale the image

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


// Mutex for thread safety
static pthread_mutex_t mutex;
static struct image_t gray_image;
static struct image_t downscaled_image;
static float normalized_image[DST_WIDTH * DST_HEIGHT];

// Image processing callback function
static struct image_t *process_image(struct image_t *img) {
  pthread_mutex_lock(&mutex);
  
  // Ensure the output images are properly allocated
  image_create(&downscaled_image, DST_WIDTH, DST_HEIGHT, IMAGE_YUV422);
  image_create(&gray_image, DST_WIDTH, DST_HEIGHT, IMAGE_GRAYSCALE);

  // Downsample image
  image_yuv422_downsample(img, &downscaled_image, DOWNSAMPLE_FACTOR);

  // Convert downscaled YUV to grayscale
  image_to_grayscale(&downscaled_image, &gray_image);

  // Normalize grayscale image
  for (int i = 0; i < DST_WIDTH * DST_HEIGHT; i++) {
      normalized_image[i] = gray_image.buf[i] / 255.0f;
  }

  pthread_mutex_unlock(&mutex);
  
  return &normalized_image;  // Return the processed grayscale, normalized image
}

// Initialization function
void dronet_image_filter_init(void) {
    pthread_mutex_init(&mutex, NULL);
    
    // Register video processing callback
    cv_add_to_device(&front_camera, process_image, COLORFILTER_FPS, 0);
}
