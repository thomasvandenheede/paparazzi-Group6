#ifndef DRONET_IMAGE_FILTER_OURS_H
#define DRONET_IMAGE_FILTER_OURS_H

#include <stdint.h>
#include <stdbool.h>
#include "modules/computer_vision/lib/vision/image.h"

// Image dimensions (expected model input)
#define IMG_WIDTH  200
#define IMG_HEIGHT 200

// Initialization and periodic functions
extern void dronet_image_filter_ours_init(void);
extern void dronet_image_filter_ours_periodic(void);

// Internal helpers (optional to expose depending on use case)
void preprocess_image(struct image_t *img);
void run_model_prediction(float *prob_collision);

#endif // DRONET_IMAGE_FILTER_OURS_H