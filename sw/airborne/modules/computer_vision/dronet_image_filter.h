#ifndef DRONET_IMAGE_FILTER_H
#define DRONET_IMAGE_FILTER_H

#include "modules/computer_vision/lib/vision/image.h"

// Define the size of the processed image
#define DST_WIDTH 200
#define DST_HEIGHT 200

// Function declarations
/**
 * @brief Initializes the Dronet Image Filter module.
 */
void dronet_image_filter_init(void);

/**
 * @brief Periodic function to send processed image data via ABI messaging.
 */
void dronet_image_filter_periodic(void);

#endif // DRONET_IMAGE_FILTER_H