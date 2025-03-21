#ifndef DRONET_IMAGE_FILTER_H
#define DRONET_IMAGE_FILTER_H

#include "modules/computer_vision/lib/vision/image.h"

// Define the size of the processed image
#define SRC_WIDTH  640  // Original camera resolution width
#define SRC_HEIGHT 480  // Original camera resolution height
#define DST_WIDTH  200  // Downscaled width
#define DST_HEIGHT 200  // Downscaled height

// Function declarations
/**
 * @brief Initializes the Dronet Image Filter module.
 */
extern void dronet_image_filter_init(void);

/**
 * @brief Periodic function to send processed image data via ABI messaging.
 */
extern void dronet_image_filter_periodic(void);

// ABI message ID
#ifndef DRONET_IMAGE_FILTER_ID
#define DRONET_IMAGE_FILTER_ID 1
#endif

// ABI message definitions
#define ABI_BROADCAST 255
#define ABI_DRONET_IMAGE_MSG 1

// ABI message function declarations
#define AbiSendMsgDRONET_IMAGE(sender_id, image_data) {}
#define AbiBindMsgDRONET_IMAGE(sender_id, cb, callback) {}


#endif // DRONET_IMAGE_FILTER_H