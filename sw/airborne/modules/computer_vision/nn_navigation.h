#ifndef NN_NAVIGATION_H
#define NN_NAVIGATION_H

#define NN_CAMERA_ID FRONT_CAMERA
#define NN_FPS 5

// Input image 520x240, reduce for saving memory
#define NN_INPUT_WIDTH 320
#define NN_INPUT_HEIGHT 160

#define SMOOTHING_FACTOR 0.8f

extern pthread_mutex_t mutex;

void nn_navigation_init(void);
void nn_navigation_periodic(struct image_t *img, uint8_t camera_id);

#endif /* NN_NAVIGATION_H */
