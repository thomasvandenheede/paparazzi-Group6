#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "state.h"
#include "modules/core/abi.h"
#include "dronet.h"

#ifndef DRONET_COMBINED_FILTER_FPS
#define DRONET_COMBINED_FILTER_FPS 0
#endif
PRINT_CONFIG_VAR(DRONET_COMBINED_FILTER_FPS)

#define IMG_WIDTH 200
#define IMG_HEIGHT 200
#define INV_255 (1.0f / 255.0f)

#define DRONET_COMBINED_MESSAGE_ID 42

// Settings for floor detection (Y, U, V range)
#define FLOOR_LUM_MIN 50
#define FLOOR_LUM_MAX 255
#define FLOOR_CB_MIN 0
#define FLOOR_CB_MAX 140
#define FLOOR_CR_MIN 100
#define FLOOR_CR_MAX 255

static pthread_mutex_t mutex;

struct dronet_combined_output_t {
  float s_k;
  float p;
  int32_t floor_count;
  int32_t floor_centroid;
  bool updated;
};

static struct dronet_combined_output_t combined_output;

static float input_tensor[1][IMG_HEIGHT][IMG_WIDTH][1];

static void preprocess_image(struct image_t *img) {
  if (!img || img->w != IMG_WIDTH || img->h != IMG_HEIGHT) {
    printf("[preprocess] Invalid image\n");
    return;
  }

  uint32_t cnt = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  for (int y = 0; y < img->h; y++) {
    for (int x = 0; x < img->w; x++) {
      uint8_t *yp = &buffer[y * 2 * img->w + 2 * x + 1];
      input_tensor[0][y][x][0] = (*yp) * INV_255;

      // Floor detection using YUV thresholds
      uint8_t *up, *vp;
      if (x % 2 == 0) {
        up = &buffer[y * 2 * img->w + 2 * x];
        vp = &buffer[y * 2 * img->w + 2 * x + 2];
      } else {
        up = &buffer[y * 2 * img->w + 2 * x - 2];
        vp = &buffer[y * 2 * img->w + 2 * x];
      }
      if ((*yp >= FLOOR_LUM_MIN && *yp <= FLOOR_LUM_MAX) &&
          (*up >= FLOOR_CB_MIN && *up <= FLOOR_CB_MAX) &&
          (*vp >= FLOOR_CR_MIN && *vp <= FLOOR_CR_MAX)) {
        cnt++;
        tot_y += y;
      }
    }
  }

  pthread_mutex_lock(&mutex);
  combined_output.floor_count = cnt;
  combined_output.floor_centroid = (cnt > 0) ? (int32_t)(IMG_HEIGHT / 2 - tot_y / cnt) : 0;
  pthread_mutex_unlock(&mutex);

  printf("[preprocess] Top-left (Y): %f\n", input_tensor[0][0][0][0]);
}

static void run_model_prediction(float *steering_input, float *prob_collision) {
  float tensor_dense_1[1][1];
  float tensor_activation_8[1][1];

  entry(input_tensor, tensor_dense_1, tensor_activation_8);

  *steering_input = tensor_dense_1[0][0];
  *prob_collision = tensor_activation_8[0][0];
}

static struct image_t *combined_detector(struct image_t *img, uint8_t cam_id __attribute__((unused))) {
  if (!img) return NULL;

  preprocess_image(img);

  float s_k, p;
  run_model_prediction(&s_k, &p);

  pthread_mutex_lock(&mutex);
  combined_output.s_k = s_k;
  combined_output.p = p;
  combined_output.updated = true;
  pthread_mutex_unlock(&mutex);

  printf("[dronet_combined_filter] Inference: s_k=%.2f, p=%.2f\n", s_k, p);
  return img;
}

void dronet_combined_filter_init(void) {
  memset(&combined_output, 0, sizeof(combined_output));
  pthread_mutex_init(&mutex, NULL);

#ifdef NN_OBJECT_DETECTOR_CAMERA
  cv_add_to_device(&NN_OBJECT_DETECTOR_CAMERA, combined_detector, DRONET_COMBINED_FILTER_FPS, 0);
#endif
}

void dronet_combined_filter_periodic(void) {
  static struct dronet_combined_output_t local;
  pthread_mutex_lock(&mutex);
  memcpy(&local, &combined_output, sizeof(local));
  combined_output.updated = false;
  pthread_mutex_unlock(&mutex);

  if (local.updated) {
    AbiSendMsgDRONET_COMBINED(DRONET_COMBINED_MESSAGE_ID, local.s_k, local.p, local.floor_count, local.floor_centroid);
  }
}
