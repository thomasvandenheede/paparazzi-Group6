/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef DRONET_CONTROLLER_H
#define DRONET_CONTROLLER_H

#include <stdint.h>

// settings
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};
extern enum navigation_state_t navigation_state;

extern int color_count;
extern float oa_color_count_frac;
extern float obstacle_free_confidence;
extern float max_trajectory_confidence;
extern float maxDistance;

void downscale_and_convert_gray(uint8_t *src_yuv, uint8_t *dst_gray);
void normalize_image(uint8_t *gray_img, float *normalized_img);
void entry(const float tensor_input_1[1][200][200][1], 
  float tensor_dense_1[1][1], 
  float tensor_activation_8[1][1]);

#ifdef VERBOSE_PRINT
#undef VERBOSE_PRINT
#endif
#define VERBOSE_PRINT(fmt, ...) fprintf(stderr, "[Dronet Controller] " fmt, ##__VA_ARGS__)

// functions
extern void dronet_controller_init(void);
extern void dronet_controller_periodic(void);

#endif

