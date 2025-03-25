
#ifndef DRONET_FLOOR_DETECTOR_H
#define DRONET_FLOOR_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

// Module settings
extern uint8_t cod_lum_min;
extern uint8_t cod_lum_max;
extern uint8_t cod_cb_min;
extern uint8_t cod_cb_max;
extern uint8_t cod_cr_min;
extern uint8_t cod_cr_max;

extern bool cod_draw;

// Module functions
extern void dronet_floor_detector_init(void);
extern void dronet_floor_detector_periodic(void);

#endif /* DRONET_FLOOR_DETECTOR_H */