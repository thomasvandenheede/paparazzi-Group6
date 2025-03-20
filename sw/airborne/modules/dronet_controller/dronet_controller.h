#ifndef DRONET_CONTROLLER_H
#define DRONET_CONTROLLER_H

#include <stdint.h>

// DroNet control parameters
#define DST_WIDTH 200
#define DST_HEIGHT 200

// Function declarations
extern void dronet_controller_init(void);
extern void dronet_controller_periodic(void);

#endif 