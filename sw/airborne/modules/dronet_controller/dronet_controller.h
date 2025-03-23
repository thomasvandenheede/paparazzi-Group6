#ifndef DRONET_CONTROLLER_H
#define DRONET_CONTROLLER_H

#include <stdint.h>

// extern struct nn_object_t global_output;
// extern void entry(const float tensor_input_1[1][200][200][1], float tensor_dense_1[1][1], float tensor_activation_8[1][1]);

// Function declarations
extern void dronet_controller_init(void);
extern void dronet_controller_periodic(void);

#endif  // DRONET_CONTROLLER_H