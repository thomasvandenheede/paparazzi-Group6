/*
 * Neural Network-based navigation module for Paparazzi UAV
 * Takes live camera feed, converts it to grayscale, and runs previously trained ONNX inference.
 */

#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>

// Include ONNX-generated model
#include "onnx_model.h"

// Mutex for thread safety
static pthread_mutex_t mutex;
static int last_output = -1;

int smooth_output(int current_output) {
    if (last_output == -1) {
        last_output = current_output;
    } else {
        last_output = (int)(SMOOTHING_FACTOR * last_output + (1.0f - SMOOTHING_FACTOR) * current_output);
    }
    return last_output;
}

// Convert YUV422 to grayscale
void yuv422_to_grayscale(uint8_t *yuv_buffer, float *gray_buffer, int width, int height) {
    int index = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x += 2) {
            // Convert luminance information from Y channel to grayscale
            uint8_t y1 = yuv_buffer[y * width * 2 + 2 * x];
            uint8_t y2 = yuv_buffer[y * width * 2 + 2 * x + 2];
            // Normalize values to range [0, 1]
            gray_buffer[index++] = y1 * (1.0f / 255.0f);
            gray_buffer[index++] = y2 * (1.0f / 255.0f);
        }
    }
}

// Preprocess frame for ONNX
void preprocess_frame(struct image_t *img, float *input_buffer) {
    if (img == NULL || img->buf == NULL) {
        printf("[NN Navigation] WARNING: Lost camera input!\n");
        return;
    }
    yuv422_to_grayscale(img->buf, input_buffer, img->w, img->h);
}

// Interpret model output
void handle_nn_output(int output) {
    output = smooth_output(output);
    if (output < 0 || output > 2) return;

    switch (output) {
        case 0:
            printf("[NN Navigation] Output: LEFT\n");
            AbiSendMsgCOMMANDS(ABI_BROADCAST, 0, -1.0, 0, 0); // Fly left
            break;
        case 1:
            printf("[NN Navigation] Output: CENTER\n");
            AbiSendMsgCOMMANDS(ABI_BROADCAST, 0, 0, 0, 0); // Fly straight
            break;
        case 2:
            printf("[NN Navigation] Output: RIGHT\n");
            AbiSendMsgCOMMANDS(ABI_BROADCAST, 0, 1.0, 0, 0); // Fly right
            break;
    }
}

// Neural Network inference
void run_nn_inference(struct image_t *img) {
    float input_buffer[NN_INPUT_WIDTH * NN_INPUT_HEIGHT];
    preprocess_frame(img, input_buffer);

    float output[3];  // left, center, right

    pthread_mutex_lock(&mutex);
    nn_inference(input_buffer, output);
    pthread_mutex_unlock(&mutex);

    if (isnan(output[0]) || isnan(output[1]) || isnan(output[2]) || 
        isinf(output[0]) || isinf(output[1]) || isinf(output[2])) {
        printf("[NN Navigation] WARNING: Invalid model output!\n");
        return;
    }

    printf("[NN Navigation] Scores: Left=%.3f, Center=%.3f, Right=%.3f\n",
            output[0], output[1], output[2]);

    int predicted_class = 0;
    float max_score = output[0];
    for (int i = 1; i < 3; i++) {
        if (output[i] > max_score) {
            max_score = output[i];
            predicted_class = i;
        }
    }
    handle_nn_output(predicted_class);
}

// Initialization function 
void nn_navigation_init(void) {
    printf("[NN Navigation] Initializing...\n");
    cv_add_to_device(&NN_CAMERA_ID, nn_navigation_periodic, NN_FPS, 0);
    pthread_mutex_init(&mutex, NULL);
}

// Periodic function
void nn_navigation_periodic(struct image_t *img, uint8_t camera_id) {
    pthread_mutex_lock(&mutex);
    run_nn_inference(img);
    pthread_mutex_unlock(&mutex);
}
