/*
* Depth Estimator Module for Paparazzi UAV using MiDaS
* Captures live camera feed, estimates depth using MiDaS, and assigns cost map for path planning.
*
* Author: Marcin Poplawski
*/

#include "depth_estimator.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include "mav_camera.h"

// Camera and FPS Configuration
#define DEPTH_ESTIMATOR_CAMERA_ID FRONT_CAMERA
#define DEPTH_ESTIMATOR_FPS 10

// Load MiDaS Model
void load_midas_model() {
    try {
        midas_model = torch::jit::load("/home/user/paparazzi/models/midas_v21_small_256.pt");
        midas_model.to(torch::kCUDA);
        printf("[Depth Estimator] MiDaS model loaded successfully.\n");
    } catch (const c10::Error& e) {
        printf("[Depth Estimator] Error loading MiDaS model: %s\n", e.what());
        exit(-1);
    }
}

// Capture Live Frame from UAV Camera
cv::Mat capture_frame(struct image_t *img) {
    if (img == NULL || img->buf == NULL) {
        printf("[Depth Estimator] Error: Invalid image buffer.\n");
        exit(-1);
    }

    // Convert YUV422 to OpenCV Mat format
    cv::Mat frame(img->h, img->w, CV_8UC2, img->buf);
    cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_YUYV);

    if (frame.empty()) {
        printf("[Depth Estimator] Error: Unable to capture frame from UAV camera.\n");
        exit(-1);
    }

    return frame;
}

// Preprocess Image for MiDaS
torch::Tensor preprocess_image(cv::Mat img) {
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    cv::resize(img, img, cv::Size(256, 256));  // Resize for small model

    torch::Tensor tensor_image = torch::from_blob(img.data, {1, img.rows, img.cols, 3}, torch::kByte);
    tensor_image = tensor_image.permute({0, 3, 1, 2}).to(torch::kFloat).div(255.0);
    tensor_image = tensor_image.to(torch::kCUDA);
    return tensor_image;
}

// Generate Depth Map Using MiDaS
void generate_depth_map(struct image_t *img) {
    cv::Mat frame = capture_frame(img);

    // Preprocess frame and pass through MiDaS model
    torch::Tensor input_tensor = preprocess_image(frame);
    torch::Tensor depth = midas_model.forward({input_tensor}).toTensor();

    depth = torch::nn::functional::interpolate(
        depth,
        torch::nn::functional::InterpolateFuncOptions().size({HEIGHT, WIDTH}).mode(torch::kBilinear)
    );
    depth = depth.squeeze().detach().to(torch::kCPU);

    float* depth_data = depth.data_ptr<float>();

    pthread_mutex_lock(&mutex);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            depth_map[y][x] = depth_data[y * WIDTH + x];
        }
    }
    pthread_mutex_unlock(&mutex);
}

// Normalize Depth Map
void normalize_depth_map() {
    float min_depth = 1e6;
    float max_depth = -1e6;

    pthread_mutex_lock(&mutex);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (depth_map[y][x] < min_depth) min_depth = depth_map[y][x];
            if (depth_map[y][x] > max_depth) max_depth = depth_map[y][x];
        }
    }

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            depth_map[y][x] = (depth_map[y][x] - min_depth) / (max_depth - min_depth);
        }
    }
    pthread_mutex_unlock(&mutex);
}

// Assign Costs for Path Planning
void assign_costs() {
    pthread_mutex_lock(&mutex);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (depth_map[y][x] < 0.2) {
                mask[y][x] = 0;  // High cost for close obstacles
                cost_map[y][x] = 1000;
            } else {
                mask[y][x] = 1;
                cost_map[y][x] = (int)(depth_map[y][x] * COST_SCALE);
            }
        }
    }
    pthread_mutex_unlock(&mutex);
}

// Find Lowest Cost Path
void find_lowest_cost_path() {
    int min_cost = 999999;
    int best_x = -1, best_y = -1;

    pthread_mutex_lock(&mutex);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (cost_map[y][x] < min_cost) {
                min_cost = cost_map[y][x];
                best_x = x;
                best_y = y;
            }
        }
    }
    pthread_mutex_unlock(&mutex);

    printf("[Depth Estimator] Best path to (%d, %d) with cost %d\n", best_x, best_y, min_cost);
}

// Depth Estimator Init
void depth_estimator_init(void) {
    printf("[Depth Estimator] Initializing depth estimator...\n");
    load_midas_model();

    // Register camera stream for processing
    cv_add_to_device(&DEPTH_ESTIMATOR_CAMERA_ID, depth_estimator_process, DEPTH_ESTIMATOR_FPS, 0);
    pthread_mutex_init(&mutex, NULL);
}

// Depth Estimator Processing Function
void depth_estimator_process(struct image_t *img, uint8_t camera_id) {
    if (img == NULL) return;

    generate_depth_map(img);
    normalize_depth_map();
    assign_costs();
    find_lowest_cost_path();
}

