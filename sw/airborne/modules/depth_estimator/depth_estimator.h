#ifndef DEPTH_ESTIMATOR_H
#define DEPTH_ESTIMATOR_H

#include <stdbool.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_THRESHOLD 1000
#define MAX_DEPTH 5000
#define COST_SCALE 100

// Camera and processing rate settings
#define DEPTH_ESTIMATOR_CAMERA_ID FRONT_CAMERA
#define DEPTH_ESTIMATOR_FPS 10

// Depth map and cost map
extern float depth_map[HEIGHT][WIDTH];
extern int cost_map[HEIGHT][WIDTH];
extern bool mask[HEIGHT][WIDTH];

// MiDaS Model
extern torch::jit::script::Module midas_model;

// Mutex for thread safety
extern pthread_mutex_t mutex;

// Initialization and setup
void depth_estimator_init(void);

// Periodic processing
void depth_estimator_periodic(void);
void depth_estimator_process(struct image_t *img, uint8_t camera_id);

// MiDaS model handling
void load_midas_model(void);
torch::Tensor preprocess_image(cv::Mat img);
void generate_depth_map(struct image_t *img);
void normalize_depth_map(void);

// Cost assignment and path planning
void assign_costs(void);
void find_lowest_cost_path(void);

// Frame capture and conversion
cv::Mat capture_frame(struct image_t *img);

#endif /* DEPTH_ESTIMATOR_H */
