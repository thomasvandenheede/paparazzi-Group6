#ifndef DEPTH_PROCESSING_H
#define DEPTH_PROCESSING_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_THRESHOLD 1000
#define GROUND_THRESHOLD 128
#define MAX_DEPTH 5000
#define COST_SCALE 100

// Depth map and cost map
extern int depth_map[HEIGHT][WIDTH];
extern int cost_map[HEIGHT][WIDTH];
extern bool mask[HEIGHT][WIDTH];

// Function prototypes
void read_depth_map(void);
void smooth_depth_map(void);
void normalize_depth_map(void);
void remove_ground_plane(void);
void detect_high_depth_areas(void);
void assign_costs(void);
void find_lowest_cost_path(void);

#endif /* DEPTH_MAP_PROCESSOR_H */
