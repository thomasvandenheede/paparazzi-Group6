#include "depth_processing.h"

int depth_map[HEIGHT][WIDTH];
int cost_map[HEIGHT][WIDTH];
bool mask[HEIGHT][WIDTH];

void read_depth_map() {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            depth_map[y][x] = rand() % MAX_DEPTH;
        }
    }
}

void smooth_depth_map() {
    int temp[HEIGHT][WIDTH];
    
    for (int y = 1; y < HEIGHT - 1; y++) {
        for (int x = 1; x < WIDTH - 1; x++) {
            temp[y][x] = (
                depth_map[y-1][x-1] + depth_map[y-1][x] + depth_map[y-1][x+1] +
                depth_map[y][x-1] + depth_map[y][x] + depth_map[y][x+1] +
                depth_map[y+1][x-1] + depth_map[y+1][x] + depth_map[y+1][x+1]
            ) / 9;
        }
    }

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            depth_map[y][x] = temp[y][x];
        }
    }
}

void normalize_depth_map() {
    int max_depth = 0;
    int min_depth = MAX_DEPTH;

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (depth_map[y][x] > max_depth) max_depth = depth_map[y][x];
            if (depth_map[y][x] < min_depth) min_depth = depth_map[y][x];
        }
    }

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            depth_map[y][x] = (depth_map[y][x] - min_depth) * 255 / (max_depth - min_depth);
        }
    }
}

void remove_ground_plane() {
    int ground_height = GROUND_THRESHOLD;
    
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (depth_map[y][x] < ground_height) {
                mask[y][x] = 0;
            } else {
                mask[y][x] = 1;
            }
        }
    }
}

void detect_high_depth_areas() {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (depth_map[y][x] > DEPTH_THRESHOLD) {
                mask[y][x] = 1;
            } else {
                mask[y][x] = 0;
            }
        }
    }
}

void assign_costs() {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (mask[y][x] == 0) {
                cost_map[y][x] = COST_SCALE * 100;
            } else {
                cost_map[y][x] = (int)((double)depth_map[y][x] / MAX_DEPTH * COST_SCALE);
            }
        }
    }
}

void find_lowest_cost_path() {
    int min_cost = 999999;
    int best_x = -1, best_y = -1;

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (cost_map[y][x] < min_cost) {
                min_cost = cost_map[y][x];
                best_x = x;
                best_y = y;
            }
        }
    }

    printf("Best path to (%d, %d) with cost %d\n", best_x, best_y, min_cost);
}

