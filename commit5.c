#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Define constants
#define STEP_INTERVAL 64
#define MAX_MOTOR_SPEED 6.28
#define NUM_PROXIMITY_SENSORS 8
#define OBSTACLE_THRESHOLD 80.0
#define TURN_THRESHOLD 50.0
#define CLOSE_DISTANCE_THRESHOLD 0.1
#define MAX_LOGGED_POSITIONS 5000
#define LIGHT_SENSOR_THRESHOLD 50.0
#define VELOCITY_ADJUSTMENT_FACTOR 0.07
#define GRID_CELL_SIZE 0.1  // Size of each grid cell for tracking exploration
#define GRID_DIMENSION 100   // Dimension of the grid (creates a 100x100 grid)
#define MIN_EXPLORATION_DURATION 60.0 // Minimum exploration time in seconds

// Structs
typedef struct {
    double x_position;
    double y_position;
    double z_position;
    double light_intensity;
} PositionAndLightIntensity;

// Global variables
PositionAndLightIntensity logged_positions[MAX_LOGGED_POSITIONS];
int num_logged_positions = 0;
double starting_position[3] = {0.0, 0.0, 0.0};
bool explored_cells_grid[GRID_DIMENSION][GRID_DIMENSION] = {false};  // Track explored areas
int total_explored_cells = 0;
double exploration_start_time = 0.0;

// Utility functions
double adjust_velocity(double current_speed, double target_speed) {
    return current_speed + VELOCITY_ADJUSTMENT_FACTOR * (target_speed - current_speed);
}

double calculate_distance(const double *point1, const double *point2) {
    return sqrt(pow(point2[0] - point1[0], 2) + pow(point2[1] - point1[1], 2) + pow(point2[2] - point1[2], 2));
}

// Convert world coordinates to grid coordinates
void convert_to_grid_coordinates(const double *position, int *grid_x, int *grid_y) {
    *grid_x = (int)((position[0] - starting_position[0]) / GRID_CELL_SIZE + GRID_DIMENSION / 2);
    *grid_y = (int)((position[1] - starting_position[1]) / GRID_CELL_SIZE + GRID_DIMENSION / 2);
    
    // Ensure coordinates are within bounds
    *grid_x = (*grid_x < 0) ? 0 : (*grid_x >= GRID_DIMENSION ? GRID_DIMENSION - 1 : *grid_x);
    *grid_y = (*grid_y < 0) ? 0 : (*grid_y >= GRID_DIMENSION ? GRID_DIMENSION - 1 : *grid_y);
}

// Mark current position as explored and return if it's a new cell
bool mark_position_as_explored(const double *position) {
    int grid_x, grid_y;
    convert_to_grid_coordinates(position, &grid_x, &grid_y);
    
    bool was_unexplored = !explored_cells_grid[grid_x][grid_y];
    if (was_unexplored) {
        explored_cells_grid[grid_x][grid_y] = true;
        total_explored_cells++;
    }
    return was_unexplored;
}

bool is_back_to_start() {
    const double *current_position = wb_gps_get_values(wb_robot_get_device("gps"));
    double dist = calculate_distance(current_position, starting_position);
    return dist < 0.5;
}

bool is_exploration_complete() {
    double current_time = wb_robot_get_time();
    double elapsed_time = current_time - exploration_start_time;
    
    // Check if minimum exploration time has passed and we're back at start
    if (elapsed_time > MIN_EXPLORATION_DURATION && is_back_to_start()) {
        printf("Exploration stats: Time: %.2f seconds, Cells explored: %d\n", 
               elapsed_time, total_explored_cells);
        return true;
    }
    return false;
}

bool is_at_target(const double *current_position, const PositionAndLightIntensity *target_position) {
    double dist = calculate_distance(current_position, (double[]){target_position->x_position, target_position->y_position, target_position->z_position});
    return dist < 0.1;
}

int main(int argc, char **argv) {
    wb_robot_init();

    // Set up motors
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);
    wb_motor_set_velocity(left_motor, 0.0);

    // Set up proximity sensors
    WbDeviceTag proximity_sensors[NUM_PROXIMITY_SENSORS];
    char proximity_sensor_name[50];
    for (int i = 0; i < NUM_PROXIMITY_SENSORS; ++i) {
        sprintf(proximity_sensor_name, "ps%d", i);
        proximity_sensors[i] = wb_robot_get_device(proximity_sensor_name);
        wb_distance_sensor_enable(proximity_sensors[i], STEP_INTERVAL);
    }

    WbDeviceTag gps_device = wb_robot_get_device("gps");
    wb_gps_enable(gps_device, STEP_INTERVAL);

    // Set up light sensors
    WbDeviceTag light_sensors[NUM_PROXIMITY_SENSORS];
    char light_sensor_name[5];
    for (int i = 0; i < NUM_PROXIMITY_SENSORS; i++) {
        sprintf(light_sensor_name, "ls%d", i);
        light_sensors[i] = wb_robot_get_device(light_sensor_name);
        wb_light_sensor_enable(light_sensors[i], STEP_INTERVAL);
    }

    // Initialize motor speeds and exploration start time
    double right_motor_speed = MAX_MOTOR_SPEED;
    double left_motor_speed = MAX_MOTOR_SPEED;

    // Capture initial position
    while (wb_robot_step(STEP_INTERVAL) != -1) {
        const double *initial_position = wb_gps_get_values(gps_device);
        if (initial_position != NULL && !isnan(initial_position[0]) && !isnan(initial_position[1]) && !isnan(initial_position[2])) {
            starting_position[0] = initial_position[0];
            starting_position[1] = initial_position[1];
            starting_position[2] = initial_position[2];
            exploration_start_time = wb_robot_get_time();
            printf("Initial Position: (%.2f, %.2f, %.2f)\n--Starting Maze Exploration--\n", 
                   starting_position[0], starting_position[1], starting_position[2]);
            break;
        }
    }

    // Main exploration loop
    bool is_exploring = true;
    PositionAndLightIntensity target_position = {0.0, 0.0, 0.0, 0.0};
    bool target_position_reached = false;

    while (wb_robot_step(STEP_INTERVAL) != -1) {
        if (target_position_reached) {
            // Stop the robot
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            continue;
        }

        if (is_exploring) {
            // Check if exploration is complete
            if (is_exploration_complete()) {
                is_exploring = false;
                printf("\n--Maze Exploration Complete--\nFinding position with highest light intensity...\n");
                
                // Find the position with the highest light intensity
                double highest_light_intensity = -1.0;
                for (int i = 0; i < num_logged_positions; i++) {
                    if (logged_positions[i].light_intensity > highest_light_intensity) {
                        highest_light_intensity = logged_positions[i].light_intensity;
                        target_position = logged_positions[i];
                    }
                }
                
                printf("Moving to brightest position: (%.2f, %.2f, %.2f) with intensity %.2f\n",
                       target_position.x_position, target_position.y_position, target_position.z_position, target_position.light_intensity);
                continue;
            }

            // Get proximity sensor readings
            double proximity_values[NUM_PROXIMITY_SENSORS];
            for (int i = 0; i < NUM_PROXIMITY_SENSORS; ++i) {
                proximity_values[i] = wb_distance_sensor_get_value(proximity_sensors[i]);
            }

            // Get light sensor readings and current position
            double light_sum = 0.0;
            for (int i = 0; i < NUM_PROXIMITY_SENSORS; i++) {
                light_sum += wb_light_sensor_get_value(light_sensors[i]);
            }
            double average_light_value = light_sum / NUM_PROXIMITY_SENSORS;

            const double *current_position = wb_gps_get_values(gps_device);
            
            // Record position and light intensity
            if (num_logged_positions < MAX_LOGGED_POSITIONS && mark_position_as_explored(current_position)) {
                logged_positions[num_logged_positions].x_position = current_position[0];
                logged_positions[num_logged_positions].y_position = current_position[1];
                logged_positions[num_logged_positions].z_position = current_position[2];
                logged_positions[num_logged_positions].light_intensity = average_light_value;
                num_logged_positions++;
                
                if (num_logged_positions % 50 == 0) {
                    printf("Exploration progress: %d positions logged\n", num_logged_positions);
                }
            }

            // Wall-following logic for exploration
            bool is_front_wall = proximity_values[7] > OBSTACLE_THRESHOLD;
            bool is_left_wall = proximity_values[1] > OBSTACLE_THRESHOLD;

            if (is_front_wall) {
                left_motor_speed = MAX_MOTOR_SPEED;
                right_motor_speed = -MAX_MOTOR_SPEED;
            } else if (!is_left_wall) {
                left_motor_speed = MAX_MOTOR_SPEED / 8;
                right_motor_speed = MAX_MOTOR_SPEED;
            } else {
                left_motor_speed = MAX_MOTOR_SPEED;
                right_motor_speed = MAX_MOTOR_SPEED;
            }
        } else {
            // Navigation to brightest point phase
            const double *current_position = wb_gps_get_values(gps_device);
            
            if (is_at_target(current_position, &target_position)) {
                // Stop the robot completely
                wb_motor_set_velocity(left_motor, 0.0);
                wb_motor_set_velocity(right_motor, 0.0);
                target_position_reached = true;
                
                printf("===Reached position with highest light intensity===\n");
                printf("Final Position: (%.2f, %.2f, %.2f)\n", 
                       target_position.x_position, target_position.y_position, target_position.z_position);
                printf("Light Intensity: %.2f\n", target_position.light_intensity);
                continue;
            }

            // Wall-following logic for navigation
            double proximity_values[NUM_PROXIMITY_SENSORS];
            for (int i = 0; i < NUM_PROXIMITY_SENSORS; ++i) {
                proximity_values[i] = wb_distance_sensor_get_value(proximity_sensors[i]);
            }

            bool is_front_wall = proximity_values[7] > OBSTACLE_THRESHOLD;
            bool is_left_wall = proximity_values[1] > OBSTACLE_THRESHOLD;

            if (is_front_wall) {
                left_motor_speed = MAX_MOTOR_SPEED;
                right_motor_speed = -MAX_MOTOR_SPEED;
            } else if (!is_left_wall) {
                left_motor_speed = MAX_MOTOR_SPEED / 8;
                right_motor_speed = MAX_MOTOR_SPEED;
            } else {
                left_motor_speed = MAX_MOTOR_SPEED;
                right_motor_speed = MAX_MOTOR_SPEED;
            }
        }

        // Update motor speeds
        wb_motor_set_velocity(left_motor, left_motor_speed);
        wb_motor_set_velocity(right_motor, right_motor_speed);
    }

    wb_robot_cleanup();
    return 0;
}
