#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <float.h>  // Add this for INFINITY
#include <stdbool.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28
#define LIGHT_THRESHOLD 0.1  // Threshold for comparing light intensities
#define MIN_EXPLORATION_TIME 100  // Minimum time steps before checking for cycle completion

// Sensor and motor tags
WbDeviceTag left_motor, right_motor;
WbDeviceTag ps[8];  // 8 proximity sensors
WbDeviceTag light_sensor;
bool first_move = true;
double max_light_intensity = 0.0;
double current_light_intensity = 0.0;
double initial_light_intensity = 0.0;
int time_steps = 0;
bool cycle_completed = false;

// Initialize devices
void initialize_devices() {
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    if (left_motor == 0 || right_motor == 0) {
        printf("Error: Motors not found!\n");
        return;
    }
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    
    // Initialize proximity sensors
    for (int i = 0; i < 8; i++) {
        char sensor_name[4];
        sprintf(sensor_name, "ps%d", i);
        ps[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }
    
    // Initialize light sensor
    light_sensor = wb_robot_get_device("ls0");
    if (light_sensor == 0) {
        printf("Error: Light sensor not found!\n");
        return;
    }
    wb_light_sensor_enable(light_sensor, TIME_STEP);
}

// Check if the robot has completed a cycle
bool check_cycle_completion() {
    // Only start checking after minimum exploration time
    if (time_steps < MIN_EXPLORATION_TIME) {
        return false;
    }
    
    // Check if current light intensity is close to initial intensity
    double difference = fabs(current_light_intensity - initial_light_intensity);
    if (difference < LIGHT_THRESHOLD) {
        printf("Cycle completed! Initial: %f, Current: %f\n", 
               initial_light_intensity, current_light_intensity);
        return true;
    }
    return false;
}

// Process sensors and control the robot
void process_sensors_and_control() {
    // First move: Robot starts moving forward and records initial light intensity
    if (first_move) {
        initial_light_intensity = wb_light_sensor_get_value(light_sensor);
        printf("Initial light intensity: %f\n", initial_light_intensity);
        wb_motor_set_velocity(left_motor, 0.5 * MAX_SPEED);
        wb_motor_set_velocity(right_motor, 0.5 * MAX_SPEED);
        first_move = false;
        return;
    }
    
    // If cycle is completed, stop the robot
    if (cycle_completed) {
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        return;
    }
    
    // Get proximity sensor values
    double ps_values[8];
    for (int i = 0; i < 8; i++) {
        ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    }
    
    // Get current light intensity
    current_light_intensity = wb_light_sensor_get_value(light_sensor);
    
    // Track the brightest light intensity during exploration
    if (current_light_intensity > max_light_intensity) {
        max_light_intensity = current_light_intensity;
        printf("New brightest light intensity recorded: %f\n", max_light_intensity);
    }
    
    // Check if cycle is completed
    if (check_cycle_completion()) {
        cycle_completed = true;
        printf("Stopping robot - cycle completed\n");
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        return;
    }
    
    // Movement logic
    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;
    bool front_obstacle = ps_values[0] > 80.0 || ps_values[7] > 80.0;
    bool side_left = ps_values[5] > 80.0;
    bool side_right = ps_values[2] > 80.0;
    
    if (front_obstacle) {
        if (side_left && side_right) {
            left_speed = -0.5 * MAX_SPEED;
            right_speed = -0.5 * MAX_SPEED;
        } else if (side_left) {
            left_speed = MAX_SPEED;
            right_speed = 0.1 * MAX_SPEED;
        } else if (side_right) {
            left_speed = 0.1 * MAX_SPEED;
            right_speed = MAX_SPEED;
        } else {
            left_speed = MAX_SPEED;
            right_speed = -0.1 * MAX_SPEED;
        }
    } else {
        if (side_left && !side_right) {
            left_speed = 0.1 * MAX_SPEED;
            right_speed = MAX_SPEED;
        } else if (!side_left && !side_right) {
            left_speed = 0.5 * MAX_SPEED;
            right_speed = 0.5 * MAX_SPEED;
        } else if (!side_left && side_right) {
            left_speed = 0.1 * MAX_SPEED;
            right_speed = MAX_SPEED;
        }
    }
    
    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    
    // Increment time steps
    time_steps++;
}

int main() {
    wb_robot_init();
    initialize_devices();
    while (wb_robot_step(TIME_STEP) != -1) {
        process_sensors_and_control();
    }
    wb_robot_cleanup();
    return 0;
}


