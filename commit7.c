#include <webots/robot.h>  
// to control the robot
#include <webots/motor.h>  
// to control the motors
#include <webots/distance_sensor.h> 
// to use the distance sensors
#include <webots/gps.h> 
// to use the gps device 
#include <webots/light_sensor.h> 
// to use light sensors
#include <stdio.h> 
// standard input/output library 
#include <stdbool.h> 
// standard boolean library 
#include <math.h>
// math library 

// Define constants
#define STEP_INTERVAL 80
// Time step interval in milliseconds
#define MAX_MOTOR_SPEED 6.28
// Maximum speed of the robot's motors
#define OBSTACLE_THRESHOLD 100
// Threshold for detecting obstacles using proximity sensors
#define VELOCITY_ADJUSTMENT_FACTOR 0.05
// Adjustment factor for motor velocit changes
#define GRID_CELL_SIZE 0.1  
// Size of each grid cell for tracking exploration
#define GRID_DIMENSION 100   
// Dimension of the grid (creates a 100x100 grid)
#define MIN_EXPLORATION_DURATION 60.0 
// Minimum exploration time in seconds
#define NUM_PROXIMITY_SENSORS 8
// number of proximity sensors
#define MAX_LOGGED_POSITIONS 150
// Maximum number of positions to log during exploration

// Structs
typedef struct {
    double x_position; // x-coordinate position 
    double y_position; // y-coordinate position 
    double z_position; // z-coordinate position 
    double light_intensity;
} PositionAndLightIntensity;

// Global variables
PositionAndLightIntensity logged_positions[MAX_LOGGED_POSITIONS];
 // Array to store logged positions and light intensities
int num_logged_positions = 0;  // Number of logged positions
double starting_position[3] = {0.0, 0.0, 0.0};  // Initial position of the robot
bool explored_cells_grid[GRID_DIMENSION][GRID_DIMENSION] = {false};  // Grid to track explored cells
int total_explored_cells = 0;   // Total number of explored cells
double exploration_start_time = 0.0;    // Start time of the exploration

// Utility functions
double adjust_velocity(double current_speed, double target_speed) {
    // Gradually adjust the motor velocity to avoid abrupt changes
    return current_speed + VELOCITY_ADJUSTMENT_FACTOR * (target_speed - current_speed);
}

double calculate_distance(const double *point1, const double *point2) {
    // Calculate Euclidean distance between two points in 3D space
    return sqrt(pow(point2[0] - point1[0], 2) + pow(point2[1] - point1[1], 2) + pow(point2[2] - point1[2], 2));
}

// Convert world coordinates to grid coordinates
void convert_to_grid_coordinates(const double *position, int *grid_x, int *grid_y) {
    // Map real-world coordinates to grid cells for exploration tracking
    *grid_x = (int)((position[0] - starting_position[0]) / GRID_CELL_SIZE + GRID_DIMENSION / 2);
    *grid_y = (int)((position[1] - starting_position[1]) / GRID_CELL_SIZE + GRID_DIMENSION / 2);
    
    // Ensure coordinates are within bounds
    *grid_x = (*grid_x < 0) ? 0 : (*grid_x >= GRID_DIMENSION ? GRID_DIMENSION - 1 : *grid_x);
    *grid_y = (*grid_y < 0) ? 0 : (*grid_y >= GRID_DIMENSION ? GRID_DIMENSION - 1 : *grid_y);
     // Convert position to grid coordinates
}

// Mark current position as explored and return if it's a new cell
bool mark_position_as_explored(const double *position) {
    int grid_x, grid_y;
    convert_to_grid_coordinates(position, &grid_x, &grid_y);
    
    bool was_unexplored = !explored_cells_grid[grid_x][grid_y];  // Check if cell was unexplored
    if (was_unexplored) {
        explored_cells_grid[grid_x][grid_y] = true; // Mark cell as explored
        total_explored_cells++; // Increment explored cell count
    }
    return was_unexplored;
}

bool to_initialposition_again() {
    // Check if the robot has returned to its starting position
    const double *current_position = wb_gps_get_values(wb_robot_get_device("gps"));
     // Get current position from GPS
    double dist = calculate_distance(current_position, starting_position);
      // Calculate distance from start
    return dist < 0.5;  // Return true if close to start
}

bool is_exploration_complete() {
    // Check if exploration is complete
    double current_time = wb_robot_get_time();
     // Get the current simulation time
    double elapsed_time = current_time - exploration_start_time;
    // Calculate elapsed time since start
    
    if (elapsed_time > MIN_EXPLORATION_DURATION && to_initialposition_again()) {
        // Print exploration stats if minimum time has passed and robot is back at start
        printf("Exploration stats: Time: %.2f seconds, Cells explored: %d\n", 
               elapsed_time, total_explored_cells);
        return true; // Exploration is complete
    }
    return false; // Exploration is still ongoing
}

bool is_at_target(const double *current_position, const PositionAndLightIntensity *target_position) {
    // Check if the robot is at the target position
    double dist = calculate_distance(current_position, (double[]){target_position->x_position, target_position->y_position, target_position->z_position});
    return dist < 0.1;
}

int main(int argc, char **argv) {
    wb_robot_init(); // initialize the robot

    // Set up motors
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor"); // right motor 
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor"); // left motor
    wb_motor_set_position(right_motor, INFINITY);  // Set position for right motor to infinite for speed control
    wb_motor_set_position(left_motor, INFINITY); // Set position for left motor to infinite for speed control
    wb_motor_set_velocity(right_motor, 0.0); // Set initial velocity for right motor to 0
    wb_motor_set_velocity(left_motor, 0.0);  // Set initial velocity for left motor to 0

    // Set up proximity sensors
    WbDeviceTag proximity_sensors[NUM_PROXIMITY_SENSORS]; // Array for proximity sensor tags
    char proximity_sensor_name[50];  // Buffer for sensor names
    for (int i = 0; i < NUM_PROXIMITY_SENSORS; ++i) {
        sprintf(proximity_sensor_name, "ps%d", i); // Generate sensor name dynamically
        proximity_sensors[i] = wb_robot_get_device(proximity_sensor_name);  // Get sensor tag
        wb_distance_sensor_enable(proximity_sensors[i], STEP_INTERVAL);  // Enable sensor with specified time step
    }
    

    WbDeviceTag gps_device = wb_robot_get_device("gps");
    // Retrieve the GPS device by its name ("gps") and assign its tag to a variable 
    wb_gps_enable(gps_device, STEP_INTERVAL); // Enable the GPS device

    // Set up light sensors
    WbDeviceTag light_sensors[NUM_PROXIMITY_SENSORS];  // Declare an array to store tags for light sensors.
    char light_sensor_name[5]; // Declare a string to hold the name of each light sensor.
    for (int i = 0; i < NUM_PROXIMITY_SENSORS; i++) {
    // Iterate over the number of light sensors.
        sprintf(light_sensor_name, "ls%d", i);
         // Format the name of the light sensor
        light_sensors[i] = wb_robot_get_device(light_sensor_name);
        // Retrieve and store the device tag for each light sensor.
        wb_light_sensor_enable(light_sensors[i], STEP_INTERVAL);
        // Enable each light sensor with a specific sampling interval.
    }

    // Initialize motor speeds and exploration start time
    double right_motor_speed = MAX_MOTOR_SPEED;  // Set the initial speed for the right motor to the maximum value.
    double left_motor_speed = MAX_MOTOR_SPEED; // Set the initial speed for the left motor to the maximum value.

    // Capture initial position
    while (wb_robot_step(STEP_INTERVAL) != -1) {
    // Main simulation loop; runs until simulation ends or step returns -1.
        const double *initial_position = wb_gps_get_values(gps_device);
        // Get the robot's current GPS position.
        if (initial_position != NULL && !isnan(initial_position[0]) && !isnan(initial_position[1]) && !isnan(initial_position[2])) {
         // If the GPS position is valid
            starting_position[0] = initial_position[0]; // Store the x-coordinate of the starting position.
            starting_position[1] = initial_position[1]; // Store the y-coordinate of the starting position.
            starting_position[2] = initial_position[2]; // Store the z-coordinate of the starting position.
            exploration_start_time = wb_robot_get_time();  // Record the simulation time at the start of exploration.

            printf("Initial Position: (%.2f, %.2f, %.2f)\n--Starting Maze Exploration--\n", 
                   starting_position[0], starting_position[1], starting_position[2]); // prints the starting position 
            break;  // Exit the loop once the initial position is captured.
        }
    }

    // Main exploration loop
    bool is_exploring = true; // to indicate if the robot is exploring.
    PositionAndLightIntensity target_position = {0.0, 0.0, 0.0, 0.0}; // Initialize the target position with default values.
    bool target_position_reached = false; //  to indicate if the robot has reached the target position.

    while (wb_robot_step(STEP_INTERVAL) != -1) {
    // this while loop continues until the end of the simulation
        if (target_position_reached) {
         // If the robot has reached the target position:
            wb_motor_set_velocity(left_motor, 0.0);
            // stop the left motor
            wb_motor_set_velocity(right_motor, 0.0);
            // stop the right motor
            continue; // Skip the rest of the loop and wait.
        }

        if (is_exploring) {
         // If the robot is in the exploration phase
            if (is_exploration_complete()) {
            // Check if the exploration phase is complete.
                is_exploring = false; // sets flag to false indicating that the exploration is over
                printf("\n--Maze Exploration Complete--\nFinding position with highest light intensity...\n");
               // message to alert exploration completion.
               
                // Find the position with the highest light intensity
                double highest_light_intensity = -1.0;  // Initialize the highest light intensity value.
                for (int i = 0; i < num_logged_positions; i++) { 
                // Iterate through all logged positions.
                    if (logged_positions[i].light_intensity > highest_light_intensity) {
                     // If the current position has a higher light intensity
                        highest_light_intensity = logged_positions[i].light_intensity;
                         // Update the highest intensity.
                        target_position = logged_positions[i];
                        // Update the target position.
                    }
                }
                
                printf("Moving to brightest position: (%.2f, %.2f, %.2f) with intensity %.2f\n",
                       target_position.x_position, target_position.y_position, target_position.z_position, target_position.light_intensity);
                        // Print details of the brightest position.
                continue;  // Skip the rest of the loop.
            }

            // Get proximity sensor readings
            double proximity_values[NUM_PROXIMITY_SENSORS];
             // Array to store readings from proximity sensors.
            for (int i = 0; i < NUM_PROXIMITY_SENSORS; ++i) {
             // Iterate over all proximity sensors.
                proximity_values[i] = wb_distance_sensor_get_value(proximity_sensors[i]);
             // Get the reading for each sensor.
            }


            // Get light sensor readings and current position
            double light_sum = 0.0;  // Variable to accumulate light sensor readings.
            for (int i = 0; i < NUM_PROXIMITY_SENSORS; i++) {
             // Iterate through all light sensors.
                light_sum += wb_light_sensor_get_value(light_sensors[i]);
             // Sum up the readings from all light sensors.
            }
            
            double average_light_value = light_sum / NUM_PROXIMITY_SENSORS;
            // Calculate the average light intensity.

            const double *current_position = wb_gps_get_values(gps_device);
             // Get the robot's current GPS position.

            
            // Record position and light intensity
            if (num_logged_positions < MAX_LOGGED_POSITIONS && mark_position_as_explored(current_position)) {
            // If the maximum log limit isn't reached and the position is newly explored
                logged_positions[num_logged_positions].x_position = current_position[0];
                // Store x-coordinate of the current position.
                logged_positions[num_logged_positions].y_position = current_position[1];
                // Store y-coordinate of the current position.
                logged_positions[num_logged_positions].z_position = current_position[2];
                // Store z-coordinate of the current position.

                logged_positions[num_logged_positions].light_intensity = average_light_value;
                 // Store the average light intensity.

                num_logged_positions++;
                // Increment the logged position counter.
                
                if (num_logged_positions % 50 == 0) {
                    printf("Exploration progress: %d positions logged\n", num_logged_positions);
                     // Every 50 logged positions, print progress.
                }
            }

            // Wall-following logic for exploration
            bool is_front_wall = proximity_values[7] > OBSTACLE_THRESHOLD;  // Check if there's an obstacle in front.
            bool is_left_wall = proximity_values[1] > OBSTACLE_THRESHOLD; // Check if there's an obstacle to the left.

            if (is_front_wall) {
            // If a front obstacle is detected
                left_motor_speed = MAX_MOTOR_SPEED;
                // Turn by rotating the left motor forward.
                right_motor_speed = -MAX_MOTOR_SPEED;
                // Rotate the right motor backward.

            } else if (!is_left_wall) {
             // If no obstacle on the left
                left_motor_speed = MAX_MOTOR_SPEED / 8;
                // Slow down the left motor.
                right_motor_speed = MAX_MOTOR_SPEED;
                // Keep the right motor at full speed.
            } else {
            // Otherwise, move forward.
                left_motor_speed = MAX_MOTOR_SPEED;  // Set left motor to full speed.
                right_motor_speed = MAX_MOTOR_SPEED;  // Set right motor to full speed.
            }
        } else {
            // Navigation to brightest point phase
            const double *current_position = wb_gps_get_values(gps_device);
             // Get the robot's current GPS position.

            
            if (is_at_target(current_position, &target_position)) {
              // If the robot has reached the target position:

                wb_motor_set_velocity(left_motor, 0.0);  // Stop the left motor.
                wb_motor_set_velocity(right_motor, 0.0);  // Stop the right motor.
                target_position_reached = true;  // Mark the target position as reached.
                
                printf("===Reached position with highest light intensity===\n");
                printf("Final Position: (%.2f, %.2f, %.2f)\n", 
                       target_position.x_position, target_position.y_position, target_position.z_position);
                printf("Light Intensity: %.2f\n", target_position.light_intensity); //alert message
                continue;  // Skip the rest of the loop
            }

            // Wall-following logic for navigation
            double proximity_values[NUM_PROXIMITY_SENSORS];
             // Array to store readings from proximity sensors.
            for (int i = 0; i < NUM_PROXIMITY_SENSORS; ++i) {
             // Iterate over all proximity sensors.
                proximity_values[i] = wb_distance_sensor_get_value(proximity_sensors[i]);
                 // Get the reading for each sensor.
            }

            bool is_front_wall = proximity_values[7] > OBSTACLE_THRESHOLD;
             // Check if there's an obstacle in front.

            bool is_left_wall = proximity_values[1] > OBSTACLE_THRESHOLD;
             // Check if there's an obstacle to the left.


            if (is_front_wall) {
             // If a front obstacle is detected
                left_motor_speed = MAX_MOTOR_SPEED;  // Turn by rotating the left motor forward
                right_motor_speed = -MAX_MOTOR_SPEED; // turn by rotating the right motor 
                
            } else if (!is_left_wall) { 
              // If there is no wall on the left, reduce the left motor speed to turn slightly left.
                left_motor_speed = MAX_MOTOR_SPEED / 8;
                right_motor_speed = MAX_MOTOR_SPEED;
                
            } else {
           // If there are no obstacles, maintain maximum speed for both motors to move forward.
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
