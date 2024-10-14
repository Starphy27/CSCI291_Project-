#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define MAX_LIGHT_SOURCES 9

// Function prototypes
void move_forward();
void turn_left();
void turn_right();
void move_backward();
void stop();
void read_sensors();
void follow_left_wall();
void measure_light_intensity();
void navigate_to_highest_intensity_light();

// Global variables
WbDeviceTag left_motor, right_motor;
WbDeviceTag distance_sensors;
WbDeviceTag light_sensors[MAX_LIGHT_SOURCES];
double light_intensities[MAX_LIGHT_SOURCES];
int light_sources_count = 0;
int current_light_source = 0;

int main(int argc, char **argv) {
  wb_robot_init();

  // Initialize motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Initialize sensors
  distance_sensors = wb_robot_get_device("ds_left");
  distance_sensors = wb_robot_get_device("ds_front");
  distance_sensors = wb_robot_get_device("ds_right");
  for (int i = 0; i < 3; i++) {
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
  }

  for (int i = 0; i < MAX_LIGHT_SOURCES; i++) {
    char sensor_name;
    sprintf(sensor_name, "ls_%d", i);
    light_sensors[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    follow_left_wall();
    measure_light_intensity();
  }

  navigate_to_highest_intensity_light();

  wb_robot_cleanup();
  return 0;
}

void move_forward() {
  wb_motor_set_velocity(left_motor, 1.0);
  wb_motor_set_velocity(right_motor, 1.0);
}

void turn_left() {
  wb_motor_set_velocity(left_motor, -1.0);
  wb_motor_set_velocity(right_motor, 1.0);
}

void turn_right() {
  wb_motor_set_velocity(left_motor, 1.0);
  wb_motor_set_velocity(right_motor, -1.0);
}

void move_backward() {
  wb_motor_set_velocity(left_motor, -1.0);
  wb_motor_set_velocity(right_motor, -1.0);
}

void stop() {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

void read_sensors() {
  for (int i = 0; i < 3; i++) {
    double value = wb_distance_sensor_get_value(distance_sensors[i]);
    printf("Distance sensor %d: %f\n", i, value);
  }
  for (int i = 0; i < light_sources_count; i++) {
    light_intensities[i] = wb_light_sensor_get_value(light_sensors[i]);
    printf("Light sensor %d: %f\n", i, light_intensities[i]);
  }
}

void follow_left_wall() {
  double left_value = wb_distance_sensor_get_value(distance_sensors);
  double front_value = wb_distance_sensor_get_value(distance_sensors);
  double right_value = wb_distance_sensor_get_value(distance_sensors);

  if (front_value > 100.0) {
    turn_right();
  } else if (left_value < 100.0) {
    move_forward();
  } else {
    turn_left();
  }
}

void measure_light_intensity() {
  double current_intensity = wb_light_sensor_get_value(light_sensors[current_light_source]);
  light_intensities[current_light_source] = current_intensity;
  printf("Measured light intensity at source %d: %f\n", current_light_source, current_intensity);
  current_light_source++;
  if (current_light_source >= MAX_LIGHT_SOURCES) {
    current_light_source = 0;
  }
}

void navigate_to_highest_intensity_light() {
  int max_index = 0;
  for (int i = 1; i < light_sources_count; i++) {
    if (light_intensities[i] > light_intensities[max_index]) {
      max_index = i;
    }
  }
  printf("Navigating to light source %d with intensity %f\n", max_index, light_intensities[max_index]);
  // Implement navigation to the selected light source
  stop();
}