/*
 * File:         irobot_create.c
 * Date:         21 Dec 2010
 * Description:  Default controller of the iRobot Create robot
 * Author:       fabien.rohrer@cyberbotics.com
 * Modifications:
 */

/* include headers */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/camera.h>

/* device stuff */
static WbDeviceTag camera; 

#define BUMPERS_NUMBER 2
#define BUMPER_LEFT 0
#define BUMPER_RIGHT 1
static WbDeviceTag bumpers[BUMPERS_NUMBER];
static const char *bumpers_name[BUMPERS_NUMBER] = {
  "bumper_left",
  "bumper_right"
};

#define DISTANCE_SENSORS_NUMBER 4
#define DISTANCE_SENSOR_LEFT 0
#define DISTANCE_SENSOR_FRONT_LEFT 1
#define DISTANCE_SENSOR_FRONT_RIGHT 2
#define DISTANCE_SENSOR_RIGHT 3
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_name[DISTANCE_SENSORS_NUMBER] = {
  "dist_left",
  "dist_front_left",
  "dist_front_right",
  "dist_right"
};

#define LEDS_NUMBER 3
#define LED_ON 0
#define LED_PLAY 1
#define LED_STEP 2
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_name[LEDS_NUMBER] = {
  "led_on",
  "led_play",
  "led_step"
};

static WbDeviceTag receiver;
static const char *receiver_name = "receiver";

/* Misc Stuff */
#define MAX_SPEED 100
#define NULL_SPEED 0
#define HALF_SPEED 50
#define MIN_SPEED -100

#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

/* helper functions */
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int) wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_devices() {
  int i;

  receiver = wb_robot_get_device(receiver_name);
  wb_receiver_enable(receiver, get_time_step());

  for (i=0; i < LEDS_NUMBER; i++) {
    leds[i] = wb_robot_get_device(leds_name[i]);
  }

  for (i=0; i < BUMPERS_NUMBER; i++) {
    bumpers[i] = wb_robot_get_device(bumpers_name[i]);
    wb_touch_sensor_enable(bumpers[i], get_time_step());
  }

  for (i=0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_name[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  
}

static bool is_there_a_collision_at_left() {
  return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0);
}

static bool is_there_a_collision_at_right() {
  return (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}

static void fflush_ir_receiver() {
  while (wb_receiver_get_queue_length(receiver) > 0)
    wb_receiver_next_packet(receiver);
}

static bool is_there_a_virtual_wall() {
  return (wb_receiver_get_queue_length(receiver) > 0);
}

static bool is_there_a_distance_at_left() {
  return !((wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_LEFT]) > 0.0) &&
           (wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT_LEFT]) > 0.0));
}

static bool is_there_a_distance_at_right() {
  return !((wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_RIGHT]) > 0.0) &&
           (wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT_RIGHT]) > 0.0));
}

static bool is_there_a_distance_at_front() {
  return !((wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT_LEFT]) > 0.0) &&
           (wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT_RIGHT]) > 0.0));
}

static void go_forward() {
  wb_differential_wheels_set_speed(MAX_SPEED, MAX_SPEED);
}

static void go_backward() {
  wb_differential_wheels_set_speed(-HALF_SPEED, -HALF_SPEED);
}

static void stop() {
  wb_differential_wheels_set_speed(-NULL_SPEED, -NULL_SPEED);
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  
  do 
  {
    step();
  } 
  while((start_time + sec) > wb_robot_get_time());
}

static double randdouble() {
  return rand() / ((double)RAND_MAX + 1);
}

static void turn(double angle) {
  stop();
  
  wb_differential_wheels_enable_encoders(get_time_step());
  wb_differential_wheels_set_encoders(0.0, 0.0);
  
  step();
  
  double neg = (angle < 0.0) ? -1.0 : 1.0;
  
  wb_differential_wheels_set_speed(neg*HALF_SPEED, -neg*HALF_SPEED);
  
  double orientation;
  
  do 
  {
    double l = wb_differential_wheels_get_left_encoder();
    double r = wb_differential_wheels_get_right_encoder();
    
    double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
    double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
    
    orientation = neg * (dl - dr) / AXLE_LENGTH; // delta orientation in radian
    
    step();
  } 
  while (orientation < neg*angle);
  
  stop();
  
  wb_differential_wheels_disable_encoders();
  
  step();
}

/* main */
int main(int argc, char **argv)
{
  wb_robot_init();
  
  printf("Default controller of the iRobot Create robot started...\n");
  
  init_devices();
  
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,get_time_step());
  
  srand(time(NULL));
  
  wb_led_set(leds[LED_ON], true);
  
  passive_wait(0.5);

  while (true) {
    if (is_there_a_virtual_wall()) 
    {    
      printf("Virtual wall detected\n");
      turn(M_PI);
    } 
    else if (is_there_a_collision_at_left()) 
    {    
      printf("Left collision detected\n");
      go_backward();
      passive_wait(0.5);
      turn(M_PI * randdouble());
    } 
    else if (is_there_a_collision_at_right()) 
    {    
      printf("Right collision detected\n");
      go_backward();
      passive_wait(0.5);
      turn(-M_PI * randdouble());
    }
    /*else if (is_there_a_distance_at_left()) 
    {    
      printf("Left distance detected\n");
      go_backward();
      passive_wait(0.5);
      turn(M_PI * randdouble());
    }
    else if (is_there_a_distance_at_right()) 
    {    
      printf("Right distance detected\n");
      go_backward();
      passive_wait(0.5);
      turn(-M_PI * randdouble());
    } */
    else if (is_there_a_distance_at_front()) 
    {
    
      printf("Front distance detected\n");
      //go_backward();
      
      if (is_there_a_distance_at_left())
      {
        if (is_there_a_distance_at_right())
        {
          printf("Going backwards\n");
          go_backward();
        }
        else
        {
          printf("Turning to the right\n");
          turn(M_PI / 2);
        }
      }
      else
      {
        printf("Turning to the left\n");
        turn(-M_PI / 2);
      }
    
      passive_wait(0.5);
    } 
    else 
    {    
      go_forward();
    }
    
    wb_camera_get_image(camera);
    fflush_ir_receiver();
    step();
  };
  
  return EXIT_SUCCESS;
}

