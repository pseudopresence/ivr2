/* 
 * File:   constants.h
 * Author: s1149322
 *
 * Created on 22 November 2011, 14:47
 */

#ifndef CONSTANTS_H
#define	CONSTANTS_H

/* Misc Stuff */
#define MAX_SPEED 500 /* Units: mm/s */
#define NULL_SPEED 0
#define HALF_SPEED ((MAX_SPEED)/2.0)
#define MIN_SPEED (-(MAX_SPEED))

#define ROBOT_RADIUS 0.1675
#define ROBOT_DIAMETER (2 * ROBOT_RADIUS)
#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

/* Room Size */
#define WALL_OFFSET 0.2
#define ORIGINAL_ROOM_SIZE 6
#define MAX_ROOM_SIZE (ORIGINAL_ROOM_SIZE - 2 * WALL_OFFSET)
#define NAV_ROOM_SIZE (ORIGINAL_ROOM_SIZE - WALL_OFFSET - ROBOT_RADIUS)

/* Precision constants */
#define TURN_PRECISION 0.03
#define ANGLE_PRECISION 0.001
#define TARGET_PRECISION 0.02

#define TARGET_TIMEOUT 3
#define DANGER_DISTANCE 0.15

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

#define DISTANCE_SENSORS_NUMBER 5
#define DISTANCE_SENSOR_LEFT 0
#define DISTANCE_SENSOR_FRONT 1
#define DISTANCE_SENSOR_RIGHT 2
#define DISTANCE_SENSOR_DIAG_LEFT 3
#define DISTANCE_SENSOR_DIAG_RIGHT 4
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_name[DISTANCE_SENSORS_NUMBER] = {
  "dist_left",
  "dist_front",
  "dist_right",
  "dist_diagleft",
  "dist_diagright"
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

#endif	/* CONSTANTS_H */

