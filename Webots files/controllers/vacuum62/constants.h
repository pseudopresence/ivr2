/* 
 * File:   constants.h
 * Author: s1149322
 *
 * Created on 22 November 2011, 14:47
 */

#ifndef CONSTANTS_H
#define	CONSTANTS_H

/* Misc Stuff */
#define MAX_SPEED (500) /* Units: mm/s */
#define NULL_SPEED (0)
#define HALF_SPEED ((MAX_SPEED)/2.0)
#define MIN_SPEED (-(MAX_SPEED))

#define ROBOT_RADIUS 0.1675
#define ROBOT_DIAMETER (2 * ROBOT_RADIUS)
#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

/* Room Size */
#define MAX_ROOM_HEIGHT (6 - ROBOT_DIAMETER)
#define MAX_ROOM_WIDTH (6 - ROBOT_DIAMETER)

/* Precision constants */
#define TURN_PRECISION 0.03
#define ANGLE_PRECISION 0.001
#define TARGET_PRECISION 0.1

#endif	/* CONSTANTS_H */

