/* 
 * File:   robot.h
 * Author: s1151960
 *
 * Created on 23 November 2011, 14:36
 */

#ifndef ROBOT_H
#define	ROBOT_H

#include <time.h>
#include <math.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/camera.h>

#include "util.h"
#include "vec2.h"
#include "pid.h"
#include "navigation.h"
#include "constants.h"

enum Behaviour {
    REST, SWEEPING, HOMING
};

/* helper functions */
static double forceFromDist(double const _dist) {
    return 1 - smootherstep(0.05, 0.1, _dist);
}

static double speedFromControlParam(double const _param) {
    if (_param < 0) {
        return 1;
    } else {
        return 1 - 2 * _param;
    }
}

Vec2 speedsFromControlParam(double const _l) {
    return Vec2(speedFromControlParam(-_l), speedFromControlParam(_l));
}

static int get_time_step() {
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int) wb_robot_get_basic_time_step();
    return time_step;
}

static void init_devices() {
    int i;

    for (i = 0; i < LEDS_NUMBER; i++) {
        leds[i] = wb_robot_get_device(leds_name[i]);
    }

    for (i = 0; i < BUMPERS_NUMBER; i++) {
        bumpers[i] = wb_robot_get_device(bumpers_name[i]);
        wb_touch_sensor_enable(bumpers[i], get_time_step());
    }

    for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
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

/*static bool is_there_a_distance_at_left() {
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
}*/

struct Pose {

    Pose(Vec2 const _pos, double const _dir) :
    m_pos(_pos),
    m_dir(_dir) {
    }

    void Update(double const _turn, double const _distance) {
        //m_dir = ceilf((m_dir + _turn) / ANGLE_PRECISION) * ANGLE_PRECISION;
        m_dir = wrap(m_dir + _turn, 0, 2 * M_PI);
        // TODO do we get a better estimate using the previous direction to update position, or the new one? Or an average?
        m_pos += Vec2::FromDirLen(m_dir, _distance);

        printf("X: %03.3lf, Y: %03.3lf, O: %03.3lf\n", m_pos.m_x, m_pos.m_y, m_dir * 180 / M_PI);
    }

    Vec2 m_pos;
    double m_dir;
};

class Robot {
private:
    Pose m_pose;
    NavigationState m_navState;
    Behaviour m_behaviour;
    Navigation m_nav;

public:

    Robot(Vec2 const _pos, double _offset, double _dir) :
    m_pose(_pos, _dir),
    m_navState(UP, _offset, _pos),
    m_behaviour(REST),
    m_nav() {
    }

    void Init() {
        wb_robot_init();
        wb_differential_wheels_enable_encoders(get_time_step());
    }

    void Shutdown() {
        wb_differential_wheels_disable_encoders();
    }

    void UpdateOdometry() {
        double encl = wb_differential_wheels_get_left_encoder();
        double encr = wb_differential_wheels_get_right_encoder();
        double dl = encl / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
        double dr = encr / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter

        double const TURN_HACK_FACTOR = 0.97;
        double const DIST_HACK_FACTOR = 1; //0.97;
        double turn = TURN_HACK_FACTOR * (dr - dl) / AXLE_LENGTH; // delta orientation in radian
        double distance = DIST_HACK_FACTOR * (dr + dl) / 2;

        m_pose.Update(turn, distance);
    }

    bool CorrectOdometry() {
        double const distFront = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT]);

        double distExpected = NAV_ROOM_SIZE;
        double targetOffset = 0;

        switch (m_navState.m_dir) {
            case UP:
                distExpected = (distExpected - m_pose.m_pos.m_x) / cos(m_pose.m_dir);
                targetOffset = m_navState.m_targetPos.m_x - m_pose.m_pos.m_x;
                break;
            case LEFT:
                distExpected = (distExpected - m_pose.m_pos.m_y) / sin(m_pose.m_dir);
                targetOffset = m_navState.m_targetPos.m_y - m_pose.m_pos.m_y;
                break;
            case DOWN:
                distExpected = (distExpected - MAX_ROOM_SIZE + m_pose.m_pos.m_x) / -cos(m_pose.m_dir);
                targetOffset = m_pose.m_pos.m_x - m_navState.m_targetPos.m_x;
                break;
            case RIGHT:
                distExpected = (distExpected - MAX_ROOM_SIZE + m_pose.m_pos.m_y) / -sin(m_pose.m_dir);
                targetOffset = m_pose.m_pos.m_y - m_navState.m_targetPos.m_y;
                break;
        }

        double const distError = distExpected - distFront;

        printf("Measured distance: %f\n", distFront);

        if (distFront > WALL_OFFSET - ROBOT_RADIUS + m_navState.m_offset + 2 * TARGET_PRECISION) {
            if (fabs(distError) > TARGET_PRECISION) {
                m_pose.Update(0, distError);
            }

            return false;
        } else {
            m_pose.Update(0, targetOffset);

            return true;
        }
    }

    void Step() {
        wb_differential_wheels_set_encoders(0.0, 0.0);

        if (wb_robot_step(get_time_step()) == -1) {
            wb_robot_cleanup();
            exit(EXIT_SUCCESS);
        }

        UpdateOdometry();
    }

    void PassiveWait(double _sec) {
        double start_time = wb_robot_get_time();

        do {
            Step();
        } while ((start_time + _sec) > wb_robot_get_time());
    }

    void Forward(double _dist) /* distance in meters */ {
        Forward();
        PassiveWait(1000 * _dist / MAX_SPEED);
        Stop();
    }

    void Forward() {
        wb_differential_wheels_set_speed(MAX_SPEED, MAX_SPEED);
    }

    void Backward(double _dist) /* distance in meters */ {
        Backward();
        PassiveWait(1000 * _dist / MAX_SPEED);
        Stop();
    }

    void Backward() {
        wb_differential_wheels_set_speed(-HALF_SPEED, -HALF_SPEED);
    }

    void Stop() {
        wb_differential_wheels_set_speed(-NULL_SPEED, -NULL_SPEED);
    }

    void Turn(double _angle) {
        Stop();
        Step();

        double neg = (_angle < 0.0) ? -1.0 : 1.0;

        wb_differential_wheels_set_speed(-neg * HALF_SPEED, neg * HALF_SPEED);

        double const start = m_pose.m_dir;
        double const end = wrap(start + _angle, 0, 2 * M_PI);
        double cur = start;

        do {
            Step();
            cur = m_pose.m_dir;
            //printf("current:%f end:%f\n", cur, end);
        } while (fabs(wrap(end - cur, -M_PI, M_PI)) > TURN_PRECISION);

        printf("current:%f end:%f delta:%f\n", cur, end, fabs(wrap(end - cur, -M_PI, M_PI)));

        Stop();
        Step();
    }

    void TurnToHeading(double const _heading) {
        double delta = wrap(_heading - m_pose.m_dir, -M_PI, M_PI);
        printf("Turning to heading: %f\n", delta * 180 / M_PI);

        if (fabs(delta) > ANGLE_PRECISION) {
            Turn(delta);
        }
    }

    double GetTargetHeading() const {
        Vec2 const targetPos = m_navState.m_targetPos;
        Vec2 const curPos = m_pose.m_pos;

        double const heading = (targetPos - curPos).GetDir();
        //printf("Dif: %f, %f\n", (targetPos - curPos).m_x, (targetPos - curPos).m_y);
        return wrap(heading, 0, 2 * M_PI);
    }

    double GetEstimatedHeading() const {
        return m_pose.m_dir;
    }

    bool HasReachedTarget(int targetIndex) {
        bool result = false;

        Vec2 const curPos = m_pose.m_pos;
        Vec2 const targetPos = m_navState.m_targetPos;
        Vec2 const deltaPos = targetPos - curPos;

        double const dx = deltaPos.m_x;
        double const dy = deltaPos.m_y;
        double const dd = (dx * dx) + (dy * dy);
        // printf("Distance^2 to target: %03.3f\n", dd);

        if (dd < TARGET_PRECISION) {
            if (targetIndex == NAV_TARGET_COUNT) {
                return CorrectOdometry();
            } else {
                result = true;
            }
        }

        return result;
    }

    void Run() {
        m_behaviour = SWEEPING;
        
        //Robots initial movement is upwards towards the first target position
        int targetIndex = m_nav.SetNextTarget(m_navState);

        init_devices();
        //camera = wb_robot_get_device("camera");
        //wb_camera_enable(camera, get_time_step());

        srand(time(NULL));

        //wb_led_set(leds[LED_ON], true);
        
        PassiveWait(0.5);

        while (m_behaviour != REST) {
            if (is_there_a_collision_at_left()) {
                printf("Left collision detected\n");
                Backward(0.1);
                Turn(M_PI * -0.25);
                Forward(0.1);
            } else if (is_there_a_collision_at_right()) {
                printf("Right collision detected\n");
                Backward(0.1);
                Turn(M_PI * -0.25);
                Forward(0.1);
            } else {
                int avoid_sensors[] = {
                    DISTANCE_SENSOR_FRONT,
                    DISTANCE_SENSOR_DIAG_LEFT,
                    DISTANCE_SENSOR_DIAG_RIGHT,
                    DISTANCE_SENSOR_LEFT,
                    DISTANCE_SENSOR_RIGHT
                };

                int N = sizeof (avoid_sensors) / sizeof (int);
                double const d_min = 0.0;
                double const d_max = 0.05;

                double l_max = 0;
                for (int i = 0; i < N; ++i) {
                    double const dist = wb_distance_sensor_get_value(distance_sensors[avoid_sensors[i]]);
                    double const l = 1 - smootherstep(d_min, d_max, dist);
                    l_max = max(l, l_max);
                }
                //printf("L_max: %f\n",l_max);
                // double const l = pid.Step(d_target, d_right);
                double const l_avoid = 0; //1.0 * l_max;
                double const l_goal = -1 * wrap(GetTargetHeading() - GetEstimatedHeading(), -M_PI, M_PI);

                //printf("Target: %f Estimated: %f Correction: %f\n", GetTargetHeading() * 180/M_PI, GetEstimatedHeading() * 180/M_PI, wrap(GetTargetHeading() - GetEstimatedHeading(), -M_PI, M_PI) *180/M_PI);

                Vec2 const m_avoid = speedsFromControlParam(l_avoid);
                Vec2 const m_goal = speedsFromControlParam(l_goal);

                Vec2 const result = l_avoid * m_avoid + (1 - l_avoid) * m_goal;

                // TODO: 'normalise' this so that at least one component is at max speed

                wb_differential_wheels_set_speed(MAX_SPEED * clamp(result.m_x, -1, 1), MAX_SPEED * clamp(result.m_y, -1, 1));

                if (HasReachedTarget(targetIndex) || wb_robot_get_time() - m_nav.GetTargetStartTime() > TARGET_TIMEOUT) {
                    printf("Target reached\n");

                    if (m_behaviour != HOMING) {

                        if (targetIndex == NAV_TARGET_COUNT) {
                            //If reached the final target in the current direction, switch to the next direction on the spiral
                            switch (m_navState.m_dir) {
                                case UP:
                                    m_navState.m_dir = LEFT;
                                    break;
                                case LEFT:
                                    m_navState.m_dir = DOWN;
                                    break;
                                case DOWN:
                                    m_navState.m_dir = RIGHT;
                                    break;
                                case RIGHT:
                                    m_navState.m_dir = UP;
                                    break;
                            }
                        }

                        targetIndex = m_nav.SetNextTarget(m_navState);

                        if (m_navState.m_offset >= MAX_ROOM_SIZE / 2) {
                            //If reached the middle of the room, where no more spiral lines are possible, start homing
                            m_behaviour = HOMING;

                            //TODO: Put the homing logic
                        } else if (targetIndex == 1) {
                            //If ready to start in the new direction on the spiral, turn in that direction
                            printf("Turning\n");
                            TurnToHeading(GetTargetHeading());
                        }
                    } else {
                        //If reached the homing target, put the robot at rest
                        m_behaviour = REST;
                    }
                }
            }

            //wb_camera_get_image(camera);
            Step();
        }

    }
};

#endif	/* ROBOT_H */

