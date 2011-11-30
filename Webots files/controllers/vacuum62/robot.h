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
#include <deque>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/camera.h>

#include "vec2.h"
#include "navigation.h"

enum Behaviour {
    REST, SWEEPING, HOMING
};

struct Pose {
    Pose(Vec2 const _pos, double const _dir);
    void Update(double const _turn, double const _distance);

    Vec2 m_pos;
    double m_dir;
};

class Robot {
private:
    Pose m_pose;
    Behaviour m_behaviour;
    NavigationState m_navState;
    Navigation m_nav;
    std::deque<NavigationState> m_targetQueue;
    double m_targetStartTime;
public:

    Robot(Vec2 const _pos, double _offset, double _dir);

    void Init();

    void Shutdown();

    void UpdateOdometry();

    bool CorrectOdometry();

    void Step();

    void PassiveWait(double _sec);

    void Forward(double _dist);

    void Forward();

    void Backward(double _dist);

    void Backward();

    void Stop();

    void Turn(double _angle);

    void TurnToHeading(double const _heading);

    double GetTargetHeading() const;

    double GetEstimatedHeading() const;

    bool HasReachedTarget();

    bool ShouldIgnoreObstacle();

    void Run();
};

#endif	/* ROBOT_H */

