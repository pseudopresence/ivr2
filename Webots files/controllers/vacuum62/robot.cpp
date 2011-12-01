#include "robot.h"

#include "util.h"
#include "pid.h"
#include "constants.h"

#include <stdio.h>

/* device stuff */
static WbDeviceTag camera;

static WbDeviceTag bumpers[BUMPERS_NUMBER];
static const char *bumpers_name[BUMPERS_NUMBER] = {
    "bumper_left",
    "bumper_right"
};

static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_name[DISTANCE_SENSORS_NUMBER] = {
    "dist_left",
    "dist_front_left_3",
    "dist_front_left_2",
    "dist_front_left_1",
    "dist_front",
    "dist_front_right_1",
    "dist_front_right_2",
    "dist_front_right_3",
    "dist_right"
};

static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_name[LEDS_NUMBER] = {
    "led_on",
    "led_play",
    "led_step"
};

/* helper functions */
static double forceFromDist(double const _dist) {
    return 1 - smootherstep(0.05, 0.1, _dist);
}

static double speedFromControlParam(double const _param) {
    if (_param < 0) {
        return 1;
    } else {
        return 1 - 3 * _param;
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

/*static bool is_there_a_collision_at_left() {
    return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0);
}

static bool is_there_a_collision_at_right() {
    return (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}*/

static bool is_collision_detected() {
    return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0) ||
            (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}

static bool is_obstacle_detected() {
    int avoid_sensors[] = {
        DISTANCE_SENSOR_FRONT,
        DISTANCE_SENSOR_FRONT_LEFT_1,
        DISTANCE_SENSOR_FRONT_LEFT_2,
        DISTANCE_SENSOR_FRONT_LEFT_3,
        DISTANCE_SENSOR_FRONT_RIGHT_1,
        DISTANCE_SENSOR_FRONT_RIGHT_2,
        DISTANCE_SENSOR_FRONT_RIGHT_3
    };

    int N = sizeof (avoid_sensors) / sizeof (int);

    for (int i = 0; i < N; i++) {
        if (wb_distance_sensor_get_value(distance_sensors[avoid_sensors[i]]) <= DANGER_DISTANCE) {
            return true;
        }
    }

    return false;
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


Pose::Pose(Vec2 const _pos, double const _dir) :
m_pos(_pos),
m_dir(_dir) {
}

void Pose::Update(double const _turn, double const _distance) {
    m_dir = wrap(m_dir + _turn, 0.0, 2 * M_PI);
    // TODO do we get a better estimate using the previous direction to update position, or the new one? Or an average?
    m_pos += Vec2::FromDirLen(m_dir, _distance);

    //printf("X: %03.3lf, Y: %03.3lf, O: %03.3lf\n", m_pos.m_x, m_pos.m_y, m_dir * 180 / M_PI);
}

Robot::Robot(Vec2 const _pos, double _offset, double _dir) :
m_home(_pos, _dir),
m_pose(_pos, _dir),
m_navState(RIGHT, _offset, _pos),
m_behaviour(REST),
m_nav(),
m_targetQueue(),
m_targetStartTime(0) {
}

void Robot::Init() {
    wb_robot_init();
    wb_differential_wheels_enable_encoders(get_time_step());
    wb_differential_wheels_set_encoders(0.0, 0.0);

    init_devices();
    //camera = wb_robot_get_device("camera");
    //wb_camera_enable(camera, get_time_step());
    //wb_led_set(leds[LED_ON], true);
    srand(time(NULL));

    /* Generate target path */
    NavigationState navState(m_navState);

    while ((navState.m_offset < MAX_ROOM_SIZE / 2) && (m_nav.SetNextSpiralTarget(navState) > 0)) {
        m_targetQueue.push_back(navState);
    }

    m_navState = m_targetQueue.front();
}

void Robot::Shutdown() {
    wb_differential_wheels_disable_encoders();
}

void Robot::UpdateOdometry() {
    double encl = wb_differential_wheels_get_left_encoder();
    double encr = wb_differential_wheels_get_right_encoder();
    double dl = encl / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
    double dr = encr / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter

    double const TURN_HACK_FACTOR = 0.97;
    double const DIST_HACK_FACTOR = 1; //1.20;
    double turn = TURN_HACK_FACTOR * (dr - dl) / AXLE_LENGTH; // delta orientation in radian
    double distance = DIST_HACK_FACTOR * (dr + dl) / 2;

    m_pose.Update(turn, distance);
    wb_differential_wheels_set_encoders(0.0, 0.0);
}

bool Robot::CorrectOdometry() {
    double const distFront = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT]);
    double const distFrontLeft = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT_LEFT_3]);

    //Using the left front and front sensor readings, 
    //this calculates the angle under which the robot is facing the wall
    double const dirOffset = atan2(distFrontLeft - distFront - FRONT_SENSOR_INTERVAL_Y, FRONT_SENSOR_INTERVAL_X);
    double const dirError = wrap(dirOffset + M_PI / 2 * (int) m_navState.m_dir, 0.0, 2 * M_PI) - m_pose.m_dir;

    //Correct robot's orientation estimate
    m_pose.Update(wrap(dirError, -M_PI, M_PI), 0);

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

void Robot::Step() {
    if (wb_robot_step(get_time_step()) == -1) {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }

    UpdateOdometry();
}

void Robot::Stop() {
    wb_differential_wheels_set_speed(-NULL_SPEED, -NULL_SPEED);
}

void Robot::Forward() {
    wb_differential_wheels_set_speed(MAX_SPEED, MAX_SPEED);
}

void Robot::Backward() {
    wb_differential_wheels_set_speed(-HALF_SPEED, -HALF_SPEED);
}

void Robot::PassiveWait(double _sec) {
    double start_time = wb_robot_get_time();

    do {
        Step();
    } while ((start_time + _sec) > wb_robot_get_time());
}

void Robot::Forward(double _dist /* distance in meters */) {
    Forward();
    PassiveWait(1000 * _dist / MAX_SPEED);
    Stop();
}

void Robot::Backward(double _dist /* distance in meters */) {
    Backward();
    PassiveWait(1000 * _dist / MAX_SPEED);
    Stop();
}

/* Perform an on-the-spot turn by a given angle */
void Robot::Turn(double _angle) {
    Stop();
    Step();

    double neg = (_angle < 0.0) ? -1.0 : 1.0;

    wb_differential_wheels_set_speed(-1.05 * neg * HALF_SPEED, neg * HALF_SPEED);

    double const start = m_pose.m_dir;
    double const end = wrap(start + _angle, 0.0, 2 * M_PI);
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

/* Perform an on-the-spot turn to a given heading */
void Robot::TurnToHeading(double const _heading) {
    double delta = wrap(_heading - m_pose.m_dir, -M_PI, M_PI);
    printf("Turning to heading: %f\n", delta * 180 / M_PI);

    if (fabs(delta) > ANGLE_PRECISION) {
        Turn(delta);
    }
}

double Robot::GetTargetHeading() const {
    Vec2 const targetPos = m_navState.m_targetPos;
    Vec2 const curPos = m_pose.m_pos;

    double const heading = (targetPos - curPos).GetDir();
    //printf("Dif: %f, %f\n", (targetPos - curPos).m_x, (targetPos - curPos).m_y);
    return wrap(heading, 0.0, 2 * M_PI);
}

double Robot::GetEstimatedHeading() const {
    return m_pose.m_dir;
}

bool Robot::HasReachedTarget() {
    bool result = false;

    Vec2 const curPos = m_pose.m_pos;
    Vec2 const targetPos = m_navState.m_targetPos;

    double dd;

    if (m_navState.m_targetType != AVOIDANCE) {
        dd = (targetPos - curPos).GetLengthSquared();
    } else {
        dd = (targetPos - curPos).GetLength();
    }

    if (dd < TARGET_PRECISION) {
        if ((m_navState.m_targetType != AVOIDANCE)
                && ((m_targetQueue.size() < 2) || (m_targetQueue.at(1).m_dir != m_navState.m_dir))) {
            //If it is the final target in the current direction, correct the position estimate
            result = CorrectOdometry();
        } else {
            result = true;
        }
    }

    return result;
}

/* Returns true if we are expecting to see the wall */
bool Robot::IsWallExpected() {
    Vec2 const targetPos = m_navState.m_targetPos;

    if (((targetPos.m_x <= DANGER_DISTANCE) && (m_navState.m_dir == DOWN))
            || ((MAX_ROOM_SIZE - targetPos.m_x <= DANGER_DISTANCE) && (m_navState.m_dir == UP))
            || ((targetPos.m_y <= DANGER_DISTANCE) && (m_navState.m_dir == RIGHT))
            || ((MAX_ROOM_SIZE - targetPos.m_y <= DANGER_DISTANCE) && (m_navState.m_dir == LEFT))) {
        return true;
    } else {
        return false;
    }
}

void Robot::generateHomingTargets() {
    int count = 0;
    int targetCount;
    int targetIndex;
    double distance;
    Vec2* targets = m_nav.GetHomingTargets(m_pose.m_pos, m_home.m_pos, count);
    
    Vec2 target1;
    Vec2 target2;
    Vec2 target3;
   

    NavigationState navState(m_navState);
    navState.m_offset = 0;
    navState.m_targetType = HOME;

    if (count > 2) {
        target1 = targets[0];
        target2 = targets[1];
        target3 = targets[3];
        
        for (int i = 0; i < count - 1; i++) {
            targetCount = NAV_TARGET_COUNT;
            targetIndex = 1;
            distance = 0;           

            m_nav.SetNextHomingTarget(navState, target1, target2, distance, targetIndex, targetCount);

            if (targetCount > 0) {
                m_targetQueue.push_back(navState);

                for (int j = 2; j < targetCount + 1; j++) {
                    m_nav.SetNextHomingTarget(navState, target1, target2, distance, j, targetCount);
                    m_targetQueue.push_back(navState);
                }
            }
            
            target1 = target2;
            target2 = target3;
        }
    }
}

void Robot::Run() {
    m_behaviour = SWEEPING;

    PassiveWait(0.5);

    while (m_behaviour != REST) {
        /* Navigation logic */
        if (HasReachedTarget() /*|| wb_robot_get_time() - m_targetStartTime > TARGET_TIMEOUT*/) {
            printf("Target reached\n");

            m_targetQueue.pop_front();

            if (m_targetQueue.empty()) {
                if (m_behaviour == HOMING) {
                    //If reached the homing target, turn in the initial direction and put the robot at rest
                    TurnToHeading(m_home.m_dir);

                    m_behaviour = REST;
                } else {
                    printf("Homing\n");
                    m_behaviour = HOMING;

                    generateHomingTargets();
                }
            }

            if (!m_targetQueue.empty()) {
                Direction const prevDir = m_navState.m_dir;

                m_navState = m_targetQueue.front();

                printf("Current target: %f %f\n", m_navState.m_targetPos.m_x, m_navState.m_targetPos.m_y);

                if (m_navState.m_dir != prevDir) {
                    //If ready to start in the new direction on the spiral, turn in that direction
                    printf("Turning\n");

                    TurnToHeading(GetTargetHeading());
                }

                m_targetStartTime = wb_robot_get_time();
            }
        }//TODO: REMOVE THIS
        else if ((m_behaviour == HOMING) || (m_navState.m_offset > 2.5)) {
            //TurnToHeading(GetTargetHeading());
            printf("X: %03.3lf, Y: %03.3lf, O: %03.3lf\n", m_pose.m_pos.m_x, m_pose.m_pos.m_y, m_pose.m_dir * 180 / M_PI);
        }

        bool const isCollision = is_collision_detected();

        /* Obstacle avoidance logic */
        if (is_obstacle_detected() && !IsWallExpected() || isCollision) {

            if (isCollision) {
                printf("Collision detected\n");

                Backward(0.5);

            } else {
                printf("Obstacle detected\n");
            }

            /* If an obstacle is detected, we discard the next few target positions,
            and instead generate new ones that move around the obstacle. */
            //TODO: Generalize the condition
            while (!m_targetQueue.empty() && (m_targetQueue.front().m_targetPos.m_x < m_pose.m_pos.m_x + 2 * OBSTACLE_SIZE)) {
                printf("Removing target: %f %f\n", m_targetQueue.front().m_targetPos.m_x, m_targetQueue.front().m_targetPos.m_y);
                m_targetQueue.pop_front();
            }

            NavigationState avoidState1 = m_navState;
            avoidState1.m_targetPos = Vec2(0, OBSTACLE_SIZE).RotatedBy(m_pose.m_dir) + m_pose.m_pos;
            avoidState1.m_targetType = AVOIDANCE;
            avoidState1.m_dir++;

            NavigationState avoidState2 = m_navState;
            avoidState2.m_targetType = AVOIDANCE;
            avoidState2.m_targetPos = Vec2(OBSTACLE_SIZE, 0).RotatedBy(m_pose.m_dir) + avoidState1.m_targetPos;

            NavigationState avoidState3 = m_navState;
            avoidState3.m_targetType = AVOIDANCE;
            avoidState3.m_targetPos = Vec2(OBSTACLE_SIZE, 0).RotatedBy(m_pose.m_dir) + avoidState2.m_targetPos;

            NavigationState avoidState4 = m_navState;
            avoidState4.m_targetPos = Vec2(0, -OBSTACLE_SIZE).RotatedBy(m_pose.m_dir) + avoidState3.m_targetPos;
            avoidState4.m_dir--;

            m_targetQueue.push_front(avoidState4);
            m_targetQueue.push_front(avoidState3);
            m_targetQueue.push_front(avoidState2);
            m_targetQueue.push_front(avoidState1);

            m_navState = m_targetQueue.front();

            printf("Current target: %f %f\n", m_navState.m_targetPos.m_x, m_navState.m_targetPos.m_y);

            TurnToHeading(GetTargetHeading());

            m_targetStartTime = wb_robot_get_time();
        } else {
            /*int avoid_sensors[] = {
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

            double const l_avoid = 0; //1.0 * l_max;*/
            double const l_goal = -1 * wrap(GetTargetHeading() - GetEstimatedHeading(), -M_PI, M_PI);

            if (m_behaviour == HOMING) {
                printf("Target: %f Estimated: %f Correction: %f\n", GetTargetHeading() * 180 / M_PI, GetEstimatedHeading() * 180 / M_PI, wrap(GetTargetHeading() - GetEstimatedHeading(), -M_PI, M_PI) *180 / M_PI);
            }

            //                Vec2 const m_avoid = speedsFromControlParam(l_avoid);
            Vec2 const m_goal = speedsFromControlParam(l_goal);

            //                Vec2 const result = l_avoid * m_avoid + (1 - l_avoid) * m_goal;

            // TODO: 'normalise' this so that at least one component is at max speed              
            wb_differential_wheels_set_speed(MAX_SPEED * clamp(m_goal.m_x, -1, 1), MAX_SPEED * clamp(m_goal.m_y, -1, 1));
        }

        //wb_camera_get_image(camera);
        Step();
    }
}
