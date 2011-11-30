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
  "dist_front",
  "dist_right",
  "dist_diagleft",
  "dist_diagright"
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

static bool is_there_a_collision_at_left() {
    return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0);
}

static bool is_there_a_collision_at_right() {
    return (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}

static bool was_obstacle_detected() {
    return (wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT]) < DANGER_DISTANCE);
}

Pose::Pose(Vec2 const _pos, double const _dir) :
m_pos(_pos),
m_dir(_dir) {
}

void Pose::Update(double const _turn, double const _distance) {
  m_dir = wrap(m_dir + _turn, 0.0, 2 * M_PI);
  // TODO do we get a better estimate using the previous direction to update position, or the new one? Or an average?
  m_pos += Vec2::FromDirLen(m_dir, _distance);

  printf("X: %03.3lf, Y: %03.3lf, O: %03.3lf\n", m_pos.m_x, m_pos.m_y, m_dir * 180 / M_PI);
}

Robot::Robot(Vec2 const _pos, double _offset, double _dir) :
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

  while (navState.m_offset < MAX_ROOM_SIZE / 2) {
    m_nav.SetNextTarget(navState);
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

  double const dd = (targetPos - curPos).GetLengthSquared();

  if (dd < TARGET_PRECISION) {
    if ((m_targetQueue.size() < 2) || (m_targetQueue.at(1).m_dir != m_navState.m_dir)) {
      //If it is the final target in the current direction, correct the position estimate
      result = CorrectOdometry();
    } else {
      result = true;
    }
  }

  return result;
}

/* Returns true if we are expecting to see the wall */
bool Robot::ShouldIgnoreObstacle() {
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

void Robot::Run() {
  m_behaviour = SWEEPING;

  PassiveWait(0.5);

  while (m_behaviour != REST) {
    
    /* Navigation logic */
    if (HasReachedTarget() /*|| wb_robot_get_time() - m_targetStartTime > TARGET_TIMEOUT*/) {
      printf("Target reached\n");

      if (m_behaviour != HOMING) {

        m_targetQueue.pop_front();

        if (m_targetQueue.empty()) {
          //If the robot is already covering the next target (spiral finished), switch to homing behaviour
          printf("Homing\n");
          m_behaviour = HOMING;

          //TODO: Put the homing logic and remove the following line
          m_behaviour = REST;
        } else {
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

      } else {
        //If reached the homing target, put the robot at rest
        m_behaviour = REST;
      }
    }

    /* Obstacle avoidance logic */
    if (was_obstacle_detected() && !ShouldIgnoreObstacle()) {
      printf("Obstacle detected\n");

      //TODO: Make dynamic
      m_targetQueue.pop_front();
      m_targetQueue.pop_front();

      NavigationState avoidState1 = m_navState;
      avoidState1.m_targetPos = Vec2(0, 0.6).RotatedBy(m_pose.m_dir) + m_pose.m_pos;
      avoidState1.m_dir++;

      NavigationState avoidState2 = m_navState;
      avoidState2.m_targetPos = Vec2(0.6, 0).RotatedBy(m_pose.m_dir) + avoidState1.m_targetPos;

      NavigationState avoidState3 = m_navState;
      avoidState3.m_targetPos = Vec2(0.6, 0).RotatedBy(m_pose.m_dir) + avoidState2.m_targetPos;

      m_targetQueue.front().m_dir--;

      m_targetQueue.push_front(avoidState3);
      m_targetQueue.push_front(avoidState2);
      m_targetQueue.push_front(avoidState1);

      m_navState = m_targetQueue.front();

      TurnToHeading(GetTargetHeading());

      m_targetStartTime = wb_robot_get_time();
    } else {
      /* Control logic */
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

      double const l_avoid = 0; //1.0 * l_max;
      double const l_goal = -1 * wrap(GetTargetHeading() - GetEstimatedHeading(), -M_PI, M_PI);

      // printf("Target: %f Estimated: %f Correction: %f\n", GetTargetHeading() * 180/M_PI, GetEstimatedHeading() * 180/M_PI, wrap(GetTargetHeading() - GetEstimatedHeading(), -M_PI, M_PI) *180/M_PI);

      Vec2 const m_avoid = speedsFromControlParam(l_avoid);
      Vec2 const m_goal = speedsFromControlParam(l_goal);

      Vec2 const result = l_avoid * m_avoid + (1 - l_avoid) * m_goal;

      // TODO: 'normalise' this so that at least one component is at max speed?
      wb_differential_wheels_set_speed(MAX_SPEED * clamp(result.m_x, -1, 1), MAX_SPEED * clamp(result.m_y, -1, 1));
    }

    //wb_camera_get_image(camera);
    Step();
  }

}