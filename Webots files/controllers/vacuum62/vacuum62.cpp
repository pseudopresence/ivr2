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
#define MAX_SPEED (500) /* Units: mm/s */
#define NULL_SPEED (0)
#define HALF_SPEED ((MAX_SPEED)/2.0)
#define MIN_SPEED (-(MAX_SPEED))

#define ROBOT_RADIUS 0.17
#define ROBOT_DIAMETER 2 * ROBOT_RADIUS
#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

/* Room Size */
#define MAX_ROOM_HEIGHT 6 - ROBOT_DIAMETER - 0.1
#define MAX_ROOM_WIDTH 6 - ROBOT_DIAMETER - 0.1

enum Direction { LEFT, UP, RIGHT, DOWN };

/* helper functions */

/* random double [0.0 - 1.0] */
static double randdouble() {
  return rand() / ((double)RAND_MAX + 1);
}

/* wrap a value into the given range, for example -M_PI to M_PI */
static double wrap(double _x, double const _min, double const _max)
{
   while (_x < _min)
   {
     _x += (_max - _min);
   }

   while (_x > _max)
   {
     _x -= (_max - _min);
   }

   return _x;
}

/* clamp a value to the given range, for example 0 to 1 */
static double clamp(double _x, double const _min, double const _max)
{
  if ( _x < _min )
  {
    return _min;
  } else if ( _max < _x ) {
    return _max;
  } else {
    return _x;
  }
}

/* smootherstep interpolation function from http://en.wikipedia.org/wiki/Smoothstep */
static float smootherstep(float edge0, float edge1, float x)
{
  // Scale, and clamp x to 0..1 range
  x = clamp((x - edge0)/(edge1 - edge0), 0, 1);
  // Evaluate polynomial
  return x*x*x*(x*(x*6 - 15) + 10);
}

static double forceFromDist(double const _dist)
{
  return 1 - smootherstep(0.05, 0.1, _dist);
}

static double speedFromControlParam(double const _param)
{
  if (_param < 0)
  {
    return 1;
  }
  else
  {
    return 1 - 2*_param;
  }
}

class Vec2
{
public:
  double m_x;
  double m_y;

  Vec2() : m_x(0), m_y(0) {}
  Vec2(double const _x, double const _y) : m_x(_x), m_y(_y) {}
  Vec2(Vec2 const& _v) : m_x(_v.m_x), m_y(_v.m_y) {}

  Vec2 const& operator=(Vec2 const& _v) { m_x = _v.m_x; m_y = _v.m_y; return *this; }

  double GetDir() const { return atan2(m_y, m_x); }
  static Vec2 FromDir(double const _dir) { return Vec2(cos(_dir), sin(_dir)); }
  
  static Vec2 XAxis() { return Vec2(1, 0); }
  static Vec2 YAxis() { return Vec2(0, 1); }

  Vec2 RotatedBy(double const _theta) const
  {
    return Vec2(
        m_x * cos(_theta) - m_y * sin(_theta),
        m_x * sin(_theta) + m_y * cos(_theta)
    );
  }
};

inline Vec2 operator+(Vec2 const& _l, Vec2 const& _r)
{
  return Vec2(_l.m_x + _r.m_x, _l.m_y + _r.m_y);
}

inline Vec2 operator-(Vec2 const& _l, Vec2 const& _r)
{
  return Vec2(_l.m_x - _r.m_x, _l.m_y - _r.m_y);
}

inline Vec2 operator*(double const _s, Vec2 const& _v)
{
  return Vec2(_v.m_x * _s, _v.m_y * _s);
}

inline Vec2 operator*(Vec2 const& _v, double const _s)
{
  return _s * _v;
}

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int) wb_robot_get_basic_time_step();
  return time_step;
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


class Location
{    
  public:
    double X;
    double Y;
    
    Location(double _x, double _y)
    {
      X = _x;
      Y = _y;
    }
};

class Robot
{
 public:
  double m_x;
  double m_y;
  double m_theta;
  
 public:
  Direction CurrentDirection;
  Location* CurrentTarget;
  Location* NextTarget;
  
  Robot(double _x, double _y, double _theta):
    m_x(_x),
    m_y(_y),

    m_theta(_theta)
  {
    CurrentDirection = RIGHT;
    CurrentTarget = new Location(_x, _y);
    NextTarget = new Location(_x, _y);
  }

  ~Robot()
  {
    delete CurrentTarget;
    delete NextTarget;
  }
  
  void Init()
  {
    wb_robot_init();
    wb_differential_wheels_enable_encoders(get_time_step());
  }
  
  void Shutdown()
  {
    wb_differential_wheels_disable_encoders();
  }
  
  void UpdateOdometry()
  {
    double encl = wb_differential_wheels_get_left_encoder();
    double encr = wb_differential_wheels_get_right_encoder();
    double dl = encl / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
    double dr = encr / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
    
    double turn = (dl - dr) / AXLE_LENGTH; // delta orientation in radian
    double distance = (dl + dr) / 2;
    
    m_theta += turn;
    m_theta = wrap(m_theta, 0, 2*M_PI);
    m_x += distance * cos(m_theta);
    m_y += distance * -sin(m_theta);
    //printf("X: %03.3lf, Y: %03.3lf, O: %03.3lf [T: %03.3lf, %03.3lf]\n", m_x, m_y, m_theta, CurrentTarget->X, CurrentTarget->Y);
  }
  
  void Step() {
    wb_differential_wheels_set_encoders(0.0, 0.0);
    if (wb_robot_step(get_time_step()) == -1) {
      wb_robot_cleanup();
      exit(EXIT_SUCCESS);
    }
    UpdateOdometry();
  }
  
  void PassiveWait(double sec) 
  {
    double start_time = wb_robot_get_time();
    
    do 
    {
      Step();
    } 
    while((start_time + sec) > wb_robot_get_time());
  }
 
  void Forward(double _dist) /* distance in meters */
  {
    Forward();
    PassiveWait(1000 * _dist / MAX_SPEED);
    Stop();
  }

  void Forward() 
  {
    wb_differential_wheels_set_speed(MAX_SPEED, MAX_SPEED);
  }

  void Backward(double _dist) /* distance in meters */
  {
    Backward();
    PassiveWait(1000 * _dist / MAX_SPEED);
    Stop();
  }

  void Backward() 
  {
    wb_differential_wheels_set_speed(-HALF_SPEED, -HALF_SPEED);
  }

  void Stop() 
  {
    wb_differential_wheels_set_speed(-NULL_SPEED, -NULL_SPEED);
  }
  
  void Turn(double angle)
  {
    Stop();
    Step();
    
    double neg = (angle < 0.0) ? -1.0 : 1.0;
    
    wb_differential_wheels_set_speed(neg*HALF_SPEED, -neg*HALF_SPEED);
    
    double const start = m_theta;
    double const end = wrap(start + angle, 0, 2*M_PI);
    double cur = start;
    do 
    {
      Step();
      cur = m_theta;
    } 
    while (fabs(wrap(end - cur, -M_PI, M_PI)) > 0.03);
    
    Stop();
    Step();
  }

  void TurnToHeading(double const _heading)
  {
    double delta = wrap(_heading - m_theta, -M_PI, M_PI);
    // printf("Turning to heading: %03.3lf\n", _heading);
    if (fabs(delta) > 0.05) { Turn(delta); }
  }

  double GetTargetHeading()
  {
    return wrap(-atan2(CurrentTarget->Y - m_y, CurrentTarget->X - m_x), 0, 2*M_PI);
  }
  
  bool HasReachedTarget()
  {
    double const dx = m_x - CurrentTarget->X;
    double const dy = m_y - CurrentTarget->Y;
    double const dd = (dx * dx) + (dy * dy);
    // printf("Distance^2 to target: %03.3f\n", dd);
    return dd < 0.3;
    
    bool result = false;
    
    switch (CurrentDirection)
    {    
      case RIGHT:
        result = fabs(m_y) <= CurrentTarget->Y;
        break;
      case UP:
        result = m_x >= CurrentTarget->X;
        break;
      case LEFT:
        result = fabs(m_y) >= CurrentTarget->Y;
        break;
      case DOWN:
        result = m_x <= CurrentTarget->X;
        break;      
    }
    
    //printf("Current location: %f %f\n", m_x, m_y);
    //printf("Is: %s %f %f\n", (m_y >= CurrentTarget->Y) ? "true" : "false", m_y, CurrentTarget->Y);
    
    return result;
  }
};

class Navigation
{
private:
  double m_indent;
  
public:
  Navigation()
  {
    m_indent = 0;
  }
  
  void SetNextTarget(Robot* robot)
  {
    switch (robot->CurrentDirection)
    {
      case RIGHT:
        robot->NextTarget->X = MAX_ROOM_WIDTH - m_indent;
        robot->NextTarget->Y = m_indent;
        
        break;
        
      case UP:
        robot->NextTarget->X = MAX_ROOM_WIDTH - m_indent;
        robot->NextTarget->Y = MAX_ROOM_HEIGHT - m_indent;
        
        break;
        
      case LEFT:
        robot->NextTarget->Y = MAX_ROOM_HEIGHT - m_indent;
        robot->NextTarget->X = m_indent;
        
        break;
        
      case DOWN:
        robot->NextTarget->X = m_indent;
        
        m_indent += ROBOT_DIAMETER;
        
        robot->NextTarget->Y = m_indent;
        
        break;
    }
    
    printf("Current target: %f %f\n", robot->CurrentTarget->X, robot->CurrentTarget->Y);
    printf("Next target: %f %f\n", robot->NextTarget->X, robot->NextTarget->Y);
  }
};

class PIDController
{
  public:
    PIDController(double _Kp, double _Ki, double _Kd) : m_Kp(_Kp), m_Ki(_Ki), m_Kd(_Kd), m_prev(0), m_sum(0), m_firstStep(true) {}

    double Step(double _goal, double _curr)
    {
      // There's no sensible initialisation value of m_prev before the first Step(), so have a special case
      if (m_firstStep)
      {
        m_prev = _curr;
        m_firstStep = false;
      }

      double const e = _goal - _curr;
      m_sum += e;

      double const v = m_Kp * e + m_Ki * m_sum + m_Kd * (_curr - m_prev);
      
      m_prev = _curr;
      return v;
    }
  private:
    double const m_Kp;
    double const m_Ki;
    double const m_Kd;

    double m_prev;
    double m_sum;

    bool m_firstStep;
};

/* main */
int main(int argc, char **argv)
{
  Robot r(0,0,0);
  Navigation n;
 
  r.Init();
  
  //Robots initial movement is upwards towards the first target position
  n.SetNextTarget(&r);
  r.CurrentDirection = UP;
  r.CurrentTarget->X = r.NextTarget->X;
  r.CurrentTarget->Y = r.NextTarget->Y;
  
  //Look ahead for 1 target
  n.SetNextTarget(&r);
 
  printf("Controller of the iRobot Create robot started...\n");
  
  init_devices();
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, get_time_step());  

  srand(time(NULL));
  
  wb_led_set(leds[LED_ON], true);

  Vec2 prevForce;
  
  r.PassiveWait(0.5);
  // r.TurnToHeading(r.GetTargetHeading());
  
  double d_target = 0.1;
  PIDController pid(1, 0, -10);

  while (true) {
    if (is_there_a_collision_at_left()) 
    {    
      printf("Left collision detected\n");
      r.Backward(0.1);
      r.Turn(M_PI * -0.25);
      r.Forward(0.1);
      // r.TurnToHeading(r.GetTargetHeading());
      // d_target += 0.05;
    } 
    else if (is_there_a_collision_at_right()) 
    {    
      printf("Right collision detected\n");
      r.Backward(0.1);
      r.Turn(M_PI * -0.25);
      r.Forward(0.1);
      // d_target += 0.05;
      // r.TurnToHeading(r.GetTargetHeading());
   }
   else 
   {    
      double const d_left = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_LEFT]);
      double const d_frontleft = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_FRONT_LEFT]);
      double const d_frontright = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_RIGHT]);
      double const d_right = wb_distance_sensor_get_value(distance_sensors[DISTANCE_SENSOR_RIGHT]);

      double const l = pid.Step(d_target, d_right);
      double const s_left = speedFromControlParam(l);
      double const s_right = speedFromControlParam(-l);

      wb_differential_wheels_set_speed(MAX_SPEED * s_left, MAX_SPEED * s_right);

      /*
      double const goalWeight = 1.0;
      double const avoidWeight = 10.0;
    
      // Avoidance force in local coordinates
      double const avoidForce_lr = forceFromDist(d_left) - forceFromDist(d_right);
      double const avoidForce_fb = 0.5 * forceFromDist(d_frontleft) + 0.5 * forceFromDist(d_frontright);
      Vec2 const avoidForce = Vec2(avoidForce_lr, avoidForce_fb);

      // Goal force in local coordinates
      double const goalHeading = r.GetTargetHeading() - r.m_theta;
      Vec2 const goalForce = Vec2::FromDir(goalHeading);
      
      // Total force in local coordinates
      Vec2 const totalForceLocal = goalWeight * goalForce - avoidWeight * avoidForce;
      Vec2 const totalForce = totalForceLocal.RotatedBy(r.m_theta);

      Vec2 const curForce = 0.1 * totalForce + 0.9 * prevForce;
      prevForce = curForce;

      // Total force direction in local coordinates
      double const forceHeading = curForce.GetDir(); 
      r.TurnToHeading(forceHeading);
      r.Forward();
      */

      if (r.HasReachedTarget())
      {        
        //If the robot has reached its current target, turn in the correct direction and switch to next target
        printf("Target reached\n");
        
        // r.Turn(-M_PI / 2);
        // r.PassiveWait(0.5);
        
        switch (r.CurrentDirection)
        {
          case RIGHT:
            r.CurrentDirection = UP;
            break;
          case UP:
            r.CurrentDirection = LEFT;
            break;
          case LEFT:
            r.CurrentDirection = DOWN;
            break;
          case DOWN:
            r.CurrentDirection = RIGHT;
            break;
        }
        
        r.CurrentTarget->X = r.NextTarget->X;
        r.CurrentTarget->Y = r.NextTarget->Y;
        n.SetNextTarget(&r);
        // r.TurnToHeading(r.GetTargetHeading());
      }
      
      // r.Forward();
    } 
    wb_camera_get_image(camera);
    fflush_ir_receiver();
    r.Step();
  }
  
  r.Shutdown();
  
  return EXIT_SUCCESS;
}
