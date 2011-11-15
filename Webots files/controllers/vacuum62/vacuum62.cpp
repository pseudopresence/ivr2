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
// static WbDeviceTag camera; 

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
#define MAX_SPEED 200
#define NULL_SPEED 0
#define HALF_SPEED 100
#define MIN_SPEED -200

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

static double randdouble() {
  return rand() / ((double)RAND_MAX + 1);
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
 private:
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
    
    double orientation = (dl - dr) / AXLE_LENGTH; // delta orientation in radian
    double distance = (dl + dr) / 2;
    
    m_theta += orientation;
    m_x += distance * cos(m_theta);
    m_y += distance * sin(m_theta);
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
  
  void Forward() 
  {
    wb_differential_wheels_set_speed(MAX_SPEED, MAX_SPEED);
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
    double cur = start;
    do 
    {
      Step();
      cur = m_theta;
    } 
    while (fabs(cur - start) < fabs(angle));
    
    Stop();
    
    Step();
  }
  
  bool HasReachedTarget()
  {
    bool result = false;
    
    switch (CurrentDirection)
    {    
      case RIGHT:
        result = fabs(m_y) <= CurrentTarget->Y;
        break;
      case UP:
        result = fabs(m_x) >= CurrentTarget->X;
        break;
      case LEFT:
        result = fabs(m_y) >= CurrentTarget->Y;
        break;
      case DOWN:
        result = fabs(m_x) <= CurrentTarget->X;
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
        
        //robot->CurrentDirection = UP;
        break;
        
      case UP:
        robot->NextTarget->X = MAX_ROOM_WIDTH - m_indent;
        robot->NextTarget->Y = MAX_ROOM_HEIGHT - m_indent;
        
        //robot->CurrentDirection = RIGHT;
        break;
        
      case LEFT:
        robot->NextTarget->Y = MAX_ROOM_HEIGHT - m_indent;
        robot->NextTarget->X = m_indent;
        
        //robot->CurrentDirection = DOWN;
        
        break;
        
      case DOWN:
        robot->NextTarget->Y = m_indent;
        
        m_indent += ROBOT_DIAMETER;
        
        robot->NextTarget->X = m_indent;
        
        //robot->CurrentDirection = LEFT;
        break;
    }
    
    printf("Current target: %f %f\n", robot->CurrentTarget->X, robot->CurrentTarget->Y);
    printf("Next target: %f %f\n", robot->NextTarget->X, robot->NextTarget->Y);
        
    //robot->PassiveWait(1.5);
  }
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
  
  // camera = wb_robot_get_device("camera");
  // wb_camera_enable(camera,get_time_step());
  
  srand(time(NULL));
  
  wb_led_set(leds[LED_ON], true);
  
  r.PassiveWait(0.5);

  while (true) {
    /*
    if (is_there_a_virtual_wall()) 
    {    
      printf("Virtual wall detected\n");
      r.Turn(M_PI);
    } 
    else if (is_there_a_collision_at_left()) 
    {    
      printf("Left collision detected\n");
      r.Backward();
      r.PassiveWait(0.5);
      r.Turn(M_PI * randdouble());
    } 
    else if (is_there_a_collision_at_right()) 
    {    
      printf("Right collision detected\n");
      r.Backward();
      r.PassiveWait(0.5);
      r.Turn(-M_PI * randdouble());
    }
    else if (is_there_a_distance_at_left()) 
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
    } 
    else if (is_there_a_distance_at_front()) 
    {    
      printf("Front distance detected\n");
      //go_backward();
      
      if (is_there_a_distance_at_left())
      {
        if (is_there_a_distance_at_right())
        {
          printf("Going backwards\n");
          r.Backward();
          r.PassiveWait(0.5);
          r.Turn(-M_PI / 2);
        }
        else
        {
          printf("Turning to the right\n");
          r.Turn(M_PI / 2);
        }
      }
      else
      {
        printf("Turning to the left\n");
        r.Turn(-M_PI / 2);
      }
    
      r.PassiveWait(0.5);
    } 
    else 
    {*/    
      if (r.HasReachedTarget())
      {        
        //If the robot has reached its current target, turn in the correct direction and switch to next target
        printf("Turning\n");
        
        r.Turn(-M_PI / 2);
        r.PassiveWait(0.5);
        
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
      }
      
      r.Forward();
    /*}*/
    
    // wb_camera_get_image(camera);
    fflush_ir_receiver();
    r.Step();
  };
  
  r.Shutdown();
  
  return EXIT_SUCCESS;
}

