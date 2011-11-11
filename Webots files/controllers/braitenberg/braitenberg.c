/*
 * File:         braitenberg.c
 * Date:         September 1st, 2006
 * Description:  A controller moving various robots using the Braitenberg method.
 * Author:       Simon Blanchoud
 *
 * Copyright (c) 2008 Cyberbotics - www.cyberbotics.com
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <string.h>
#include <stdio.h>

#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)

static WbDeviceTag sensors[MAX_SENSOR_NUMBER], camera;
static double matrix[MAX_SENSOR_NUMBER][2];
static int num_sensors;
static double range;
static int time_step = 0;

/* We use this variable to enable a camera device if the robot has one. */
static int camera_enabled;

static void initialize()
{
    /* necessary to initialize Webots */
    wb_robot_init();
    
    time_step = wb_robot_get_basic_time_step();
  
    const char *robot_name = wb_robot_get_name();

    const char alice_name[] = "ir0";
    const char e_puck_name[] = "ps0";
    const char khepera_name[] = "ds0";
    const char koala_name[] = "ds0";
    const char magellan_name[] = "so0";
    const char pioneer2_name[] = "ds0";
    const char scout2_name[] = "ds0";

    double alice_matrix[2][2] = { {-40, 80}, {80, -40} };
    double e_puck_matrix[8][2] =
        { {150, -35}, {100, -15}, {80, -10}, {-10, -10},
        {-10, -10}, {-10, 80}, {-30, 100}, {-20, 150} };
    double khepera_matrix[8][2] = 
        { {-2, 4}, {-3, 5}, {-7, 7}, {7, -6},
        {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5} };
    double khepera3_matrix[9][2] = 
        { {-5000, -5000}, {-20000, 40000}, {-30000, 50000}, {-70000, 70000}, {70000, -60000},
        {50000, -40000}, {40000, -20000}, {-5000, -5000}, {-10000, -10000} };
    double koala_matrix[16][2] =
        { {17, -1}, {8, -2}, {4, -3}, {9, -2}, {5, -3}, {-4, -2}, {-4, -2},
        {-2, -2}, {-2, -2}, {-2, -4}, {-2, -4}, {-4, 5}, {-3, 8}, {-3, 5},
        {-2, 10}, {-1, 15} };
    double magellan_matrix[16][2] =
        { {-1, 5}, {-3, 10}, {-3, 9}, {-3, 7}, {-3, 7}, {-4, -2}, {-4, -2},
        {-2, -2}, {-1, -1}, {-2, -2}, {-2, -4}, {-2, -4}, {9, -2}, {10, -4},
        {8, -3}, {12, -2} };
    double pioneer2_matrix[16][2] =
        { {-1, 15}, {-3, 13}, {-3, 8}, {-2, 7}, {-3, -4}, {-4, -2}, {-3, -2},
        {-1, -1}, {-1, -1}, {-2, -3}, {-2, -4}, {4, -3}, {7, -5}, {7, -3},
        {10, -2}, {11, -1} };
    double scout2_matrix[16][2] =
        { {-1, 5}, {-3, 10}, {-3, 9}, {-3, 7}, {-3, 7}, {-4, -2}, {-4, -2},
        {-2, -2}, {-1, -1}, {-2, -2}, {-2, -4}, {-2, -4}, {9, -2}, {10, -4},
        {8, -3}, {12, -2} };

    char sensors_name[5];
    double (*temp_matrix)[2];

    camera_enabled = 0;
    range = RANGE;

    /* 
     * Here we adapt the generic variables to the specificity of the current 
     * robot. We need to adapt the number of sensors it has, their name and
     * finally the matrix used by the Braitenberg algorithm.
     */
    if (strcmp(robot_name, "alice") == 0) {
        num_sensors = 2;
        sprintf(sensors_name, "%s", alice_name);
        temp_matrix = alice_matrix;
    } else if (strncmp(robot_name, "e-puck", 6) == 0) {
        num_sensors = 8;
        sprintf(sensors_name, "%s", e_puck_name);
        temp_matrix = e_puck_matrix;

        if (strcmp(robot_name, "e-puck camera") == 0) {
            camera_enabled = 1;
        }
    } else if (strncmp(robot_name, "khepera3", 8) == 0) {
        num_sensors = 9;
        sprintf(sensors_name, "%s", khepera_name);
        temp_matrix = khepera3_matrix;
        range = 2000;
    } else if (strncmp(robot_name, "khepera", 7) == 0) {
        num_sensors = 8;
        sprintf(sensors_name, "%s", khepera_name);
        temp_matrix = khepera_matrix;
    } else if (strcmp(robot_name, "koala") == 0) {
        num_sensors = 16;
        sprintf(sensors_name, "%s", koala_name);
        temp_matrix = koala_matrix;
    } else if (strcmp(robot_name, "magellan") == 0) {
        num_sensors = 16;
        sprintf(sensors_name, "%s", magellan_name);
        temp_matrix = magellan_matrix;
    } else if (strcmp(robot_name, "pioneer2") == 0) {
        num_sensors = 16;
        sprintf(sensors_name, "%s", pioneer2_name);
        temp_matrix = pioneer2_matrix;
    } else if (strcmp(robot_name, "scout2") == 0) {
        num_sensors = 16;
        sprintf(sensors_name, "%s", scout2_name);
        temp_matrix = scout2_matrix;
    }

    int i;
    for (i = 0; i < num_sensors; i++) {
        sensors[i] = wb_robot_get_device(sensors_name);
        wb_distance_sensor_enable(sensors[i], time_step);

        if ((i + 1) >= 10) {
            sensors_name[2] = '1';
            sensors_name[3]++;

            if ((i + 1) == 10) {
                sensors_name[3] = '0';
                sensors_name[4] = '\0';
            }
        } else {
            sensors_name[2]++;
        }

        int j;
        for (j = 0; j < 2; j++) {
            matrix[i][j] = temp_matrix[i][j];
        }
    }

    if (camera_enabled == 1) {
        camera = wb_robot_get_device("camera");
        wb_camera_enable(camera, time_step);
        wb_camera_move_window(camera, 0, 0);
    }

    printf("The %s robot is initialized, it uses %d distance sensors\n", robot_name, num_sensors);
}

int main()
{
  initialize();
  
  while (1) {
    int i, j;
    double speed[2];
    double sensors_value[MAX_SENSOR_NUMBER];

    /* If there is a camera, we need to update refresh it. */
    if (camera_enabled == 1) {
        wb_camera_get_image(camera);
    }

    for (i = 0; i < num_sensors; i++) {
        sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    /*
     * The Braitenberg algorithm is really simple, it simply computes the
     * speed of each wheel by summing the value of each sensor multiplied by
     * its corresponding weight. That is why each sensor must have a weight 
     * for each wheel.
     */
    for (i = 0; i < 2; i++) {
        speed[i] = 0.0;

        for (j = 0; j < num_sensors; j++) {

            /* 
             * We need to recenter the value of the sensor to be able to get
             * negative values too. This will allow the wheels to go 
             * backward too.
             */
            speed[i] += matrix[j][i] * (1.0 - (sensors_value[j] / range));
        }
    }

    /* Set the motor speeds */
    wb_differential_wheels_set_speed(speed[0], speed[1]);
    
    /* Run simulation */
    wb_robot_step(time_step);
  }
  
  return 0;
}
