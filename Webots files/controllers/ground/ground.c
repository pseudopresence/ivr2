/*
 * File:         ground.c
 * Date:         February 21nd, 2011
 * Description:  This supervisor track the absolute position
 *               of the robot and remove the dirty from the
 *               area given by the robot
 * Author:       fabien.rohrer@cyberbotics.com
 *
 * Copyright (c) 2010 Cyberbotics - www.cyberbotics.com
 */
#include <stdlib.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/display.h>

#define TIME_STEP  64

#define X 0
#define Y 1
#define Z 2

// size of the ground
#define GROUND_X   6
#define GROUND_Z   6

// main function
int main() {

  // init Webtos stuff
  wb_robot_init();

  // First we get a handler to devices
  WbDeviceTag display = wb_robot_get_device("ground_display");
  
  // get the properties of the Display
  int width = wb_display_get_width(display);
  int height = wb_display_get_height(display);
  
  // prepare stuff to get the
  // DifferentialWheels(IROBOT_CREATE).translation field
  int nn,nbots=0;
  WbNodeRef mybots[3];
  WbFieldRef translationField[3];
  WbNodeRef mybot = wb_supervisor_node_get_from_def("VACU1");
  if (mybot) {
    mybots[0]=mybot;
    nbots++;
  }
  mybot = wb_supervisor_node_get_from_def("VACU2");
  if (mybot) {
    mybots[nbots]=mybot;
    nbots++;
  }
  mybot = wb_supervisor_node_get_from_def("VACU3");
  if (mybot) {
    mybots[nbots]=mybot;
    nbots++;
  }
  if (nbots==0) printf("there aren't any robots\n");
  if (nbots==1) printf("there is now %d robot\n",nbots);
  if (nbots>1) printf("there are now %d robots\n",nbots);
  const double *translation[3];
  for (nn=0;nn<nbots;nn++) {
  translationField[nn] = wb_supervisor_node_get_field(mybots[nn],"translation");
  }
  // set the background (otherwise an empty ground is displayed at this step)
  WbImageRef background = wb_display_image_load(display, "../../worlds/textures/dirty.png");
  wb_display_image_paste(display, background, 0, 0);
  
  // set the pen to remove the texture
  wb_display_set_alpha(display, 0.0);
  
  while(wb_robot_step(TIME_STEP)!=-1) {
  
    // Update the translation field
    for (nn=0;nn<nbots;nn++) {
    translation[nn] = wb_supervisor_field_get_sf_vec3f(translationField[nn]);
    
    // display the robot position
    wb_display_fill_oval(
      display,
      width*(translation[nn][X]+GROUND_X/2)/GROUND_X,
      height*(translation[nn][Z]+GROUND_Z/2)/GROUND_Z,
      15,
      15);
  }
  }
  wb_robot_cleanup();

  return 0;
}
