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

#include "robot.h"

/* main */
int main(int argc, char **argv)
{
  Robot r(Vec2(0,0),0, 0);
 
  r.Init();
  r.Run();
  r.Shutdown();
  
  return EXIT_SUCCESS;
}
