/****************************************************************************

A simple controller for Webots.
 adapted from 
Copyright (C) 2006 Laboratory of Intelligent Systems, EPFL, Lausanne
Authors:    Alexis Guanella            guanella@ini.phys.ethz.ch
            Antoine Beyeler            antoine.beyeler@epfl.ch
            Jean-Christophe Zufferey   jean-christophe.zufferey@epfl.ch
            Dario Floreano             dario.floreano@epfl.ch
            Olivier Michel             Olivier.Michel@cyberbotics.com

Web: http://lis.epfl.ch

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/

#include <webots/robot.h>
#include <webots/emitter.h>

#include "js.h"

#define TIMESTEP 32 // ms
#define KEY_INCREMENT 0.05

int main()
{
  double command[3]={0.0, 0.0, 0.0};
  jsJoystick *gJoystick = NULL;
  WbDeviceTag gEmitter = 0;

  wb_robot_init();

	// Handle joystick
	gJoystick = new jsJoystick();
	if(gJoystick->notWorking())
	{
		delete gJoystick,
		gJoystick = NULL;
		printf("*** blimp_joystick :: reset :: could not connect to joystick, reverting to keyboard.\n");
		printf("available control keys: up, down, right, left, page up, page down and space (reset)\n");
		wb_robot_keyboard_enable(TIMESTEP);
	}

	// Handle emitter
	gEmitter = wb_robot_get_device("emitter");
	if(!gEmitter)
		printf("!!! blimp_joystick :: reset :: emitter is not available.\n");

  while(wb_robot_step(TIMESTEP)!=-1) {
  	// Send joystick value.
	  if(gEmitter){
		  float axes[12];
		  int buttons[12];
		  // read joystick.
	  	if (gJoystick) {
		    gJoystick->read(buttons, axes);
		    command[0] = (double) -axes[1];
		    command[1] = (double) axes[3];
		    command[2] = (double) axes[2];
      } else {
        switch(wb_robot_keyboard_get_key()) {
        case WB_ROBOT_KEYBOARD_DOWN:
          command[0]-=KEY_INCREMENT;
          break;
        case WB_ROBOT_KEYBOARD_UP:
          command[0]+=KEY_INCREMENT;
          break;
        case WB_ROBOT_KEYBOARD_LEFT:
          command[1]-=KEY_INCREMENT;
          break;
        case WB_ROBOT_KEYBOARD_RIGHT:
          command[1]+=KEY_INCREMENT;
          break;
        case WB_ROBOT_KEYBOARD_PAGEUP:
          command[2]-=KEY_INCREMENT;
          break;
        case WB_ROBOT_KEYBOARD_PAGEDOWN:
          command[2]+=KEY_INCREMENT;
          break;
        case ' ': // space -> reset
          command[0]=0.0;
          command[1]=0.0;
          command[2]=0.0;
        }
      }
		  // setup emitter buffer
      printf("command = ( %g , %g , %g )\n",command[0],command[1],command[2]);
		  wb_emitter_send(gEmitter, command, sizeof(command));
	  }
  }

  wb_robot_cleanup();
	return 0;
}
