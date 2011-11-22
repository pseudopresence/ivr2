/* 
 * File:   navigation.h
 * Author: s1149322
 *
 * Created on 22 November 2011, 14:42
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

#include "vec2.h"
#include "constants.h"

//Number of targets in one direction of movement
#define NAV_TARGET_COUNT 12

enum Direction { LEFT, UP, RIGHT, DOWN };

struct NavigationState
{
  NavigationState(Direction const _dir, Vec2 const _targetPos) :
    m_dir(_dir),
    m_targetPos(_targetPos)
  { }

  Direction m_dir;
  Vec2 m_targetPos;
};

class Navigation
{
private:
  double m_indent;
  int m_targetIndex;
  double m_targetStartTime;
  
public:
  Navigation():
    m_indent(0),
    m_targetIndex(0),
    m_targetStartTime(0)
  { }
 
  double GetTargetStartTime() const { return m_targetStartTime; }

  /* Calculates the coordinates of the next target position
     and returns the index of the target in the direction of movement
  */
  int SetNextTarget(NavigationState& _state )
  {
    m_targetIndex = wrap(++m_targetIndex, 0, NAV_TARGET_COUNT);
    m_targetStartTime = wb_robot_get_time();

    double targetX = 0;
    double targetY = 0;
    
    //Detect the next target depending on which direction the robot comes from
    switch (_state.m_dir)
    {
      case UP:
      
        targetX = MAX_ROOM_WIDTH - m_indent;
        targetY = m_indent;
        
        if (m_targetIndex < NAV_TARGET_COUNT)
        {         
          //If it's an intermediate target (non at the vertices of the spiral), calculate it relative to the starting vertex
          targetX = (targetX - m_indent) * (double)m_targetIndex / NAV_TARGET_COUNT;
        }
        else
        {
          _state.m_dir = LEFT;
        }
        
        break;
        
      case LEFT:
        targetX = MAX_ROOM_WIDTH - m_indent;
        targetY = MAX_ROOM_HEIGHT - m_indent;
        
        if (m_targetIndex < NAV_TARGET_COUNT)
        {
          //If it's an intermediate target (non at the vertices of the spiral), calculate it relative to the starting vertex
          targetY = (targetY - m_indent) * (double)m_targetIndex / NAV_TARGET_COUNT;
        }
        else
        {
          _state.m_dir = DOWN;
        }
        
        break;
        
      case DOWN:        
        targetX = m_indent;
        targetY = MAX_ROOM_HEIGHT - m_indent;
        
        if (m_targetIndex < NAV_TARGET_COUNT)
        {
          //If it's an intermediate target (non at the vertices of the spiral), calculate it relative to the starting vertex
          targetX = (MAX_ROOM_WIDTH - m_indent - targetX) * (1 - (double)m_targetIndex / (double)NAV_TARGET_COUNT);
        }
        else
        {
          _state.m_dir = RIGHT;
        }
        
        break;
        
      case RIGHT:
        targetX = m_indent;    
        
        if (m_targetIndex == 1)
        {
          m_indent += ROBOT_DIAMETER;
        }
        
        targetY = m_indent;
        
        if (m_targetIndex < NAV_TARGET_COUNT)
        {
          //If it's an intermediate target (non at the vertices of the spiral), calculate it relative to the starting vertex
          targetY = (MAX_ROOM_HEIGHT - m_indent - targetY + ROBOT_DIAMETER) * (1 - (double)m_targetIndex / (double)NAV_TARGET_COUNT);
        }
        else
        {
          _state.m_dir = UP;
        }
        
        break;
    }
    
    Vec2 newTargetPos = Vec2(targetX, targetY);
    
    _state.m_targetPos = newTargetPos;
    
    printf("Target index: %d\n", m_targetIndex);
    printf("Current target: %f %f\n", newTargetPos.m_x, newTargetPos.m_y);
    
    return m_targetIndex;
  }
};

#endif	/* NAVIGATION_H */

