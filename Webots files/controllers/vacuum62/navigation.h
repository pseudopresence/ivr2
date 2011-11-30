/* 
 * File:   navigation.h
 * Author: s1149322
 *
 * Created on 22 November 2011, 14:42
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

#include "util.h"
#include "vec2.h"
#include "constants.h"

//Number of targets in one direction of movement
#define NAV_TARGET_COUNT 12

enum Direction {
    UP = 0, LEFT, DOWN, RIGHT
};

inline Direction operator++ (Direction& _d, int)
{
  _d = (Direction)wrap((int)_d + 1, 0, 3);
  return _d;
}

inline Direction operator-- (Direction& _d, int)
{
  _d = (Direction)wrap((int)_d - 1, 0, 3);
  return _d;
}

struct NavigationState {

    NavigationState(Direction const _dir, double const _offset, Vec2 const _targetPos) :
    m_dir(_dir),
    m_offset(_offset),
    m_targetPos(_targetPos) {
    }

    Direction m_dir;
    double m_offset;
    Vec2 m_targetPos;
};

class Navigation {
private:
    double m_offset;
    int m_targetIndex;
    double m_targetStartTime;

public:

    Navigation() :
    m_offset(0),
    m_targetIndex(0),
    m_targetStartTime(0) {
    }

    double GetTargetStartTime() const {
        return m_targetStartTime;
    }

    /* Calculates the coordinates of the next target position
       and returns the index of the target in the direction of movement
     */
    int SetNextTarget(NavigationState& _state) {
        m_targetIndex = wrap(++m_targetIndex, 1, NAV_TARGET_COUNT);
        // m_targetStartTime = wb_robot_get_time();

        double targetX = 0;
        double targetY = 0;
        
        if (m_targetIndex == 1)
        {          
          _state.m_dir++;
        }

        //Detect the next target depending on which direction the robot comes from
        switch (_state.m_dir) {
            case UP:

                targetX = MAX_ROOM_SIZE - m_offset;
                targetY = m_offset;

                if (m_targetIndex < NAV_TARGET_COUNT) {
                    //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                    targetX = m_offset + (targetX - m_offset) * (double)m_targetIndex / NAV_TARGET_COUNT;
                }

                break;

            case LEFT:
                targetX = targetY = MAX_ROOM_SIZE - m_offset;

                if (m_targetIndex < NAV_TARGET_COUNT) {
                    //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                    targetY = m_offset + (targetY - m_offset) * (double) m_targetIndex / NAV_TARGET_COUNT;
                }

                break;

            case DOWN:
                targetX = m_offset;
                targetY = MAX_ROOM_SIZE - m_offset;

                if (m_targetIndex < NAV_TARGET_COUNT) {
                    //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                    targetX = MAX_ROOM_SIZE - m_offset - (double)(MAX_ROOM_SIZE - m_offset - targetX) * m_targetIndex / (double) NAV_TARGET_COUNT;
                }

                break;

            case RIGHT:
                targetX = m_offset;

                if (m_targetIndex == NAV_TARGET_COUNT) {
                    //TODO: Replace this to ROBOT_RADIUS, in order to get a more thorough coverage
                    m_offset += ROBOT_DIAMETER;
                }

                targetY = m_offset;

                if (m_targetIndex < NAV_TARGET_COUNT) {
                    //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                    targetY = MAX_ROOM_SIZE - m_offset - (double)(MAX_ROOM_SIZE - m_offset - targetY - ROBOT_DIAMETER) * m_targetIndex / (double) NAV_TARGET_COUNT;
                }

                break;
        }

        Vec2 newTargetPos = Vec2(targetX, targetY);

        _state.m_offset = m_offset;
        _state.m_targetPos = newTargetPos;

        // printf("Target index: %d\n", m_targetIndex);
        // printf("Current target: %f %f\n", newTargetPos.m_x, newTargetPos.m_y);

        return m_targetIndex;
    }
};

#endif	/* NAVIGATION_H */

