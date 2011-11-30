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

enum Direction {
    UP = 0, LEFT, DOWN, RIGHT
};

enum TargetType {
    NORMAL = 0, AVOIDANCE, HOME
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
    NavigationState(Direction const _dir, double const _offset, Vec2 const _targetPos);

    Direction m_dir;
    double m_offset;
    Vec2 m_targetPos;
    TargetType m_targetType;
};

class Navigation {
private:
    double m_offset;
    int m_targetIndex;

public:
    Navigation();

    /* Calculates the coordinates of the next target position
       and returns the index of the target in the direction of movement
     */
    int SetNextTarget(NavigationState& _state);
};

#endif	/* NAVIGATION_H */

