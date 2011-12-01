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

#include <stdio.h>

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
    double m_offsetWidth;
    double m_offset;
    int m_targetCount;
    int m_targetIndex;

public:
    Navigation();

    /* Calculates the coordinates of the next target position
       and returns the index of the target in the direction of movement
     */
    int SetNextSpiralTarget(NavigationState& _state);
    void SetNextHomingTarget(NavigationState& _state, Vec2 _start, Vec2 _end, double& _distance, int _index, int& _count);
    Vec2* GetHomingTargets(Vec2 _current, Vec2 _home, int& count);
    Direction GetDirectionByAngle(double angle);
};

#endif	/* NAVIGATION_H */

