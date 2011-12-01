#include "navigation.h"

NavigationState::NavigationState(Direction const _dir, double const _offset, Vec2 const _targetPos) :
m_dir(_dir),
m_offset(_offset),
m_targetPos(_targetPos),
m_targetType(NORMAL) {
}

Navigation::Navigation() :
m_offsetWidth(ROBOT_DIAMETER),
m_offset(0),
m_targetCount(NAV_TARGET_COUNT),
m_targetIndex(0) {
}

/* Calculates the coordinates of the next target position
   and returns the index of the target in the direction of movement
 */
int Navigation::SetNextSpiralTarget(NavigationState& _state) {
    if (m_targetCount > 0) {
        if (((m_targetIndex == 0) || (m_targetIndex == m_targetCount))) {
            //If the robot is just about to start in a new direction,
            //calculate the appropriate number of steps, so that the distance between the targets is not too small

            m_targetIndex = 1;
            _state.m_dir++;

            double offset = 2 * m_offset;

            if (_state.m_dir == RIGHT) {
                //When moving to the right, the path length decreases
                offset += m_offsetWidth;
            }

            while ((m_targetCount > 0) && ((MAX_ROOM_SIZE - offset) / m_targetCount <= ROBOT_RADIUS)) {
                m_targetCount--;
            }
        } else {
            m_targetIndex++;
        }

        if (m_targetCount > 0) {
            double targetX = 0;
            double targetY = 0;

            //Detect the next target depending on which direction the robot comes from
            switch (_state.m_dir) {
                case UP:

                    targetX = MAX_ROOM_SIZE - m_offset;
                    targetY = m_offset;

                    if (m_targetIndex < m_targetCount) {
                        //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                        targetX = m_offset + (targetX - m_offset) * m_targetIndex / (double) m_targetCount;
                    }

                    break;

                case LEFT:
                    targetX = targetY = MAX_ROOM_SIZE - m_offset;

                    if (m_targetIndex < m_targetCount) {
                        //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                        targetY = m_offset + (targetY - m_offset) * m_targetIndex / (double) m_targetCount;
                    }

                    break;

                case DOWN:
                    targetX = m_offset;
                    targetY = MAX_ROOM_SIZE - m_offset;

                    if (m_targetIndex < m_targetCount) {
                        //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                        targetX = MAX_ROOM_SIZE - m_offset - (MAX_ROOM_SIZE - m_offset - targetX) * m_targetIndex / (double) m_targetCount;
                    }

                    break;

                case RIGHT:
                    targetX = m_offset;

                    if (m_targetIndex == m_targetCount) {
                        m_offset += m_offsetWidth;
                    }

                    targetY = m_offset;

                    if (m_targetIndex < m_targetCount) {
                        //If it's an intermediate target (not at the vertices of the spiral), calculate it relative to the starting vertex
                        targetY = MAX_ROOM_SIZE - m_offset - (MAX_ROOM_SIZE - m_offset - targetY - m_offsetWidth) * m_targetIndex / (double) m_targetCount;
                    }

                    break;
            }

            Vec2 newTargetPos = Vec2(targetX, targetY);

            _state.m_offset = m_offset;
            _state.m_targetPos = newTargetPos;

            return m_targetIndex;
        }
    }

    m_targetIndex = 0;

    return m_targetIndex;
}

void Navigation::SetNextHomingTarget(NavigationState& _state, Vec2 _start, Vec2 _end, double& _distance, int _index, int& _count) {
    if (_count > 0) { 
        if (_index == 1) {
            //Get the direction of movement
            if (_start.m_x == _end.m_x) {
                _distance = _end.m_y - _start.m_y;

                if (_distance > 0) {
                    _state.m_dir = LEFT;
                } else {
                    _state.m_dir = RIGHT;
                    _distance = -_distance;
                }

                _state.m_targetPos.m_x = _start.m_x;
            } else {
                _distance = _end.m_x - _start.m_x;

                if (_distance > 0) {
                    _state.m_dir = UP;
                } else {
                    _state.m_dir = DOWN;
                    _distance = -_distance;
                }

                _state.m_targetPos.m_y = _start.m_y;
            }

            while ((_count > 0) && (_distance / _count <= ROBOT_RADIUS)) {
                _count--;
            }
            
            // printf("Distance: %f\n", _distance);
            // printf("Start target: %f %f %d %d\n", _start.m_x, _start.m_y, _index, _count);
        }

        if (_count > 0) {            
            //Detect the next target depending on which direction the robot comes from
            switch (_state.m_dir) {
                case UP:
                    _state.m_targetPos.m_x = _start.m_x + _index * _distance / _count;
                    break;

                case LEFT:
                    _state.m_targetPos.m_y = _start.m_y + _index * _distance / _count;

                    break;

                case DOWN:
                    _state.m_targetPos.m_x = _start.m_x - _index * _distance / _count;

                    break;

                case RIGHT:
                    _state.m_targetPos.m_y = _start.m_y - _index * _distance / _count;

                    break;
            }
            
            // printf("Homing target %d: %f %f [direction=%d]\n", _index, _state.m_targetPos.m_x, _state.m_targetPos.m_y, _state.m_dir);
        }
    }
}

void Navigation::GetHomingTargets(Vec2* _results, Vec2 _current, Vec2 _home, int& count) {
    _results[0] = _current;

    if (fabs(_home.m_x - _current.m_x) <= fabs(_home.m_y - _current.m_y)) {
        _results[1].m_x = _current.m_x;
        _results[1].m_y = _home.m_y;
    } else {
        _results[1].m_x = _home.m_x;
        _results[1].m_y = _current.m_y;
    }

    _results[2] = _home;
    
    // printf("Homing target %d: %f %f\n", 1, _results[0].m_x, _results[0].m_y);
    // printf("Homing target %d: %f %f\n", 2, _results[1].m_x, _results[1].m_y);
    // printf("Homing target %d: %f %f\n", 3, _results[2].m_x, _results[2].m_y);

    count = 3;
}

Direction Navigation::GetDirectionByAngle(double angle) {
    double const stepAngle = M_PI / 4;

    if ((angle <= stepAngle) || (angle > 7 * stepAngle)) {
        return UP;
    } else if ((angle > stepAngle) && (angle <= 3 * stepAngle)) {
        return LEFT;
    } else if (angle <= 5 * stepAngle) {
        return DOWN;
    } else {
        return RIGHT;
    }
}