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
int Navigation::SetNextTarget(NavigationState& _state) {
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

    return 0;
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