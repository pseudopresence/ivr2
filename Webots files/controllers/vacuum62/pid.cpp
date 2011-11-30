#include "pid.h"

PIDController::PIDController( double _Kp, double _Ki, double _Kd ) :
m_Kp(_Kp),
m_Ki(_Ki),
m_Kd(_Kd),
m_prev(0),
m_sum(0),
m_firstStep(true) {
}

/* Takes the target and measured values. Returns the control parameter value. */
double PIDController::Step(double _goal, double _curr) {
  // There's no sensible initialisation value of m_prev before the first Step(), so have a special case
  if (m_firstStep) {
    m_prev = _curr;
    m_firstStep = false;
  }

  double const err = _goal - _curr;
  m_sum += err;

  double const v = m_Kp * err + m_Ki * m_sum + m_Kd * (_curr - m_prev);

  m_prev = _curr;
  return v;
}
