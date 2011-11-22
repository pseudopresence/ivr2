#ifndef PID_H
#define	PID_H

class PIDController
{
  public:
    PIDController(double _Kp, double _Ki, double _Kd) : m_Kp(_Kp), m_Ki(_Ki), m_Kd(_Kd), m_prev(0), m_sum(0), m_firstStep(true) {}

    double Step(double _goal, double _curr)
    {
      // There's no sensible initialisation value of m_prev before the first Step(), so have a special case
      if (m_firstStep)
      {
        m_prev = _curr;
        m_firstStep = false;
      }

      double const e = _goal - _curr;
      m_sum += e;

      double const v = m_Kp * e + m_Ki * m_sum + m_Kd * (_curr - m_prev);
      
      m_prev = _curr;
      return v;
    }
  private:
    double const m_Kp;
    double const m_Ki;
    double const m_Kd;

    double m_prev;
    double m_sum;

    bool m_firstStep;
};

#endif	/* PID_H */

