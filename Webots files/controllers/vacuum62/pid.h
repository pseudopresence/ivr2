#ifndef PID_H
#define	PID_H

/* A straightforward implementation of PID control */
class PIDController
{
  public:
    PIDController(double _Kp, double _Ki, double _Kd);
    double Step(double _goal, double _curr);
    
  private:
    double const m_Kp;
    double const m_Ki;
    double const m_Kd;

    double m_prev;
    double m_sum;

    bool m_firstStep;
};

#endif	/* PID_H */

