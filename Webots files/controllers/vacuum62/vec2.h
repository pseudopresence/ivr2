#ifndef _VEC2_H
#define _VEC2_H

#include <math.h>

class Vec2
{
public:
  double m_x;
  double m_y;

  Vec2() : m_x(0), m_y(0) {}
  Vec2(double const _x, double const _y) : m_x(_x), m_y(_y) {}
  Vec2(Vec2 const& _v) : m_x(_v.m_x), m_y(_v.m_y) {}

  Vec2 const& operator=(Vec2 const& _v) { m_x = _v.m_x; m_y = _v.m_y; return *this; }

  static Vec2 XAxis() { return Vec2(1, 0); }
  static Vec2 YAxis() { return Vec2(0, 1); }

  double GetLengthSquared() const
  {
    return m_x * m_x + m_y * m_y;
  }
  
  double GetLength() const
  {
    return sqrt(GetLengthSquared());
  }
  
  double GetDir() const { return atan2(m_y, m_x); }
  
  Vec2& operator+=(Vec2 const& _r);
  Vec2& operator-=(Vec2 const& _r);
  Vec2& operator*=(double const _s);
  
  inline static Vec2 UnitFromDir(double const _dir);
  inline static Vec2 FromDirLen(double const _dir, double const _len );
  
  Vec2 RotatedBy(double const _theta) const
  {
    return Vec2(
        m_x * cos(_theta) - m_y * sin(_theta),
        m_x * sin(_theta) + m_y * cos(_theta)
    );
  }
};

inline Vec2 operator+(Vec2 const& _l, Vec2 const& _r)
{
  return Vec2(_l.m_x + _r.m_x, _l.m_y + _r.m_y);
}

inline Vec2 operator-(Vec2 const& _l, Vec2 const& _r)
{
  return Vec2(_l.m_x - _r.m_x, _l.m_y - _r.m_y);
}

inline Vec2 operator*(double const _s, Vec2 const& _v)
{
  return Vec2(_v.m_x * _s, _v.m_y * _s);
}

inline Vec2 operator*(Vec2 const& _v, double const _s)
{
  return _s * _v;
}

inline Vec2& Vec2::operator+=(Vec2 const& _r)
{
  *this = *this + _r;
  return *this;
}

inline Vec2& Vec2::operator-=(Vec2 const& _r)
{
  *this = *this - _r;
  return *this;
}

inline Vec2& Vec2::operator*=(double const _s)
{
  *this = *this * _s;
  return *this;
}

Vec2 Vec2::UnitFromDir(double const _dir) { return Vec2(cos(_dir), sin(_dir)); }

Vec2 Vec2::FromDirLen(double const _dir, double const _len ) { return _len * UnitFromDir(_dir); }

#endif // _VEC2_H
