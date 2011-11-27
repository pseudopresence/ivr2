#ifndef _UTIL_H
#define _UTIL_H

/* random double [0.0 - 1.0] */
static double randdouble() {
  return rand() / ((double)RAND_MAX + 1);
}

/* wrap a value into the given range, for example -M_PI to M_PI */
static double wrap(double _x, double const _min, double const _max ) {
   
   while (_x < _min) 
   {
       _x += (_max - _min); 
   }
   
   while (_x >= _max) 
   {
       _x -= (_max - _min); 
   }
   
   return _x;
}

/* clamp a value to the given range, for example 0 to 1 */
static double clamp(double _x, double const _min, double const _max) {
  if ( _x < _min ) { return _min; }
  else if ( _max < _x ) { return _max; }
  else { return _x; }
}

static double max(double const _a, double const _b) {
   return (_a > _b) ? _a : _b;
}

static double min(double const _a, double const _b) {
   return (_a < _b) ? _a : _b;
}

/* smootherstep interpolation function from http://en.wikipedia.org/wiki/Smoothstep */
static float smootherstep(float edge0, float edge1, float x) {
  // Scale, and clamp x to 0..1 range
  x = clamp((x - edge0)/(edge1 - edge0), 0, 1);
  // Evaluate polynomial
  return x*x*x*(x*(x*6 - 15) + 10);
}



#endif // _UTIL_H
