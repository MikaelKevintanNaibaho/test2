#ifndef INTERPOLATION_HPP_
#define INTERPOLATION_HPP_

#include <assert.h>

#include <type_traits>

namespace interpolate
{
// linear interpolation between y0 and yf. x is between 0 and 1
template <typename y_t, typename x_t>
y_t linear_interp(y_t y0, y_t y_f, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value, "must be floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (y_f - y0) * x;
}

// cubic bezier interpolation between y0 and yf. x is between 0 and 1
template <typename y_t, typename x_t>
y_t cubic_bezier_interp(y_t y0, y_t y_f, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value, "must be floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = y_f - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

// cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
template <typename y_t, typename x_t>
y_t cubic_bezier_first_derevative_interp(y_t y0, y_t y_f, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value, "must be floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = y_f - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

// cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
template <typename y_t, typename x_t>
y_t cubic_bezier_second_derevative_interp(y_t y0, y_t y_f, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value, "must be floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = y_f - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}
}  // Namespace interpolate

#endif  // INTERPOLATION_HPP_
