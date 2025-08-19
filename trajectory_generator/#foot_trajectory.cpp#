#include "foot_trajectory.hpp"

#include "interpolation.hpp"

template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swing_time)
{
  // Compute position using Bezier curve for XY
  _p = interpolate::cubic_bezier_interp<Vec3<T>>(_p0, _pf, phase);
  _v = interpolate::cubic_bezier_first_derevative_interp<Vec3<T>>(_p0, _pf, phase) / swing_time;
  _a = interpolate::cubic_bezier_second_derevative_interp<Vec3<T>>(_p0, _pf, phase) /
       (swing_time * swing_time);

  T zp, zv, za;
  if (phase < T(0.5)) {
    // First half: lifting phase
    zp = interpolate::cubic_bezier_interp<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = interpolate::cubic_bezier_first_derevative_interp<T>(_p0[2], _p0[2] + _height, phase * 2) *
         2 / swing_time;
    za =
      interpolate::cubic_bezier_second_derevative_interp<T>(_p0[2], _p0[2] + _height, phase * 2) *
      4 / (swing_time * swing_time);
  } else {
    // Second half: lowering phase
    zp = interpolate::cubic_bezier_interp<T>(_p0[2] + _height, _pf[2], (phase - 0.5) * 2);
    zv = interpolate::cubic_bezier_first_derevative_interp<T>(
           _p0[2] + _height, _pf[2], (phase - 0.5) * 2) *
         2 / swing_time;
    za = interpolate::cubic_bezier_second_derevative_interp<T>(
           _p0[2] + _height, _pf[2], (phase - 0.5) * 2) *
         4 / (swing_time * swing_time);
  }

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}

// Explicit template instantiation
template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;
