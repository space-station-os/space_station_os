#ifndef SSOS_GNC__COMMON__NUMERICAL__RK4_INTEGRATOR_HPP_
#define SSOS_GNC__COMMON__NUMERICAL__RK4_INTEGRATOR_HPP_

#include <functional>

namespace ssos_gnc
{

namespace common
{

namespace numerical
{

template<typename State>
State rk4_step(
  const State & state,
  double t,
  double dt,
  const std::function<State(const State &, double)> & derivative)
{
  const State k1 = derivative(state, t);
  const State k2 = derivative(state + k1 * (0.5 * dt), t + 0.5 * dt);
  const State k3 = derivative(state + k2 * (0.5 * dt), t + 0.5 * dt);
  const State k4 = derivative(state + k3 * dt, t + dt);

  return state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}  // namespace numerical

template<typename State>
State rk4_integrate(
  const State & state,
  double t,
  double dt,
  int n_substeps,
  const std::function<State(const State &, double)> & derivative)
{
  if (n_substeps < 1) {n_substeps = 1;}
  const double h = dt / static_cast<double>(n_substeps);

  State s = state;
  double time = t;
  for (int i = 0; i < n_substeps; ++i) {
    s = rk4_step<State>(s, time, h, derivative);
    time += h;
  }
  return s;
}  // namespace common

inline int substeps_for(double dt, double max_step)
{
  if (max_step <= 0.0 || dt <= 0.0) {return 1;}
  const int n = static_cast<int>(dt / max_step) + 1;
  return (n < 1) ? 1 : n;
}  // namespace ssos_gnc
}
}
}

#endif  // SSOS_GNC__COMMON__NUMERICAL__RK4_INTEGRATOR_HPP_
