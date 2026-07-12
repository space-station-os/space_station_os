#ifndef SSOS_ECLSS__COMMON__NUMERICAL__RK4_INTEGRATOR_HPP_
#define SSOS_ECLSS__COMMON__NUMERICAL__RK4_INTEGRATOR_HPP_

#include <cstddef>
#include <vector>

// Generic classic Runge-Kutta (RK4) stepper operating on a std::vector<double>
// state with a user-supplied derivative functor. Header-only, no dependencies.

namespace ssos_eclss
{
namespace numerical
{

using State = std::vector<double>;

/// Advance @p state by one RK4 step of size @p dt.
/// The derivative functor has signature: void(const State& x, double t, State& dxdt)
/// and must size/fill @p dxdt to match @p x.
/// @param deriv derivative functor
/// @param state state vector, updated in place
/// @param t     current time
/// @param dt    timestep
template <typename Deriv>
void rk4_step(Deriv && deriv, State & state, double t, double dt)
{
  const std::size_t n = state.size();
  State k1(n), k2(n), k3(n), k4(n), tmp(n);

  deriv(state, t, k1);
  for (std::size_t i = 0; i < n; ++i) {
    tmp[i] = state[i] + 0.5 * dt * k1[i];
  }
  deriv(tmp, t + 0.5 * dt, k2);
  for (std::size_t i = 0; i < n; ++i) {
    tmp[i] = state[i] + 0.5 * dt * k2[i];
  }
  deriv(tmp, t + 0.5 * dt, k3);
  for (std::size_t i = 0; i < n; ++i) {
    tmp[i] = state[i] + dt * k3[i];
  }
  deriv(tmp, t + dt, k4);

  const double sixth = dt / 6.0;
  for (std::size_t i = 0; i < n; ++i) {
    state[i] += sixth * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
}

/// Integrate from @p t0 to @p t0 + @p total_dt using @p n_sub equal RK4 substeps.
template <typename Deriv>
void rk4_integrate(Deriv && deriv, State & state, double t0, double total_dt,
                   std::size_t n_sub)
{
  if (n_sub == 0) {
    n_sub = 1;
  }
  const double h = total_dt / static_cast<double>(n_sub);
  double t = t0;
  for (std::size_t s = 0; s < n_sub; ++s) {
    rk4_step(deriv, state, t, h);
    t += h;
  }
}

}  // namespace numerical
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__NUMERICAL__RK4_INTEGRATOR_HPP_
