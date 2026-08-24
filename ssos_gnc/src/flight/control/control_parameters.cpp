#include "ssos_gnc/flight/control/control_parameters.hpp"

#include <cmath>

namespace ssos_gnc
{

namespace flight
{

double FilterParams::alpha_from_tau(double tau_s, double dt_s)
{
  if (tau_s <= 0.0 || dt_s <= 0.0) {return 0.0;}
  return std::exp(-dt_s / tau_s);
}  // namespace flight

GncParameters default_gnc_parameters()
{
  return GncParameters{};
}  // namespace ssos_gnc
}
}
