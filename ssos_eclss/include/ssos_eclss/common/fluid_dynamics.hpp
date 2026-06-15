#ifndef SSOS_ECLSS__COMMON__FLUID_DYNAMICS_HPP_
#define SSOS_ECLSS__COMMON__FLUID_DYNAMICS_HPP_

// Packed-bed and pipe-flow hydraulics. No ROS, no external deps.

namespace ssos_eclss
{
namespace fluid
{

/// Particle Reynolds number for flow through a packed bed.
/// Re = rho * v * dp / mu  (superficial velocity basis)
/// @param density     fluid density [kg/m^3]
/// @param velocity    superficial velocity [m/s]
/// @param particle_d  particle diameter [m]
/// @param viscosity   dynamic viscosity [Pa*s]
double reynolds_number(double density, double velocity, double particle_d,
                       double viscosity);

/// Ergun-equation pressure gradient through a packed bed [Pa/m].
/// dP/dz = 150*(1-e)^2/e^3 * mu*v/dp^2 + 1.75*(1-e)/e^3 * rho*v^2/dp
/// @param velocity    superficial velocity [m/s]
/// @param voidage     bed void fraction (porosity) [-]
/// @param particle_d  particle diameter [m]
/// @param density     fluid density [kg/m^3]
/// @param viscosity   dynamic viscosity [Pa*s]
double ergun_pressure_gradient(double velocity, double voidage, double particle_d,
                               double density, double viscosity);

/// Total Ergun pressure drop across a bed of given length [Pa].
double ergun_pressure_drop(double velocity, double voidage, double particle_d,
                           double density, double viscosity, double bed_length);

/// Darcy friction factor for pipe flow (laminar 64/Re, turbulent via
/// the explicit Haaland correlation).
/// @param reynolds        Reynolds number [-]
/// @param relative_rough  roughness / diameter [-]
double darcy_friction_factor(double reynolds, double relative_rough = 0.0);

/// Superficial velocity [m/s] from volumetric flow and cross-sectional area.
double superficial_velocity(double volumetric_flow_m3s, double cross_area_m2);

/// Interstitial (pore) velocity [m/s] = superficial / voidage.
double interstitial_velocity(double superficial_v, double voidage);

}  // namespace fluid
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__FLUID_DYNAMICS_HPP_
