#ifndef SSOS_ECLSS__COMMON__HEAT_TRANSFER_HPP_
#define SSOS_ECLSS__COMMON__HEAT_TRANSFER_HPP_

// Convective correlations and heat-exchanger effectiveness. No ROS, no deps.

namespace ssos_eclss
{
namespace heat
{

/// Prandtl number Pr = cp * mu / k [-].
double prandtl_number(double cp, double viscosity, double thermal_cond);

/// Wakao-Kaguei Nusselt correlation for gas-solid heat transfer in packed beds:
/// Nu = 2 + 1.1 * Re^0.6 * Pr^(1/3)
/// @param reynolds particle Reynolds number [-]
/// @param prandtl  Prandtl number [-]
double nusselt_wakao_kaguei(double reynolds, double prandtl);

/// Gas-solid heat transfer coefficient [W/(m^2*K)] from the Nusselt number.
/// h = Nu * k / dp
/// @param nusselt      Nusselt number [-]
/// @param thermal_cond gas thermal conductivity [W/(m*K)]
/// @param particle_d   particle diameter [m]
double gas_solid_htc(double nusselt, double thermal_cond, double particle_d);

/// Specific (per unit bed volume) interfacial area of a packed bed [m^2/m^3].
/// a_v = 6 * (1 - voidage) / dp  (spherical particles)
double specific_surface_area(double voidage, double particle_d);

/// Number of transfer units NTU = U*A / C_min [-].
double ntu(double ua, double c_min);

/// Effectiveness of a counter-flow heat exchanger via the epsilon-NTU method.
/// @param ntu_val  number of transfer units [-]
/// @param cr       capacity-rate ratio C_min/C_max [-] (0..1)
double effectiveness_counterflow(double ntu_val, double cr);

/// Effectiveness of a cross-flow (both fluids unmixed) heat exchanger.
double effectiveness_crossflow(double ntu_val, double cr);

/// Heat duty [W] from effectiveness, C_min and inlet temperature difference.
/// q = eps * C_min * (T_hot_in - T_cold_in)
double hx_heat_duty(double effectiveness, double c_min,
                    double t_hot_in, double t_cold_in);

}  // namespace heat
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__HEAT_TRANSFER_HPP_
