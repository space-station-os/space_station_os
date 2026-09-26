#include "ssos_gnc/plant/disturbance_torques.hpp"

#include <cmath>

namespace ssos_gnc
{

namespace plant
{

DisturbanceTorques::DisturbanceTorques() = default;

DisturbanceTorques::DisturbanceTorques(const DisturbanceParams & params)
: params_(params)
{
}  // namespace plant

double DisturbanceTorques::atmospheric_density(double altitude_km)
{
  struct Band {double base_km; double density; double scale_height_km;};
  static const Band bands[] = {
    {200.0, 2.789e-10, 37.5},
    {250.0, 7.248e-11, 44.8},
    {300.0, 2.418e-11, 53.6},
    {350.0, 9.518e-12, 53.3},
    {400.0, 3.725e-12, 58.5},
    {450.0, 1.585e-12, 60.8},
    {500.0, 6.967e-13, 63.8},
    {600.0, 1.454e-13, 71.8},
    {700.0, 3.614e-14, 88.7},
  };
  constexpr int n = static_cast<int>(sizeof(bands) / sizeof(bands[0]));

  if (altitude_km <= bands[0].base_km) {
    return bands[0].density;
  }

  int idx = n - 1;
  for (int i = 0; i < n - 1; ++i) {
    if (altitude_km < bands[i + 1].base_km) {
      idx = i;
      break;
    }
  }

  const Band & b = bands[idx];
  return b.density * std::exp(-(altitude_km - b.base_km) / b.scale_height_km);
}  // namespace ssos_gnc

DisturbanceResult DisturbanceTorques::compute(
  const EnvironmentConditions & env, const Inertia & inertia) const
{
  DisturbanceResult out;

  const Quaternion q = common::quat_normalized(env.attitude_lvlh);
  const Eigen::Matrix3d r_body_lvlh = q.conjugate().toRotationMatrix();

  const Vector3 nadir_body = r_body_lvlh * Vector3(0.0, 0.0, 1.0);
  const Vector3 velocity_body = r_body_lvlh * Vector3(1.0, 0.0, 0.0);

  const double radius_m = common::kEarthRadius + common::km_to_m(env.altitude_km);

  if (params_.enable_gravity_gradient) {
    const double n = common::mean_motion(radius_m);
    out.gravity_gradient =
      3.0 * n * n * nadir_body.cross(inertia.tensor() * nadir_body);
  }

  if (params_.enable_aero_drag) {
    const double rho = atmospheric_density(env.altitude_km);
    const double v = std::sqrt(common::kEarthMu / radius_m);

    const double f_mag =
      0.5 * rho * v * v * params_.drag_coefficient * params_.frontal_area_m2;
    const Vector3 force = -f_mag * velocity_body;

    const Vector3 lever(0.0, 0.0, params_.center_of_pressure_offset_m);
    out.aero_drag = lever.cross(force);
  }

  if (params_.enable_srp && !env.in_eclipse) {
    const double pressure = env.solar_flux_w_m2 / common::kSpeedOfLight;
    const double f_mag = pressure * params_.srp_area_m2 * params_.srp_reflectivity;

    const Vector3 force = -f_mag * velocity_body;
    const Vector3 lever(0.0, params_.srp_center_offset_m, 0.0);
    out.solar_pressure = lever.cross(force);
  }

  out.total = out.gravity_gradient + out.aero_drag + out.solar_pressure;
  return out;
}
}
}
