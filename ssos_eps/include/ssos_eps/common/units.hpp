#ifndef SSOS_EPS__COMMON__UNITS_HPP_
#define SSOS_EPS__COMMON__UNITS_HPP_

// Physical constants and unit conversions used across the EPS physics.

namespace ssos_eps
{
namespace units
{

constexpr double PI = 3.14159265358979323846;
constexpr double SECONDS_PER_HOUR = 3600.0;
constexpr double KELVIN_OFFSET = 273.15;

constexpr double deg_to_rad(double deg) {return deg * PI / 180.0;}
constexpr double rad_to_deg(double rad) {return rad * 180.0 / PI;}

constexpr double celsius_to_kelvin(double c) {return c + KELVIN_OFFSET;}
constexpr double kelvin_to_celsius(double k) {return k - KELVIN_OFFSET;}

/// Joules from watt-hours.
constexpr double wh_to_j(double wh) {return wh * SECONDS_PER_HOUR;}

/// Watt-hours from joules.
constexpr double j_to_wh(double j) {return j / SECONDS_PER_HOUR;}

}  // namespace units
}  // namespace ssos_eps

#endif  // SSOS_EPS__COMMON__UNITS_HPP_
