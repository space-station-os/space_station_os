#ifndef SSOS_GNC__COMMON__UNITS_HPP_
#define SSOS_GNC__COMMON__UNITS_HPP_

#include <cmath>

namespace ssos_gnc
{

namespace common
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

constexpr double kEarthMu = 3.986004418e14;
constexpr double kEarthRadius = 6378137.0;
constexpr double kEarthRotRate = 7.292115e-5;

constexpr double kSolarPressure = 4.56e-6;

constexpr double kSpeedOfLight = 2.99792458e8;

inline constexpr double deg_to_rad(double deg) {return deg * kDegToRad;}
inline constexpr double rad_to_deg(double rad) {return rad * kRadToDeg;}
inline constexpr double km_to_m(double km) {return km * 1000.0;}
inline constexpr double m_to_km(double m) {return m / 1000.0;}

inline double wrap_pi(double angle)
{
  double a = std::fmod(angle + kPi, kTwoPi);
  if (a < 0.0) {a += kTwoPi;}
  return a - kPi;
}  // namespace common

inline double wrap_two_pi(double angle)
{
  double a = std::fmod(angle, kTwoPi);
  if (a < 0.0) {a += kTwoPi;}
  return a;
}  // namespace ssos_gnc

inline constexpr double clamp(double v, double lo, double hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}
}
}

#endif  // SSOS_GNC__COMMON__UNITS_HPP_
