#ifndef SSOS_ECLSS__COMMON__UNITS_HPP_
#define SSOS_ECLSS__COMMON__UNITS_HPP_

// Physical constants and unit conversions for the ECLSS physics library.
// Header-only, zero dependencies. All SI internally; conversions provided
// for the imperial/legacy units used in the ICES source papers.

namespace ssos_eclss
{
namespace units
{

// ---- Universal physical constants ----
constexpr double R_GAS = 8.31446;       // Universal gas constant [J/(mol*K)]
constexpr double FARADAY = 96485.0;     // Faraday constant [C/mol]
constexpr double AVOGADRO = 6.02214076e23;  // [1/mol]
constexpr double G0 = 9.80665;          // Standard gravity [m/s^2]
constexpr double T0_KELVIN = 273.15;    // 0 degC in Kelvin
constexpr double STD_PRESSURE_PA = 101325.0;  // Standard atmosphere [Pa]

// ---- Molar masses [kg/mol] ----
constexpr double M_AIR = 0.029;
constexpr double M_CO2 = 0.044;
constexpr double M_H2O = 0.018;
constexpr double M_O2 = 0.032;
constexpr double M_H2 = 0.002;
constexpr double M_N2 = 0.028;
constexpr double M_CH4 = 0.016;

// ---- Pressure conversion factors ----
constexpr double TORR_TO_PA = 133.322;          // 1 torr -> Pa
constexpr double PA_TO_TORR = 1.0 / TORR_TO_PA;
constexpr double IN_H2O_TO_PA = 249.089;        // 1 inch of water -> Pa
constexpr double PA_TO_IN_H2O = 1.0 / IN_H2O_TO_PA;
constexpr double KPA_TO_PA = 1000.0;
constexpr double PA_TO_KPA = 1.0 / KPA_TO_PA;

// ---- Flow conversion factors ----
constexpr double SCFM_TO_M3S = 4.7195e-4;       // standard cubic ft/min -> m^3/s
constexpr double M3S_TO_SCFM = 1.0 / SCFM_TO_M3S;
constexpr double GPM_TO_M3S = 6.30902e-5;       // US gallon/min -> m^3/s
constexpr double M3S_TO_GPM = 1.0 / GPM_TO_M3S;

// ---- Time ----
constexpr double SECONDS_PER_DAY = 86400.0;
constexpr double SECONDS_PER_HOUR = 3600.0;
constexpr double SECONDS_PER_MINUTE = 60.0;

// ---- Temperature conversions (functions: not all are affine-only) ----
inline double celsius_to_kelvin(double c) { return c + T0_KELVIN; }
inline double kelvin_to_celsius(double k) { return k - T0_KELVIN; }
inline double fahrenheit_to_kelvin(double f) { return (f - 32.0) * 5.0 / 9.0 + T0_KELVIN; }
inline double kelvin_to_fahrenheit(double k) { return (k - T0_KELVIN) * 9.0 / 5.0 + 32.0; }
inline double fahrenheit_to_celsius(double f) { return (f - 32.0) * 5.0 / 9.0; }
inline double celsius_to_fahrenheit(double c) { return c * 9.0 / 5.0 + 32.0; }

// ---- Pressure helpers ----
inline double torr_to_pa(double torr) { return torr * TORR_TO_PA; }
inline double pa_to_torr(double pa) { return pa * PA_TO_TORR; }
inline double in_h2o_to_pa(double in) { return in * IN_H2O_TO_PA; }
inline double pa_to_in_h2o(double pa) { return pa * PA_TO_IN_H2O; }
inline double kpa_to_pa(double kpa) { return kpa * KPA_TO_PA; }
inline double pa_to_kpa(double pa) { return pa * PA_TO_KPA; }

// ---- Flow helpers ----
inline double scfm_to_m3s(double scfm) { return scfm * SCFM_TO_M3S; }
inline double m3s_to_scfm(double m3s) { return m3s * M3S_TO_SCFM; }
inline double gpm_to_m3s(double gpm) { return gpm * GPM_TO_M3S; }
inline double m3s_to_gpm(double m3s) { return m3s * M3S_TO_GPM; }

// ---- Concentration helpers ----
inline double ppm_to_fraction(double ppm) { return ppm * 1.0e-6; }
inline double fraction_to_ppm(double frac) { return frac * 1.0e6; }

// ---- Mass rate helpers ----
inline double kg_per_day_to_kg_per_s(double kg_day) { return kg_day / SECONDS_PER_DAY; }
inline double kg_per_s_to_kg_per_day(double kg_s) { return kg_s * SECONDS_PER_DAY; }

}  // namespace units
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__UNITS_HPP_
