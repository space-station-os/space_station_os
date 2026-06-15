#ifndef SSOS_ECLSS__FAULTS__FAULT_TYPES_HPP_
#define SSOS_ECLSS__FAULTS__FAULT_TYPES_HPP_

#include <string>

// Physics-level fault taxonomy. Deliberately ROS-free: severity integer values
// mirror space_station_interfaces/msg/FaultEvent so the ROS layer can map them
// directly without this library depending on ROS. No external deps.

namespace ssos_eclss
{
namespace faults
{

/// Category of fault and the physical effect it induces.
enum class FaultType
{
  NONE,
  // Sensor faults (corrupt a reported value, not the physics)
  SENSOR_STUCK,
  SENSOR_DRIFT,
  SENSOR_BIAS,
  SENSOR_NOISE,
  SENSOR_SCALE,
  // Actuator faults (corrupt the physics)
  VALVE_STUCK_OPEN,
  VALVE_STUCK_CLOSED,
  BLOWER_DEGRADED,
  BLOWER_FAILED,
  PUMP_FAILED,
  // Thermal faults (corrupt the physics)
  HEATER_PARTIAL,
  HEATER_FAILED,
  PRECOOLER_DEGRADED,
  INSULATION_LOSS,
  // Environmental
  CABIN_LEAK
};

/// Severity mirrors FaultEvent.SEVERITY_* constants.
enum class FaultSeverity
{
  WARNING = 0,
  CRITICAL = 1,
  EMERGENCY = 2
};

/// A configured fault.
struct FaultDefinition
{
  FaultType type{FaultType::NONE};
  FaultSeverity severity{FaultSeverity::WARNING};
  std::string target;       // component / sensor name the fault acts on
  double magnitude{0.0};    // meaning depends on type (bias value, factor, rate...)
  double start_time_s{0.0}; // activation time [s]
  double duration_s{-1.0};  // active duration [s]; <0 means permanent
  std::string description;  // human-readable description
};

/// Human-readable name for a fault type (also used as FaultEvent.fault_type).
std::string to_string(FaultType type);

/// True if the fault corrupts a reported sensor value rather than the physics.
bool is_sensor_fault(FaultType type);

/// True if the fault alters actuator/thermal physics behaviour.
bool is_physics_fault(FaultType type);

}  // namespace faults
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__FAULTS__FAULT_TYPES_HPP_
