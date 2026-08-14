#include "ssos_eclss/faults/fault_types.hpp"

namespace ssos_eclss
{
namespace faults
{

std::string to_string(FaultType type)
{
  switch (type) {
    case FaultType::NONE: return "none";
    case FaultType::SENSOR_STUCK: return "sensor_stuck";
    case FaultType::SENSOR_DRIFT: return "sensor_drift";
    case FaultType::SENSOR_BIAS: return "sensor_bias";
    case FaultType::SENSOR_NOISE: return "sensor_noise";
    case FaultType::SENSOR_SCALE: return "sensor_scale";
    case FaultType::VALVE_STUCK_OPEN: return "valve_stuck_open";
    case FaultType::VALVE_STUCK_CLOSED: return "valve_stuck_closed";
    case FaultType::BLOWER_DEGRADED: return "blower_degraded";
    case FaultType::BLOWER_FAILED: return "blower_failed";
    case FaultType::PUMP_FAILED: return "pump_failed";
    case FaultType::HEATER_PARTIAL: return "heater_partial";
    case FaultType::HEATER_FAILED: return "heater_failed";
    case FaultType::PRECOOLER_DEGRADED: return "precooler_degraded";
    case FaultType::INSULATION_LOSS: return "insulation_loss";
    case FaultType::CABIN_LEAK: return "cabin_leak";
  }
  return "unknown";
}

bool is_sensor_fault(FaultType type)
{
  switch (type) {
    case FaultType::SENSOR_STUCK:
    case FaultType::SENSOR_DRIFT:
    case FaultType::SENSOR_BIAS:
    case FaultType::SENSOR_NOISE:
    case FaultType::SENSOR_SCALE:
      return true;
    default:
      return false;
  }
}

bool is_physics_fault(FaultType type)
{
  return type != FaultType::NONE && !is_sensor_fault(type);
}

}  // namespace faults
}  // namespace ssos_eclss
