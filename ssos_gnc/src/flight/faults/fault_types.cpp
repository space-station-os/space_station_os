#include "ssos_gnc/flight/faults/fault_types.hpp"

namespace ssos_gnc
{

namespace flight
{

namespace faults
{

const char * fault_type_name(FaultType type)
{
  switch (type) {
    case FaultType::NONE: return "none";
    case FaultType::CMG_DEGRADED: return "cmg_degraded";
    case FaultType::CMG_FAILED: return "cmg_failed";
    case FaultType::THRUSTER_STUCK_OPEN: return "thruster_stuck_open";
    case FaultType::THRUSTER_STUCK_CLOSED: return "thruster_stuck_closed";
    case FaultType::IMU_BIAS: return "imu_bias";
    case FaultType::IMU_DRIFT: return "imu_drift";
    case FaultType::IMU_NOISE: return "imu_noise";
    case FaultType::IMU_STUCK: return "imu_stuck";
    case FaultType::STAR_TRACKER_DROPOUT: return "star_tracker_dropout";
    case FaultType::CONTROL_DIVERGENCE: return "control_divergence";
  }
  return "unknown";
}  // namespace faults

bool parse_fault_type(const std::string & text, FaultType & out)
{
  static const FaultType all[] = {
    FaultType::CMG_DEGRADED, FaultType::CMG_FAILED,
    FaultType::THRUSTER_STUCK_OPEN, FaultType::THRUSTER_STUCK_CLOSED,
    FaultType::IMU_BIAS, FaultType::IMU_DRIFT, FaultType::IMU_NOISE,
    FaultType::IMU_STUCK, FaultType::STAR_TRACKER_DROPOUT,
    FaultType::CONTROL_DIVERGENCE
  };

  for (FaultType t : all) {
    if (text == fault_type_name(t)) {
      out = t;
      return true;
    }
  }
  return false;
}  // namespace flight

FaultSeverity default_severity(FaultType type)
{
  switch (type) {
    case FaultType::CMG_FAILED:
    case FaultType::CONTROL_DIVERGENCE:
      return FaultSeverity::EMERGENCY;

    case FaultType::CMG_DEGRADED:
    case FaultType::THRUSTER_STUCK_OPEN:
    case FaultType::IMU_STUCK:
      return FaultSeverity::CRITICAL;

    default:
      return FaultSeverity::WARNING;
  }
}  // namespace ssos_gnc

bool disables_cmg(FaultType type)
{
  return type == FaultType::CMG_FAILED;
}
}
}
}
