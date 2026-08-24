#ifndef SSOS_GNC__FLIGHT__FAULTS__FAULT_TYPES_HPP_
#define SSOS_GNC__FLIGHT__FAULTS__FAULT_TYPES_HPP_

#include <string>

namespace ssos_gnc
{

namespace flight
{

namespace faults
{

enum class FaultType
{
  NONE,
  CMG_DEGRADED,
  CMG_FAILED,
  THRUSTER_STUCK_OPEN,
  THRUSTER_STUCK_CLOSED,
  IMU_BIAS,
  IMU_DRIFT,
  IMU_NOISE,
  IMU_STUCK,
  STAR_TRACKER_DROPOUT,
  CONTROL_DIVERGENCE
};

enum class FaultSeverity
{
  WARNING = 0,
  CRITICAL = 1,
  EMERGENCY = 2
};

struct FaultDefinition
{
  FaultType type{FaultType::NONE};
  FaultSeverity severity{FaultSeverity::WARNING};
  std::string target;
  double magnitude{0.0};
  double start_time_s{0.0};
  double duration_s{-1.0};
  std::string description;
};

const char * fault_type_name(FaultType type);

bool parse_fault_type(const std::string & text, FaultType & out);

FaultSeverity default_severity(FaultType type);

bool disables_cmg(FaultType type);
}  // namespace faults
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__FAULTS__FAULT_TYPES_HPP_
