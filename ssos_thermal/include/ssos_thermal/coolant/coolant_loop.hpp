#ifndef SSOS_THERMAL__COOLANT__COOLANT_LOOP_HPP_
#define SSOS_THERMAL__COOLANT__COOLANT_LOOP_HPP_

// Internal-loop-to-ammonia cooldown model. No ROS dependency. Ported from
// the physics embedded in space_station_thermal_control's
// CoolantActionServer::execute() (the only part of that class actually
// exercised by anything today -- see REFACTOR_PLAN.md's "cooling_server
// port" section for what was intentionally left behind: an inert Behavior
// Tree loop whose condition variables were never updated, water-recycling
// service calls, and two publishers that were never actually published to).

namespace ssos_thermal
{
namespace coolant
{

struct CoolantParams
{
  double mass_kg = 200.0;
  double specific_heat_j_per_kg_c = 4186.0;
  double heat_transfer_efficiency = 0.85;
  double vent_threshold_kj = 250.0;
};

struct CoolStepResult
{
  double node_temp_c;
  double ammonia_temp_c;
  double ammonia_heat_kj;
  bool vent_triggered;
  // True once node_temp_c has settled to within 0.5 degC of the target --
  // matches the legacy loop's termination condition exactly.
  bool done;
};

class CoolantLoop
{
public:
  explicit CoolantLoop(const CoolantParams & params);

  // One physics step of the cooldown process: node_temp_c moves at most
  // 2.5 degC toward target_temp_c, the heat removed is transferred to the
  // ammonia loop at heat_transfer_efficiency, and vent_triggered is set once
  // that transferred heat reaches vent_threshold_kj (matches the legacy
  // per-iteration body of CoolantActionServer::execute()).
  CoolStepResult step(double node_temp_c, double target_temp_c) const;

  const CoolantParams & params() const {return params_;}

private:
  CoolantParams params_;
};

}  // namespace coolant
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__COOLANT__COOLANT_LOOP_HPP_
