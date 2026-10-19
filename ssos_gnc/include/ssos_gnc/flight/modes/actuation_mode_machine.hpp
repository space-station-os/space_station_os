#ifndef SSOS_GNC__FLIGHT__MODES__ACTUATION_MODE_MACHINE_HPP_
#define SSOS_GNC__FLIGHT__MODES__ACTUATION_MODE_MACHINE_HPP_

#include <string>

#include "ssos_gnc/flight/control/control_parameters.hpp"

namespace ssos_gnc
{

namespace flight
{

enum class ActuationMode
{
  CMG,
  THRUSTER
};

struct ModeInputs
{
  bool cmg_healthy{true};
  bool thruster_healthy{true};
};

struct ModeRequestResult
{
  bool accepted{false};
  std::string reason;
  ActuationMode current{ActuationMode::CMG};
};

class ActuationModeMachine
{
public:
  ActuationModeMachine();
  explicit ActuationModeMachine(const ModeParams & params);

  void reset();

  void update(double dt, const ModeInputs & inputs);

  ModeRequestResult request_mode(ActuationMode desired);

  ActuationMode mode() const {return mode_;}
  double time_in_mode() const {return time_in_mode_;}
  int transition_count() const {return transition_count_;}
  bool forced_by_fault() const {return forced_by_fault_;}

  void set_parameters(const ModeParams & p) {params_ = p;}

  static const char * mode_name(ActuationMode m);
  const char * mode_name() const {return mode_name(mode_);}

  static bool parse_mode(const std::string & text, ActuationMode & out);

private:
  void transition_to(ActuationMode m, bool by_fault);

  ModeParams params_;
  ActuationMode mode_{ActuationMode::CMG};
  double time_in_mode_{0.0};
  int transition_count_{0};
  bool forced_by_fault_{false};
};
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__MODES__ACTUATION_MODE_MACHINE_HPP_
