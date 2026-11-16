#include "ssos_gnc/flight/modes/actuation_mode_machine.hpp"

#include <algorithm>

namespace ssos_gnc
{

namespace flight
{

ActuationModeMachine::ActuationModeMachine()
: params_(ModeParams{})
{
}  // namespace flight

ActuationModeMachine::ActuationModeMachine(const ModeParams & params)
: params_(params)
{
}  // namespace ssos_gnc

void ActuationModeMachine::reset()
{
  mode_ = ActuationMode::CMG;
  time_in_mode_ = params_.min_dwell_s;
  transition_count_ = 0;
  forced_by_fault_ = false;
}

const char * ActuationModeMachine::mode_name(ActuationMode m)
{
  switch (m) {
    case ActuationMode::CMG: return "cmg";
    case ActuationMode::THRUSTER: return "thruster";
  }
  return "unknown";
}

bool ActuationModeMachine::parse_mode(const std::string & text, ActuationMode & out)
{
  std::string lower;
  lower.reserve(text.size());
  for (char c : text) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }

  if (lower == "cmg") {
    out = ActuationMode::CMG;
    return true;
  }
  if (lower == "thruster") {
    out = ActuationMode::THRUSTER;
    return true;
  }
  return false;
}

void ActuationModeMachine::transition_to(ActuationMode m, bool by_fault)
{
  if (m == mode_) {return;}
  mode_ = m;
  time_in_mode_ = 0.0;
  ++transition_count_;
  forced_by_fault_ = by_fault;
}

void ActuationModeMachine::update(double dt, const ModeInputs & inputs)
{
  if (dt > 0.0) {
    time_in_mode_ += dt;
  }

  if (mode_ == ActuationMode::CMG && !inputs.cmg_healthy) {
    transition_to(ActuationMode::THRUSTER, true);
    return;
  }

  if (mode_ == ActuationMode::THRUSTER && inputs.cmg_healthy) {
    forced_by_fault_ = false;
  }
}

ModeRequestResult ActuationModeMachine::request_mode(ActuationMode desired)
{
  ModeRequestResult r;
  r.current = mode_;

  if (desired == mode_) {
    r.accepted = true;
    r.reason = "already in requested mode";
    return r;
  }

  if (time_in_mode_ < params_.min_dwell_s) {
    r.reason = "rejected: minimum dwell time not elapsed";
    return r;
  }

  if (forced_by_fault_ && desired == ActuationMode::CMG) {
    r.reason = "rejected: CMG unavailable, mode forced by fault";
    return r;
  }

  transition_to(desired, false);
  r.accepted = true;
  r.current = mode_;
  r.reason = "accepted";
  return r;
}
}
}
