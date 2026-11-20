#ifndef SSOS_SIM__FAULT_FORMAT_HPP_
#define SSOS_SIM__FAULT_FORMAT_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>

// Faults from the simulation path (/sim/inject_fault and scenario files) are
// published on /ssos/fault_event with a description of the form
//   "Injected fault: <type> (params: {...}, duration: <s> s)"
//   "Scheduled fault: <name> (params: {...}, duration: <s> s)"
// Subsystems use these helpers to recognise and apply them. No ROS types.

namespace ssos_sim
{

/// Fault type that removes the active drill from a subsystem.
inline constexpr const char * kClearFaults = "clear_faults";

/// The drills each subsystem accepts, by subsystem name. The simulation
/// refuses anything else, and the subsystems apply only their own.
const std::map<std::string, std::set<std::string>> & drill_catalog();
bool is_known_drill(const std::string & target, const std::string & type);
/// A drill the target can apply: known type, unit indices within the default
/// station topology, and no "clear" parameter (clear_faults ends a drill).
bool validate_drill(const std::string & target, const std::string & type,
                    const std::map<std::string, double> & params, std::string & reason);

struct InjectedFault
{
  std::string type;
  std::map<std::string, double> params;
  double duration_s{0.0};   // <= 0 = active until cleared
};

/// "<prefix>: <label> (params: <json>, duration: <s> s)".
std::string fault_description(const std::string & prefix, const std::string & label,
                              const std::string & parameters_json, double duration_s);

/// True if the description comes from the simulation path; fills the
/// parameter JSON and the duration.
bool parse_fault_description(const std::string & description, std::string & params_json,
                             double & duration_s);

/// Numbers and booleans (as 1/0) from a flat JSON object; strings are ignored.
std::map<std::string, double> parse_fault_params(const std::string & json);

/// The injected fault carried by a FaultEvent's type and description, or
/// nothing if the event did not come from the simulation path.
std::optional<InjectedFault> parse_injected_fault(const std::string & fault_type,
                                                  const std::string & description);

/// The one drill a subsystem is running, with its expiry in simulation time.
class FaultTracker
{
public:
  /// Start a drill at now_s, replacing any active one. clear_faults clears.
  void apply(const InjectedFault & fault, double now_s);
  /// End the drill if its duration has run out. Returns true if it ended.
  bool expire(double now_s);
  void clear();

  bool any() const {return active_.has_value();}
  bool active(const std::string & type) const {return active_ && active_->type == type;}
  const std::string & type() const;
  double param(const std::string & name, double fallback) const;
  /// Incremented whenever the drill starts, changes or ends.
  uint64_t revision() const {return revision_;}

private:
  std::optional<InjectedFault> active_;
  double expires_s_{-1.0};
  uint64_t revision_{0};
};

}  // namespace ssos_sim

#endif  // SSOS_SIM__FAULT_FORMAT_HPP_
