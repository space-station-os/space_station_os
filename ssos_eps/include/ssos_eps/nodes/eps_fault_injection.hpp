#ifndef SSOS_EPS__NODES__EPS_FAULT_INJECTION_HPP_
#define SSOS_EPS__NODES__EPS_FAULT_INJECTION_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

// Faults injected through the SSOS simulation path. ssos_sim publishes them on
// /ssos/fault_event with subsystem_name "eps" and a description of the form
//   "Injected fault: <type> (params: {...}, duration: <s> s)"
//   "Scheduled fault: <name> (params: {...}, duration: <s> s)"
// This class parses those events and keeps the set of active faults with
// their expiry, so the node can translate them into physics fault hooks. It
// holds no ROS types; time is plain seconds of node (simulation) time.

namespace ssos_eps
{
namespace nodes
{

struct InjectedFault
{
  std::string key;             // type, or type:index for per-unit faults
  std::string type;
  std::map<std::string, double> params;
  double expires_s;            // < 0 = active until cleared
};

class EpsFaultInjection
{
public:
  /// Fault types EPS understands (see docs/fault_catalog.md).
  static const std::vector<std::string> & supported_types();

  /// Name of the index parameter for per-unit faults ("channel", "oru", ...),
  /// or empty for station-wide faults.
  static std::string index_param(const std::string & type);

  /// True if the description comes from the simulation fault path; fills the
  /// parameter JSON and duration.
  static bool parse_description(const std::string & description, std::string & params_json,
                                double & duration_s);

  /// Numbers and booleans (as 1/0) from a flat JSON object; strings ignored.
  static std::map<std::string, double> parse_params(const std::string & json);

  /// Apply one fault command at time now_s. Returns false (with a reason) for
  /// unknown types.
  bool apply(const std::string & type, const std::map<std::string, double> & params,
             double duration_s, double now_s, std::string & message);

  /// Drop faults whose duration has run out. Returns true if any expired.
  bool expire(double now_s);

  void clear();

  const std::vector<InjectedFault> & active() const {return active_;}
  /// Incremented whenever the active set changes.
  uint64_t revision() const {return revision_;}

  std::set<int> indices(const std::string & type) const;
  double array_factor() const;    // lowest injected array output factor (1 = none)
  double health_factor() const;   // lowest injected battery state of health (1 = none)
  double demand_scale() const;    // highest injected load multiplier (1 = none)

  /// One-shot state-of-charge override from battery_low_soc, if pending.
  std::optional<double> take_pending_soc();

private:
  double param_or(const InjectedFault & f, const std::string & name, double fallback) const;

  std::vector<InjectedFault> active_;
  std::optional<double> pending_soc_;
  uint64_t revision_{0};
};

}  // namespace nodes
}  // namespace ssos_eps

#endif  // SSOS_EPS__NODES__EPS_FAULT_INJECTION_HPP_
