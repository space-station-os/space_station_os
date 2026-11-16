#ifndef SSOS_EPS__MBSU__MBSU_MODEL_HPP_
#define SSOS_EPS__MBSU__MBSU_MODEL_HPP_

#include <string>
#include <vector>

#include "ssos_eps/mbsu/mbsu_parameters.hpp"

// Main bus switching unit. Takes the state of each source channel wired to it,
// keeps those that are usable (connected, in the voltage window, able to
// supply), and spreads the DDCU demand across them in proportion to what each
// can deliver. Losing a channel therefore reroutes its share to the others.
// Demand beyond the combined capability is reported as a shortfall for the
// caller to shed. No ROS.

namespace ssos_eps
{
namespace mbsu
{

struct MbsuSource
{
  int channel;                 // power channel index
  bool connected;              // feeder closed and channel not isolated
  double voltage_v;            // channel bus voltage [V]
  double capability_w;         // power the channel can deliver this step [W]
};

struct MbsuState
{
  double bus_voltage_v;        // mean voltage of the usable sources [V]
  std::vector<int> source_channels;
  std::vector<bool> source_available;
  std::vector<double> allocation_w;   // power drawn from each source [W]
  int available_sources;
  double capability_w;         // combined capability of usable sources [W]
  double demand_w;             // requested by the DDCUs [W]
  double supplied_w;           // routed to the DDCUs [W]
  double shortfall_w;          // demand the sources cannot cover [W]
  bool unstable;               // no usable bus
  bool redundancy_lost;        // fewer than redundant_sources usable
  bool faulted;
  std::string status_message;
};

class MbsuModel
{
public:
  explicit MbsuModel(const MbsuParams & params = default_mbsu_params());

  void reset();

  /// True if a source can feed this MBSU.
  bool usable(const MbsuSource & source) const;

  /// Route demand_w across the sources for this step.
  const MbsuState & route(const std::vector<MbsuSource> & sources, double demand_w);

  void set_fault(bool faulted) {faulted_ = faulted;}
  bool fault() const {return faulted_;}

  void set_params(const MbsuParams & params) {params_ = params;}
  const MbsuParams & params() const {return params_;}
  const MbsuState & state() const {return state_;}

private:
  MbsuParams params_;
  MbsuState state_{};
  bool faulted_{false};
};

}  // namespace mbsu
}  // namespace ssos_eps

#endif  // SSOS_EPS__MBSU__MBSU_MODEL_HPP_
