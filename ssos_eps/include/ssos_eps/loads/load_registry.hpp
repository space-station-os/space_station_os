#ifndef SSOS_EPS__LOADS__LOAD_REGISTRY_HPP_
#define SSOS_EPS__LOADS__LOAD_REGISTRY_HPP_

#include <string>
#include <vector>

// Named electrical loads on the secondary bus. Each load has a requested
// power, a priority (0 = most critical, higher numbers are shed first) and the
// DDCU it is assigned to. Admission against DDCU ratings and supply margin is
// decided by the EPS system; this class stores the loads and orders them for
// shedding. No ROS.

namespace ssos_eps
{
namespace loads
{

struct LoadSpec
{
  std::string id;
  double power_w;              // nominal requested power [W]
  int priority;                // 0 = most critical
  int ddcu;                    // assigned DDCU (-1 = not yet assigned)
};

struct LoadState
{
  std::string id;
  int priority;
  int ddcu;
  double requested_w;          // nominal power x demand scale [W]
  double served_w;             // power actually delivered [W]
  bool shed;                   // switched off to protect the bus
};

class LoadRegistry
{
public:
  /// Add a load or replace the one with the same id.
  void upsert(const LoadSpec & spec);

  /// Remove a load. Returns false if the id is unknown.
  bool remove(const std::string & id);

  void clear() {loads_.clear();}

  /// Pointer to the load with this id, or nullptr.
  const LoadSpec * find(const std::string & id) const;

  const std::vector<LoadSpec> & loads() const {return loads_;}

  /// Scale applied to every load's nominal power.
  void set_demand_scale(double scale);
  double demand_scale() const {return demand_scale_;}

  /// Requested power of one load after scaling [W].
  double requested_w(const LoadSpec & spec) const;

  /// Total requested power on one DDCU, optionally ignoring one load [W].
  double requested_on_ddcu(int ddcu, const std::string & exclude_id = "") const;

  /// Total requested power over all loads [W].
  double total_requested_w() const;

  /// Indices into loads() in shedding order: least critical first, then
  /// largest first, so the fewest and least important loads are dropped.
  std::vector<std::size_t> shed_order(const std::vector<std::size_t> & candidates) const;

private:
  std::vector<LoadSpec> loads_;
  double demand_scale_{1.0};
};

}  // namespace loads
}  // namespace ssos_eps

#endif  // SSOS_EPS__LOADS__LOAD_REGISTRY_HPP_
