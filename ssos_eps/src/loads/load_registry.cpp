#include "ssos_eps/loads/load_registry.hpp"

#include <algorithm>

namespace ssos_eps
{
namespace loads
{

void LoadRegistry::upsert(const LoadSpec & spec)
{
  for (auto & load : loads_) {
    if (load.id == spec.id) {
      load = spec;
      return;
    }
  }
  loads_.push_back(spec);
}

bool LoadRegistry::remove(const std::string & id)
{
  const auto it = std::find_if(loads_.begin(), loads_.end(),
      [&id](const LoadSpec & l) {return l.id == id;});
  if (it == loads_.end()) {
    return false;
  }
  loads_.erase(it);
  return true;
}

const LoadSpec * LoadRegistry::find(const std::string & id) const
{
  for (const auto & load : loads_) {
    if (load.id == id) {
      return &load;
    }
  }
  return nullptr;
}

void LoadRegistry::set_demand_scale(double scale)
{
  demand_scale_ = std::max(scale, 0.0);
}

double LoadRegistry::requested_w(const LoadSpec & spec) const
{
  return std::max(spec.power_w, 0.0) * demand_scale_;
}

double LoadRegistry::requested_on_ddcu(int ddcu, const std::string & exclude_id) const
{
  double total = 0.0;
  for (const auto & load : loads_) {
    if (load.ddcu == ddcu && load.id != exclude_id) {
      total += requested_w(load);
    }
  }
  return total;
}

double LoadRegistry::total_requested_w() const
{
  double total = 0.0;
  for (const auto & load : loads_) {
    total += requested_w(load);
  }
  return total;
}

std::vector<std::size_t> LoadRegistry::shed_order(
  const std::vector<std::size_t> & candidates) const
{
  std::vector<std::size_t> order = candidates;
  std::stable_sort(order.begin(), order.end(), [this](std::size_t a, std::size_t b) {
      const auto & la = loads_[a];
      const auto & lb = loads_[b];
      if (la.priority != lb.priority) {
        return la.priority > lb.priority;
      }
      return la.power_w > lb.power_w;
    });
  return order;
}

}  // namespace loads
}  // namespace ssos_eps
