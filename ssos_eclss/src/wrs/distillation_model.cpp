#include "ssos_eclss/wrs/distillation_model.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace wrs
{

DistillationModel::DistillationModel(const DistillationParams & params)
: params_(params)
{}

DistillationResult DistillationModel::process(double feed_kg_s) const
{
  DistillationResult r{};
  const double feed = std::clamp(feed_kg_s, 0.0, params_.max_throughput_kg_s);
  const double rec = std::clamp(params_.recovery_fraction, 0.0, 1.0);

  r.distillate_kg_s = feed * rec;
  r.brine_kg_s = feed - r.distillate_kg_s;
  r.energy_w = r.distillate_kg_s * params_.specific_energy_j_kg;
  r.recovery_fraction = rec;
  return r;
}

}  // namespace wrs
}  // namespace ssos_eclss
