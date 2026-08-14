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

  // UPA vapor-compression distillation.
  r.upa_distillate_kg_s = feed * rec;
  const double upa_brine = feed - r.upa_distillate_kg_s;

  // Optional Brine Processor Assembly: dewaters the UPA brine, returning water
  // to the condensate stream and raising total recovery toward ~98%.
  if (params_.brine_processor_enabled) {
    const double brine_rec = std::clamp(params_.brine_recovery_fraction, 0.0, 1.0);
    r.bpa_water_kg_s = upa_brine * brine_rec;
  }

  r.distillate_kg_s = r.upa_distillate_kg_s + r.bpa_water_kg_s;
  r.brine_kg_s = feed - r.distillate_kg_s;
  r.energy_w = r.distillate_kg_s * params_.specific_energy_j_kg;
  r.recovery_fraction = (feed > 0.0) ? r.distillate_kg_s / feed : rec;
  return r;
}

}  // namespace wrs
}  // namespace ssos_eclss
