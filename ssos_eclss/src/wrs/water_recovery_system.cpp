#include "ssos_eclss/wrs/water_recovery_system.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace wrs
{

namespace
{
constexpr double kPotableLimitUs = 100.0;  // potable conductivity limit [uS/cm]
}

WaterRecoverySystem::WaterRecoverySystem(const WrsParameters & params)
: params_(params)
{
  distillation_ = std::make_unique<DistillationModel>(params_.distillation);
  multifiltration_ = std::make_unique<MultifiltrationModel>(params_.multifiltration);
  catalytic_ = std::make_unique<CatalyticReactorModel>(params_.catalytic);
}

void WaterRecoverySystem::reset()
{
  multifiltration_->reset();
}

void WaterRecoverySystem::set_parameters(const WrsParameters & params)
{
  params_ = params;
  distillation_->set_params(params_.distillation);
  multifiltration_->set_params(params_.multifiltration);
  catalytic_->set_params(params_.catalytic);
}

WrsResult WaterRecoverySystem::step(double dt, double urine_kg_s,
                                    double condensate_kg_s, double voc_kg_s)
{
  WrsResult r{};

  // ---- UPA: distil urine ----
  const DistillationResult dist = distillation_->process(urine_kg_s);

  // ---- Combine distillate with humidity condensate to feed the WPA ----
  const double wpa_feed = dist.distillate_kg_s + condensate_kg_s;

  // ---- Catalytic oxidation of volatile organics ----
  const CatalyticResult cat = catalytic_->process(voc_kg_s, params_.catalytic.operating_temp_k);

  // ---- Multifiltration polish ----
  const MultifiltrationResult mf = multifiltration_->process(
    dt, wpa_feed, params_.multifiltration.inlet_conductivity_us);

  r.potable_water_kg_s = mf.product_kg_s;
  r.brine_kg_s = dist.brine_kg_s;
  r.product_conductivity_us = mf.product_conductivity_us;
  r.power_w = dist.energy_w;
  r.voc_conversion = cat.conversion;
  r.multifiltration_broken_through = mf.broken_through;
  r.potable_in_spec = (mf.product_conductivity_us <= kPotableLimitUs);

  const double water_in = urine_kg_s + condensate_kg_s;
  r.overall_recovery = (water_in > 0.0) ? r.potable_water_kg_s / water_in : 0.0;
  return r;
}

// ---- Parameter factories (ISS WRS-class defaults) ----

DistillationParams default_distillation_params()
{
  DistillationParams p{};
  p.recovery_fraction = 0.87;            // UPA ~85-87%
  p.specific_energy_j_kg = 1.1e5 * 3.6;  // ~110 Wh/kg -> J/kg
  p.max_throughput_kg_s = 9.0 / 86400.0; // ~9 kg/day
  p.brine_solids_fraction = 0.05;
  return p;
}

MultifiltrationParams default_multifiltration_params()
{
  MultifiltrationParams p{};
  p.bed_capacity_kg = 0.5;          // contaminant capacity [kg]
  p.removal_efficiency = 0.999;
  p.inlet_conductivity_us = 2000.0; // feed conductivity [uS/cm]
  return p;
}

CatalyticReactorParams default_catalytic_params()
{
  CatalyticReactorParams p{};
  p.operating_temp_k = 403.15;   // ~130 C
  p.activation_temp_k = 333.15;  // ~60 C
  p.max_conversion = 0.99;
  return p;
}

WrsParameters default_wrs_parameters()
{
  WrsParameters p{};
  p.distillation = default_distillation_params();
  p.multifiltration = default_multifiltration_params();
  p.catalytic = default_catalytic_params();
  return p;
}

}  // namespace wrs
}  // namespace ssos_eclss
