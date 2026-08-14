#include "ssos_eclss/ars/cycle_state_machine.hpp"

namespace ssos_eclss
{
namespace ars
{

CycleStateMachine::CycleStateMachine(const CycleTiming & timing,
                                     const HeaterParams & heater)
: timing_(timing), heater_(heater)
{
  reset();
}

void CycleStateMachine::reset()
{
  t_half_ = 0.0;
  adsorbing_train_ = 0;
  half_cycle_count_ = 0;
}

void CycleStateMachine::update(double dt)
{
  t_half_ += dt;
  const double half = timing_.half_cycle_s();
  while (t_half_ >= half) {
    t_half_ -= half;
    adsorbing_train_ = 1 - adsorbing_train_;
    ++half_cycle_count_;
  }
}

RegenPhase CycleStateMachine::regen_phase() const
{
  if (t_half_ < timing_.air_save_s) {
    return RegenPhase::AIR_SAVE;
  }
  if (t_half_ < timing_.air_save_s + timing_.adsorb_s) {
    return RegenPhase::DESORB;
  }
  return RegenPhase::VACUUM;
}

TrainCommand CycleStateMachine::command_for_train(int train) const
{
  TrainCommand cmd{};
  if (train == adsorbing_train_) {
    // Adsorbing train: process flow through both beds, no heat.
    cmd.desiccant_mode = BedMode::ADSORBING;
    cmd.adsorbent_mode = BedMode::ADSORBING;
    cmd.desiccant_heater_w = 0.0;
    cmd.adsorbent_heater_w = 0.0;
    return cmd;
  }

  // Regenerating train: phase-dependent.
  switch (regen_phase()) {
    case RegenPhase::AIR_SAVE:
      cmd.desiccant_mode = BedMode::AIR_SAVE;
      cmd.adsorbent_mode = BedMode::AIR_SAVE;
      // Central 7 of 19 elements energised during air-save.
      cmd.adsorbent_heater_w = heater_.central_power_w;
      cmd.desiccant_heater_w = 0.0;
      break;
    case RegenPhase::DESORB:
      cmd.desiccant_mode = BedMode::DESORBING;
      cmd.adsorbent_mode = BedMode::DESORBING;
      // All 19 elements during desorption.
      cmd.adsorbent_heater_w = heater_.total_power_w;
      // Desiccant beds are regenerated largely by the warm purge; modest heat.
      cmd.desiccant_heater_w = 0.0;
      break;
    case RegenPhase::VACUUM:
      cmd.desiccant_mode = BedMode::VACUUM;
      cmd.adsorbent_mode = BedMode::VACUUM;
      cmd.adsorbent_heater_w = 0.0;
      cmd.desiccant_heater_w = 0.0;
      break;
  }
  return cmd;
}

}  // namespace ars
}  // namespace ssos_eclss
