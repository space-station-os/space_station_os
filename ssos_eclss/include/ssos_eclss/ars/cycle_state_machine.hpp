#ifndef SSOS_ECLSS__ARS__CYCLE_STATE_MACHINE_HPP_
#define SSOS_ECLSS__ARS__CYCLE_STATE_MACHINE_HPP_

#include "ssos_eclss/ars/ars_parameters.hpp"
#include "ssos_eclss/ars/bed_model.hpp"

// Sequences the 4BMS half-cycle. Two trains (each a desiccant + an adsorbent
// bed) alternate: while one train adsorbs, the other regenerates through
// air-save -> desorb -> vacuum, then the trains swap. Schedules heater power
// (central 7 elements during air-save, all 19 during desorption). No ROS.

namespace ssos_eclss
{
namespace ars
{

/// Phase of the regenerating train within a half-cycle.
enum class RegenPhase
{
  AIR_SAVE,
  DESORB,
  VACUUM
};

/// Commanded modes/heater for one train (a desiccant + adsorbent pair).
struct TrainCommand
{
  BedMode desiccant_mode;
  BedMode adsorbent_mode;
  double desiccant_heater_w;
  double adsorbent_heater_w;
};

/// Drives the half-cycle sequencing.
class CycleStateMachine
{
public:
  CycleStateMachine(const CycleTiming & timing, const HeaterParams & heater);

  /// Reset to the start of a half-cycle with train 0 adsorbing.
  void reset();

  /// Advance the cycle clock by dt; swaps trains at half-cycle boundaries.
  void update(double dt);

  /// Index (0 or 1) of the train currently adsorbing.
  int adsorbing_train() const { return adsorbing_train_; }

  /// Index (0 or 1) of the train currently regenerating.
  int regenerating_train() const { return 1 - adsorbing_train_; }

  /// Current phase of the regenerating train.
  RegenPhase regen_phase() const;

  /// Time elapsed in the current half-cycle [s].
  double time_in_half_cycle() const { return t_half_; }

  /// Number of completed half-cycles (swaps).
  int half_cycle_count() const { return half_cycle_count_; }

  /// Command for a given train index (0 or 1).
  TrainCommand command_for_train(int train) const;

  void set_timing(const CycleTiming & t) { timing_ = t; }
  void set_heater(const HeaterParams & h) { heater_ = h; }
  const CycleTiming & timing() const { return timing_; }

private:
  CycleTiming timing_;
  HeaterParams heater_;
  double t_half_{0.0};
  int adsorbing_train_{0};
  int half_cycle_count_{0};
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__CYCLE_STATE_MACHINE_HPP_
