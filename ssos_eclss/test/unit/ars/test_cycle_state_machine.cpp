#include <gtest/gtest.h>

#include "ssos_eclss/ars/cycle_state_machine.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

TEST(CycleStateMachine, StartsWithTrain0Adsorbing)
{
  CycleStateMachine sm(default_cycle_timing(), default_heater());
  EXPECT_EQ(sm.adsorbing_train(), 0);
  EXPECT_EQ(sm.regenerating_train(), 1);
}

TEST(CycleStateMachine, SwapsAtHalfCycle)
{
  CycleTiming t = default_cycle_timing();
  CycleStateMachine sm(t, default_heater());
  // Advance just past one half cycle.
  sm.update(t.half_cycle_s() + 1.0);
  EXPECT_EQ(sm.adsorbing_train(), 1);
  EXPECT_EQ(sm.half_cycle_count(), 1);
}

TEST(CycleStateMachine, RegenPhaseSequence)
{
  CycleTiming t = default_cycle_timing();
  CycleStateMachine sm(t, default_heater());

  EXPECT_EQ(sm.regen_phase(), RegenPhase::AIR_SAVE);
  sm.update(t.air_save_s + 1.0);
  EXPECT_EQ(sm.regen_phase(), RegenPhase::DESORB);
  sm.update(t.adsorb_s);
  EXPECT_EQ(sm.regen_phase(), RegenPhase::VACUUM);
}

TEST(CycleStateMachine, AdsorbingTrainHasNoHeat)
{
  CycleStateMachine sm(default_cycle_timing(), default_heater());
  TrainCommand cmd = sm.command_for_train(sm.adsorbing_train());
  EXPECT_EQ(cmd.adsorbent_mode, BedMode::ADSORBING);
  EXPECT_EQ(cmd.adsorbent_heater_w, 0.0);
}

TEST(CycleStateMachine, DesorbUsesFullHeaterPower)
{
  CycleTiming t = default_cycle_timing();
  HeaterParams h = default_heater();
  CycleStateMachine sm(t, h);
  // Advance into the desorb phase.
  sm.update(t.air_save_s + 10.0);
  TrainCommand cmd = sm.command_for_train(sm.regenerating_train());
  EXPECT_EQ(cmd.adsorbent_mode, BedMode::DESORBING);
  EXPECT_NEAR(cmd.adsorbent_heater_w, h.total_power_w, 1.0e-6);
}

TEST(CycleStateMachine, AirSaveUsesCentralHeaters)
{
  HeaterParams h = default_heater();
  CycleStateMachine sm(default_cycle_timing(), h);
  TrainCommand cmd = sm.command_for_train(sm.regenerating_train());
  EXPECT_EQ(cmd.adsorbent_mode, BedMode::AIR_SAVE);
  EXPECT_NEAR(cmd.adsorbent_heater_w, h.central_power_w, 1.0e-6);
  EXPECT_LT(cmd.adsorbent_heater_w, h.total_power_w);
}
