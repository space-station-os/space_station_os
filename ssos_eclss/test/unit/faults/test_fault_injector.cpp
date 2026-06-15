#include <gtest/gtest.h>

#include "ssos_eclss/faults/fault_injector.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::faults;

namespace
{
FaultDefinition make(FaultType type, const std::string & target, double mag,
                     double start = 0.0, double dur = -1.0)
{
  FaultDefinition d{};
  d.type = type;
  d.target = target;
  d.magnitude = mag;
  d.start_time_s = start;
  d.duration_s = dur;
  return d;
}
}  // namespace

TEST(FaultInjector, ActivationWindow)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::HEATER_FAILED, "ars_heater", 0.0, 100.0, 50.0));
  fi.update(50.0);
  EXPECT_FALSE(fi.any_active());
  fi.update(120.0);
  EXPECT_TRUE(fi.any_active());
  fi.update(200.0);
  EXPECT_FALSE(fi.any_active());
}

TEST(FaultInjector, HeaterFailedZerosPower)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::HEATER_FAILED, "ars_heater", 0.0));
  fi.update(10.0);
  EXPECT_NEAR(fi.thermal_effect("ars_heater").heater_power_factor, 0.0, 1.0e-12);
  // A different component is unaffected.
  EXPECT_NEAR(fi.thermal_effect("other").heater_power_factor, 1.0, 1.0e-12);
}

TEST(FaultInjector, HeaterPartialReducesPower)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::HEATER_PARTIAL, "ars_heater", 7.0 / 19.0));
  fi.update(0.0);
  EXPECT_NEAR(fi.thermal_effect("ars_heater").heater_power_factor, 7.0 / 19.0, 1.0e-9);
}

TEST(FaultInjector, BlowerDegradedReducesEffectiveness)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::BLOWER_DEGRADED, "ars_blower", 0.6));
  fi.update(0.0);
  EXPECT_NEAR(fi.actuator_effect("ars_blower").effectiveness, 0.6, 1.0e-9);
}

TEST(FaultInjector, ValveStuckClosedOverrides)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::VALVE_STUCK_CLOSED, "sel_valve", 0.0));
  fi.update(0.0);
  const ActuatorEffect e = fi.actuator_effect("sel_valve");
  EXPECT_TRUE(e.override_position);
  EXPECT_NEAR(e.forced_position, 0.0, 1.0e-12);
  EXPECT_NEAR(e.effectiveness, 0.0, 1.0e-12);
}

TEST(FaultInjector, SensorBiasApplied)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::SENSOR_BIAS, "co2_sensor", 50.0));
  fi.update(0.0);
  EXPECT_NEAR(fi.apply_sensor("co2_sensor", 400.0), 450.0, 1.0e-9);
  // Inactive when out of window doesn't change reading.
  EXPECT_NEAR(fi.apply_sensor("other_sensor", 400.0), 400.0, 1.0e-9);
}

TEST(FaultInjector, SensorStuckLatches)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::SENSOR_STUCK, "co2_sensor", 0.0));
  fi.update(0.0);
  const double first = fi.apply_sensor("co2_sensor", 400.0);
  const double second = fi.apply_sensor("co2_sensor", 800.0);
  EXPECT_NEAR(first, 400.0, 1.0e-9);
  EXPECT_NEAR(second, 400.0, 1.0e-9);  // latched
}

TEST(FaultInjector, CabinLeakAreaAccumulates)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::CABIN_LEAK, "cabin", 1.0e-4));
  fi.update(0.0);
  EXPECT_NEAR(fi.leak_fault_area("cabin"), 1.0e-4, 1.0e-12);
}

TEST(FaultInjector, ClearRemovesAll)
{
  FaultInjector fi;
  fi.register_fault(make(FaultType::HEATER_FAILED, "ars_heater", 0.0));
  fi.clear();
  fi.update(10.0);
  EXPECT_FALSE(fi.any_active());
  EXPECT_EQ(fi.count(), 0u);
}
