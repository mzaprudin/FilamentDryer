#include <unity.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "control/AhSlopeEstimator.h"
#include "control/HeaterThermostat.h"
#include "control/HumidityMath.h"
#include "control/PidController.h"
#include "control/PurgeScheduler.h"
#include "config/AppConfig.h"
#include "config/Defaults.h"
#include "core/DryingStateMachine.h"
#include "domain/Interfaces.h"
#include "services/ActuatorService.h"
#include "services/ControlService.h"
#include "services/SafetyService.h"

namespace {
unsigned g_callTick = 0;
}  // namespace

class MockHeaterOutput final : public IHeaterOutput {
 public:
  void begin() override {}
  void setPower(float percent, uint32_t) override {
    power = percent;
    tick = ++g_callTick;
  }
  void off() override { power = 0.0f; }
  float power = 0.0f;
  unsigned tick = 0;
};

class MockFanOutput final : public IFanOutput {
 public:
  void begin() override {}
  void setPower(uint8_t percent) override {
    power = percent;
    tick = ++g_callTick;
  }
  void off() override { power = 0; }
  uint8_t power = 0;
  unsigned tick = 0;
};

class MockVentOutput final : public IVentOutput {
 public:
  void begin() override {}
  void setAngle(uint16_t angle) override { angle_ = angle; }
  uint16_t angle_ = 0;
};

void setUp() {}
void tearDown() {}

void test_absolute_humidity_round_trip() {
  const float absolute = relativeToAbsoluteHumidity(25.0f, 50.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 11.5f, absolute);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 50.0f, absoluteToRelativeHumidity(25.0f, absolute));
}

void test_pid_output_is_bounded_and_rate_limited() {
  const PidConfig config{2.0f, 0.1f, 0.0f, 0.0f, 100.0f};
  PidController pid;
  pid.begin(config, 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, pid.compute(0.0f, 100.0f, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, pid.compute(0.0f, 100.0f, 500));
  TEST_ASSERT(pid.compute(90.0f, 100.0f, 1000) < 100.0f);
}

void test_state_machine_timed_run() {
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 40.0f;
  DryingStateMachine machine;
  machine.begin(state, 0);
  Setpoints setpoints;
  setpoints.airTemperatureC = 45.0f;
  setpoints.relativeHumidity = 20.0f;
  setpoints.heaterLimitC = 105.0f;
  setpoints.durationSeconds = 10;
  TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedManual, setpoints, 0));
  machine.update(state, 0);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Warmup), static_cast<int>(state.phase));
  state.air.temperatureC = 44.0f;
  machine.update(state, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  machine.update(state, 11000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Finish), static_cast<int>(state.phase));
}

void test_state_machine_rejects_non_run_modes() {
  DeviceState state;
  DryingStateMachine machine;
  machine.begin(state, 0);
  TEST_ASSERT_FALSE(machine.start(state, DryingMode::Idle, Setpoints{}, 0));
}

void test_weight_sensors_are_optional_for_safety() {
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 25.0f;
  state.air.relativeHumidity = 40.0f;
  state.heater.valid = true;
  state.heater.temperatureC = 40.0f;
  state.setpoints.heaterLimitC = 105.0f;
  // Both weight sensors are disconnected, but this must not be an error.
  SafetyService safety(75.0f);
  FaultCode fault = FaultCode::WatchdogReset;
  TEST_ASSERT_TRUE(safety.evaluate(state, 0, fault));
  TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::None), static_cast<int>(fault));
}

void test_actuator_interlock_forces_fan_when_heating() {
  MockHeaterOutput heater;
  MockFanOutput fan;
  MockVentOutput vent;
  ActuatorService actuators(heater, fan, vent);
  actuators.begin(90, 25);
  ActuatorState outputs;
  outputs.heaterPower = 60.0f;
  outputs.fanPower = 0;
  actuators.apply(outputs, 0);
  TEST_ASSERT_EQUAL_UINT8(25, fan.power);
  TEST_ASSERT_EQUAL_UINT8(25, outputs.fanPower);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, heater.power);
  TEST_ASSERT_TRUE(heater.tick > fan.tick);
}

void test_actuator_interlock_idle_without_heat() {
  MockHeaterOutput heater;
  MockFanOutput fan;
  MockVentOutput vent;
  ActuatorService actuators(heater, fan, vent);
  actuators.begin(90, 25);
  ActuatorState outputs;
  outputs.heaterPower = 0.0f;
  outputs.fanPower = 10;
  actuators.apply(outputs, 0);
  TEST_ASSERT_EQUAL_UINT8(10, fan.power);
  outputs.fanPower = 80;
  actuators.apply(outputs, 1000);
  TEST_ASSERT_EQUAL_UINT8(80, fan.power);
}

void test_thermostat_holds_band() {
  HeaterThermostat thermostat;
  TEST_ASSERT_TRUE(thermostat.update(25.0f, 70.0f, 0));      // below band -> on
  TEST_ASSERT_TRUE(thermostat.update(69.0f, 70.0f, 3000));   // inside band -> hold on
  TEST_ASSERT_FALSE(thermostat.update(71.5f, 70.0f, 6000));  // above band -> off
  TEST_ASSERT_FALSE(thermostat.update(69.5f, 70.0f, 9000));  // inside band -> hold off
  TEST_ASSERT_TRUE(thermostat.update(50.0f, 70.0f, 12000));  // below band -> on
}

void test_thermostat_respects_min_switch_interval() {
  HeaterThermostat thermostat;
  TEST_ASSERT_TRUE(thermostat.update(25.0f, 70.0f, 0));     // on at t=0
  TEST_ASSERT_TRUE(thermostat.update(71.5f, 70.0f, 1000));  // <2s since switch -> hold on
  TEST_ASSERT_FALSE(thermostat.update(71.5f, 70.0f, 2500)); // now switching off is allowed
}

void test_heater_setpoint_ceiling() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, heaterSetpointCeilingC(105.0f, 105.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, heaterSetpointCeilingC(95.0f, 105.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, heaterSetpointCeilingC(105.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, heaterSetpointCeilingC(0.0f, 105.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, heaterSetpointCeilingC(0.0f, 0.0f));
}

void test_cascade_bounds_heater_setpoint_and_power() {
  AppConfig config;
  setDefaultConfig(config);
  SafetyService safety(75.0f);
  DryingStateMachine machine;
  ControlService control(config, safety, machine);
  control.begin();
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 25.0f;
  state.air.relativeHumidity = 40.0f;
  state.heater.valid = true;
  state.heater.temperatureC = 25.0f;
  state.setpoints.airTemperatureC = 65.0f;
  state.setpoints.relativeHumidity = 20.0f;
  state.setpoints.heaterLimitC = 105.0f;
  state.setpoints.durationSeconds = 3600;
  machine.begin(state, 0);
  TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedManual, state.setpoints, 0));
  control.update(state, 0);
  // Large error: outer PID clamps to the ceiling (105 - 5) and the cold
  // heater switches fully on; the interlock keeps the fan at its floor.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, state.heaterSetpointC);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, state.actuators.heaterPower);
  TEST_ASSERT_TRUE(state.actuators.fanPower >= 25);
  // Heater above the ceiling band: fully off.
  state.heater.temperatureC = 101.6f;
  control.update(state, 5000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, state.actuators.heaterPower);
}

void test_purge_scheduler_cycle() {
  PurgeScheduler purge;
  PurgeParams params;
  params.minSealMs = 600000;
  params.settleMs = 180000;
  params.maxPurgeMs = 45000;
  params.riseTriggerGm3 = 0.5f;
  purge.begin(params, 1000);
  // First purge is allowed immediately: room air above the floor vents now.
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Sealed), static_cast<int>(purge.phase()));
  purge.update(5.0f, 4.0f, 2000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Purge), static_cast<int>(purge.phase()));
  // Purge ends early once AH drops to the floor.
  purge.update(3.9f, 4.0f, 10000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Settle), static_cast<int>(purge.phase()));
  // Still settling: nothing changes before settleMs elapses.
  purge.update(3.9f, 4.0f, 100000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Settle), static_cast<int>(purge.phase()));
  // Settle completes: sealed again with the current AH as baseline.
  purge.update(3.5f, 4.0f, 200000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Sealed), static_cast<int>(purge.phase()));
  // Small rise below the trigger: stays sealed even after minSeal.
  purge.update(3.7f, 4.0f, 900000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Sealed), static_cast<int>(purge.phase()));
  // Rise above the trigger after the minimum seal time: purge.
  purge.update(4.2f, 4.0f, 1000000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Purge), static_cast<int>(purge.phase()));
  // Purge runs to its cap when AH never drops to the floor.
  purge.update(4.1f, 4.0f, 1000000 + 45000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Settle), static_cast<int>(purge.phase()));
}

void test_purge_scheduler_respects_min_seal() {
  PurgeScheduler purge;
  PurgeParams params;
  params.minSealMs = 600000;
  params.settleMs = 180000;
  params.maxPurgeMs = 45000;
  params.riseTriggerGm3 = 0.5f;
  purge.begin(params, 0);
  // Saturated room air: the first purge may start immediately.
  purge.update(5.0f, 4.0f, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Purge), static_cast<int>(purge.phase()));
  // Air exchanged below the floor: settle, then seal with baseline 1.0.
  purge.update(1.0f, 4.0f, 10000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Settle), static_cast<int>(purge.phase()));
  purge.update(1.0f, 4.0f, 190000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Sealed), static_cast<int>(purge.phase()));
  // Rise of 0.4 (< trigger): stays sealed.
  purge.update(1.4f, 4.0f, 300000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Sealed), static_cast<int>(purge.phase()));
  // Rise of 0.8 (>= trigger) but only 310 s sealed (< 600 s): still sealed.
  purge.update(1.8f, 4.0f, 500000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Sealed), static_cast<int>(purge.phase()));
  // Once the minimum seal time has elapsed the pending trigger fires.
  purge.update(1.8f, 4.0f, 800000);
  TEST_ASSERT_EQUAL(static_cast<int>(PurgePhase::Purge), static_cast<int>(purge.phase()));
}

void test_ah_slope_estimator_linear_rise() {
  AhSlopeEstimator estimator;
  estimator.reset();
  // 0.1 g/m3 every 30 s -> 12 g/m3/h.
  for (uint8_t i = 0; i < 12; ++i) {
    estimator.addSample(i * 30000UL, 1.0f + 0.1f * i);
  }
  TEST_ASSERT_TRUE(estimator.slopeValid());
  TEST_ASSERT_EQUAL_UINT8(12, estimator.sampleCount());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 12.0f, estimator.slopeGm3PerHour());
}

void test_ah_slope_estimator_flat_and_reset() {
  AhSlopeEstimator estimator;
  for (uint8_t i = 0; i < 10; ++i) {
    estimator.addSample(i * 30000UL, 2.0f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, estimator.slopeGm3PerHour());
  estimator.reset();
  TEST_ASSERT_EQUAL_UINT8(0, estimator.sampleCount());
  TEST_ASSERT_FALSE(estimator.slopeValid());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, estimator.slopeGm3PerHour());
}

void test_control_pulse_ventilation_and_dryness() {
  AppConfig config;
  setDefaultConfig(config);
  SafetyService safety(75.0f);
  DryingStateMachine machine;
  ControlService control(config, safety, machine);
  control.begin();
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 45.0f;
  state.air.relativeHumidity = 30.0f;
  // Room-level AH below the rise trigger baseline so the run starts sealed.
  state.air.absoluteHumidityGm3 = 0.3f;
  state.heater.valid = true;
  state.heater.temperatureC = 45.0f;
  state.setpoints.airTemperatureC = 45.0f;
  state.setpoints.relativeHumidity = 20.0f;
  state.setpoints.heaterLimitC = 95.0f;
  state.setpoints.durationSeconds = 3600;
  machine.begin(state, 0);
  TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedManual, state.setpoints, 0));
  // Warmup: air already at target, so the machine reaches Drying quickly.
  control.update(state, 0);
  control.update(state, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  // Sealed with low AH: vent closed, fan at circulation duty.
  TEST_ASSERT_EQUAL_UINT8(0, state.purgePhase);
  TEST_ASSERT_EQUAL_UINT16(15, state.actuators.ventAngle);
  TEST_ASSERT_EQUAL_UINT8(25, state.actuators.fanPower);
  // AH jumps far above the floor: first purge starts at once.
  state.air.absoluteHumidityGm3 = 6.0f;
  control.update(state, 2000);
  TEST_ASSERT_EQUAL_UINT8(2, state.purgePhase);
  TEST_ASSERT_EQUAL_UINT16(90, state.actuators.ventAngle);
  TEST_ASSERT_EQUAL_UINT8(100, state.actuators.fanPower);
  TEST_ASSERT_EQUAL_UINT8(0, state.ahSlopeSamples);
}

void test_state_machine_preset_dry_then_hold() {
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 60.0f;
  DryingStateMachine machine;
  machine.begin(state, 0);
  Setpoints setpoints;
  setpoints.airTemperatureC = 65.0f;
  setpoints.holdTemperatureC = 50.0f;
  setpoints.relativeHumidity = 15.0f;
  setpoints.drynessSlopeGm3PerHour = 0.3f;
  setpoints.heaterLimitC = 105.0f;
  setpoints.minDurationSeconds = 10;
  setpoints.maxDurationSeconds = 60;
  TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedPreset, setpoints, 0));
  machine.update(state, 0);
  state.air.temperatureC = 64.0f;
  machine.update(state, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  // Quiet windows cannot shorten the preset's minimum active-drying time.
  state.drynessChecks = 2;
  machine.update(state, 5000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  state.drynessChecks = 0;
  // Min time served but the spool still releases moisture: keep drying.
  machine.update(state, 12000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  // Two consecutive quiet windows: dry -> Hold.
  state.drynessChecks = 2;
  machine.update(state, 13000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold), static_cast<int>(state.phase));
  TEST_ASSERT_EQUAL_UINT32(0, state.remainingSeconds);
  // Clearing the streak is not a new moisture event.
  state.drynessChecks = 0;
  machine.update(state, 19000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold), static_cast<int>(state.phase));
  // The control loop emits a confirmed re-wetting event after a quiet Hold
  // window followed by a wet one: restart with a fresh drying clock.
  state.rewetDetected = true;
  machine.update(state, 20000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  TEST_ASSERT_EQUAL_UINT32(60, state.remainingSeconds);
  TEST_ASSERT_FALSE(state.rewetDetected);
  // Safety ceiling: settle into Hold even if never quiet.
  state.drynessChecks = 0;
  machine.update(state, 20000 + 61000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold), static_cast<int>(state.phase));
  machine.update(state, 20000 + 62000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold), static_cast<int>(state.phase));
}

void test_state_machine_manual_stops_at_duration() {
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 44.0f;
  DryingStateMachine machine;
  machine.begin(state, 0);
  Setpoints setpoints;
  setpoints.airTemperatureC = 45.0f;
  setpoints.relativeHumidity = 20.0f;
  setpoints.heaterLimitC = 105.0f;
  setpoints.durationSeconds = 10;
  TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedManual, setpoints, 0));
  machine.update(state, 0);
  machine.update(state, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  // Manual runs ignore dryness: the countdown decides.
  state.drynessChecks = 5;
  machine.update(state, 11000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Finish), static_cast<int>(state.phase));
}

void test_control_dryness_streak_from_sealed_windows() {
  AppConfig config;
  setDefaultConfig(config);
  SafetyService safety(75.0f);
  DryingStateMachine machine;
  ControlService control(config, safety, machine);
  control.begin();
  DeviceState state;
  state.air.valid = true;
  state.air.temperatureC = 45.0f;
  state.air.relativeHumidity = 30.0f;
  state.air.absoluteHumidityGm3 = 0.3f;
  state.heater.valid = true;
  state.heater.temperatureC = 45.0f;
  state.setpoints.airTemperatureC = 45.0f;
  state.setpoints.relativeHumidity = 20.0f;
  state.setpoints.heaterLimitC = 95.0f;
  state.setpoints.durationSeconds = 3600;
  state.setpoints.drynessSlopeGm3PerHour = 0.3f;
  machine.begin(state, 0);
  TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedPreset, state.setpoints, 0));
  control.update(state, 0);
  control.update(state, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying), static_cast<int>(state.phase));
  // Quiet sealed window: flat AH for ~5 min of samples, then purge.
  for (uint32_t t = 2000; t <= 302000; t += 30000) {
    state.air.absoluteHumidityGm3 = 0.3f;
    control.update(state, t);
  }
  state.air.absoluteHumidityGm3 = 6.0f;  // saturated -> purge begins
  control.update(state, 310000);
  TEST_ASSERT_EQUAL_UINT8(2, state.purgePhase);
  TEST_ASSERT_EQUAL_UINT8(1, state.drynessChecks);
  // Wet sealed window: AH climbing at ~18 g/m3/h after the settle completes
  // (t=491000), well above the 0.3 threshold. The rise triggers the next
  // purge exactly at t=1091000 (600 s sealed + rise above trigger); the
  // window judgement must reset the streak. The purge itself then ends
  // immediately (AH below the floor), so assert right at that tick.
  for (uint32_t t = 311000; t <= 1091000; t += 5000) {
    const float rise = t > 491000 ? 0.005f * static_cast<float>(t - 491000) / 1000.0f : 0.0f;
    state.air.absoluteHumidityGm3 = 0.5f + rise;
    control.update(state, t);
  }
  TEST_ASSERT_EQUAL_UINT8(2, state.purgePhase);
  TEST_ASSERT_EQUAL_UINT8(0, state.drynessChecks);
}

// Integration cases use measured AH directly so the temperature gate and the
// measurement-window lifecycle can be exercised independently of sensor math.
struct ControlFixture {
  AppConfig config;
  SafetyService safety{75.0f};
  DryingStateMachine machine;
  ControlService control{config, safety, machine};
  DeviceState state;

  ControlFixture() {
    setDefaultConfig(config);
    control.begin();
    state.air.valid = true;
    state.air.temperatureC = 45.0f;
    state.air.relativeHumidity = 30.0f;
    state.air.absoluteHumidityGm3 = 0.1f;
    state.heater.valid = true;
    state.heater.temperatureC = 45.0f;
    state.setpoints.airTemperatureC = 45.0f;
    state.setpoints.holdTemperatureC = 45.0f;
    state.setpoints.relativeHumidity = 20.0f;
    state.setpoints.heaterLimitC = 95.0f;
    state.setpoints.drynessSlopeGm3PerHour = 0.3f;
    state.setpoints.maxDurationSeconds = 7200;
    machine.begin(state, 0);
  }

  void start() {
    TEST_ASSERT_TRUE(machine.start(state, DryingMode::TimedPreset,
                                   state.setpoints, 0));
    control.update(state, 0);
    control.update(state, 1000);
  }

  void flat(uint32_t first, uint32_t last) {
    for (uint32_t now = first; now <= last; now += 30000UL) {
      control.update(state, now);
    }
  }
};

void test_control_flat_ah_completes_without_extra_purges() {
  ControlFixture fixture;
  fixture.start();
  fixture.flat(31000, 601000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.purgePhase);
  TEST_ASSERT_EQUAL_UINT16(15, fixture.state.actuators.ventAngle);
  fixture.flat(631000, 1201000);
  fixture.control.update(fixture.state, 1201001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.purgePhase);
  TEST_ASSERT_EQUAL_UINT16(15, fixture.state.actuators.ventAngle);
}

void test_control_short_plan_still_requires_eight_samples() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 30;
  fixture.start();
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.ahSlopeSamples);
  fixture.flat(31000, 181000);
  TEST_ASSERT_EQUAL_UINT8(7, fixture.state.ahSlopeSamples);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.control.update(fixture.state, 211000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.purgePhase);
}

void test_control_warmup_does_not_count_dry_windows() {
  ControlFixture fixture;
  fixture.state.air.temperatureC = 35.0f;
  fixture.start();
  fixture.flat(31000, 601000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Warmup),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.state.air.temperatureC = 45.0f;
  fixture.control.update(fixture.state, 602000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.ahSlopeSamples);
}

void test_control_temperature_gap_discards_partial_window() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 30;
  fixture.start();
  fixture.flat(31000, 181000);
  fixture.state.air.temperatureC = 47.0f;
  fixture.control.update(fixture.state, 190000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.state.air.temperatureC = 45.0f;
  fixture.control.update(fixture.state, 191000);
  fixture.flat(221000, 371000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.control.update(fixture.state, 401000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
}

void test_control_resume_starts_fresh_measurement_window() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 30;
  fixture.start();
  fixture.flat(31000, 181000);
  fixture.machine.pause(fixture.state, 190000);
  fixture.control.update(fixture.state, 190000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  fixture.machine.resume(fixture.state, 191000);
  fixture.control.update(fixture.state, 191000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.ahSlopeSamples);
  fixture.flat(221000, 371000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.control.update(fixture.state, 401000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
}

void test_control_new_run_during_drying_resets_windows() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 30;
  fixture.start();
  fixture.flat(31000, 211000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  TEST_ASSERT_TRUE(fixture.machine.start(fixture.state, DryingMode::TimedPreset,
                                        fixture.state.setpoints, 212000));
  fixture.control.update(fixture.state, 212000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  fixture.control.update(fixture.state, 213000);
  fixture.flat(243000, 393000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.control.update(fixture.state, 423000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
}

void test_control_initial_purge_and_settle_do_not_count_samples() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 30;
  fixture.state.air.absoluteHumidityGm3 = 6.0f;
  TEST_ASSERT_TRUE(fixture.machine.start(fixture.state, DryingMode::TimedPreset,
                                        fixture.state.setpoints, 0));
  fixture.control.update(fixture.state, 0);
  TEST_ASSERT_EQUAL_UINT8(2, fixture.state.purgePhase);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  fixture.control.update(fixture.state, 1000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.purgePhase);
  fixture.flat(31000, 151000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  fixture.control.update(fixture.state, 181000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.purgePhase);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.ahSlopeSamples);
  fixture.flat(211000, 391000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
}

void test_control_time_ceiling_keeps_hold_with_zero_streak() {
  ControlFixture fixture;
  fixture.state.setpoints.maxDurationSeconds = 10;
  fixture.start();
  fixture.control.update(fixture.state, 11000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  for (uint32_t now = 12000; now <= 72000; now += 1000) {
    fixture.control.update(fixture.state, now);
    TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                      static_cast<int>(fixture.state.phase));
    TEST_ASSERT_EQUAL_UINT32(0, fixture.state.remainingSeconds);
  }
}

void test_control_hold_requires_quiet_then_new_wet_window() {
  ControlFixture fixture;
  fixture.state.setpoints.maxDurationSeconds = 10;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.start();
  fixture.control.update(fixture.state, 11000);
  // The original wet spool remains in Hold at the time ceiling. It must
  // first become quiet, rather than immediately starting another full run.
  for (uint32_t now = 41000; now <= 431000; now += 30000) {
    fixture.state.air.absoluteHumidityGm3 =
        0.1f + 0.0001f * static_cast<float>(now - 11000) / 1000.0f;
    fixture.control.update(fixture.state, now);
    TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                      static_cast<int>(fixture.state.phase));
  }
  // One quiet complete window arms re-wetting detection.
  fixture.flat(461000, 641000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  const float baseline = fixture.state.air.absoluteHumidityGm3;
  for (uint32_t now = 671000; now <= 851000; now += 30000) {
    fixture.state.air.absoluteHumidityGm3 =
        baseline + 0.0001f * static_cast<float>(now - 641000) / 1000.0f;
    fixture.control.update(fixture.state, now);
  }
  fixture.control.update(fixture.state, 851001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT32(851001, fixture.state.runStartedAt);
  TEST_ASSERT_EQUAL_UINT32(10, fixture.state.remainingSeconds);
  fixture.control.update(fixture.state, 852001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
}

void test_control_zero_threshold_disables_dryness_and_redry() {
  ControlFixture fixture;
  fixture.state.setpoints.drynessSlopeGm3PerHour = 0.0f;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.state.setpoints.maxDurationSeconds = 900;
  fixture.start();
  fixture.flat(31000, 841000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.control.update(fixture.state, 901000);
  fixture.flat(931000, 1321000);
  for (uint32_t now = 1351000; now <= 1771000; now += 30000) {
    fixture.state.air.absoluteHumidityGm3 += 0.003f;
    fixture.control.update(fixture.state, now);
    TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                      static_cast<int>(fixture.state.phase));
  }
}

void test_control_hold_cooling_does_not_arm_rewetting() {
  ControlFixture fixture;
  fixture.state.setpoints.maxDurationSeconds = 10;
  fixture.state.setpoints.holdTemperatureC = 40.0f;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.start();
  fixture.control.update(fixture.state, 11000);
  fixture.flat(41000, 431000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.state.air.temperatureC = 40.0f;
  fixture.control.update(fixture.state, 432000);
  // A wet first window at the Hold target cannot arm itself.
  for (uint32_t now = 462000; now <= 642000; now += 30000) {
    fixture.state.air.absoluteHumidityGm3 += 0.003f;
    fixture.control.update(fixture.state, now);
  }
  fixture.control.update(fixture.state, 642001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
}

void test_control_deadline_and_purge_count_window_once() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  // Make the deliberate purge-trigger step a quiet completed window, so
  // counting it twice would prematurely satisfy the two-window condition.
  fixture.state.setpoints.drynessSlopeGm3PerHour = 10.0f;
  fixture.start();
  fixture.flat(31000, 181000);
  fixture.state.air.absoluteHumidityGm3 = 0.6f;
  fixture.control.update(fixture.state, 211000);
  TEST_ASSERT_EQUAL_UINT8(2, fixture.state.purgePhase);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  fixture.control.update(fixture.state, 211001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
}

void test_control_partial_window_is_not_carried_through_purge() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 30;
  fixture.start();
  fixture.control.update(fixture.state, 31000);
  fixture.state.air.absoluteHumidityGm3 = 6.0f;
  fixture.control.update(fixture.state, 61000);
  TEST_ASSERT_EQUAL_UINT8(2, fixture.state.purgePhase);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.ahSlopeSamples);
  fixture.state.air.absoluteHumidityGm3 = 0.1f;
  fixture.control.update(fixture.state, 62000);
  fixture.control.update(fixture.state, 242000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.ahSlopeSamples);
  fixture.flat(272000, 422000);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
  fixture.control.update(fixture.state, 452000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
}

void test_control_paused_hold_requires_a_new_quiet_window() {
  ControlFixture fixture;
  fixture.state.setpoints.maxDurationSeconds = 10;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.start();
  fixture.control.update(fixture.state, 11000);
  fixture.flat(41000, 221000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  fixture.machine.pause(fixture.state, 222000);
  fixture.control.update(fixture.state, 222000);
  fixture.machine.resume(fixture.state, 223000);
  fixture.control.update(fixture.state, 223000);
  for (uint32_t now = 253000; now <= 433000; now += 30000) {
    fixture.state.air.absoluteHumidityGm3 += 0.003f;
    fixture.control.update(fixture.state, now);
  }
  fixture.control.update(fixture.state, 433001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_FALSE(fixture.state.rewetDetected);
}

void test_state_machine_zero_threshold_ignores_stale_dryness_events() {
  ControlFixture fixture;
  fixture.state.setpoints.maxDurationSeconds = 10;
  fixture.state.setpoints.drynessSlopeGm3PerHour = 0.0f;
  fixture.start();
  fixture.state.drynessChecks = 255;
  fixture.machine.update(fixture.state, 2000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
  fixture.machine.update(fixture.state, 11000);
  fixture.state.rewetDetected = true;
  fixture.machine.update(fixture.state, 12000);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_FALSE(fixture.state.rewetDetected);
}

void test_control_pause_between_ticks_discards_partial_window() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.start();
  fixture.flat(31000, 181000);
  TEST_ASSERT_EQUAL_UINT8(7, fixture.state.ahSlopeSamples);
  fixture.machine.pause(fixture.state, 181010);
  fixture.machine.resume(fixture.state, 181020);
  fixture.control.update(fixture.state, 181030);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.ahSlopeSamples);
  fixture.control.update(fixture.state, 211030);
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
}

void test_control_pause_between_ticks_clears_hold_arming() {
  ControlFixture fixture;
  fixture.state.setpoints.maxDurationSeconds = 10;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.start();
  fixture.control.update(fixture.state, 11000);
  fixture.flat(41000, 221000);
  TEST_ASSERT_EQUAL_UINT8(1, fixture.state.drynessChecks);
  // Equal command timestamps must still invalidate the old Hold baseline.
  fixture.machine.pause(fixture.state, 222000);
  fixture.machine.resume(fixture.state, 222000);
  fixture.control.update(fixture.state, 222000);
  for (uint32_t now = 253000; now <= 433000; now += 30000) {
    fixture.state.air.absoluteHumidityGm3 += 0.003f;
    fixture.control.update(fixture.state, now);
  }
  fixture.control.update(fixture.state, 433001);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Hold),
                    static_cast<int>(fixture.state.phase));
}

void test_control_resume_clears_verdict_before_state_machine_update() {
  ControlFixture fixture;
  fixture.state.setpoints.ventilation.minSealSeconds = 210;
  fixture.start();
  fixture.flat(31000, 421000);
  TEST_ASSERT_EQUAL_UINT8(2, fixture.state.drynessChecks);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
  fixture.machine.pause(fixture.state, 421010);
  fixture.machine.resume(fixture.state, 421020);
  fixture.control.update(fixture.state, 421030);
  TEST_ASSERT_EQUAL(static_cast<int>(DryingPhase::Drying),
                    static_cast<int>(fixture.state.phase));
  TEST_ASSERT_EQUAL_UINT8(0, fixture.state.drynessChecks);
}

int runTests() {
  UNITY_BEGIN();
  RUN_TEST(test_absolute_humidity_round_trip);
  RUN_TEST(test_pid_output_is_bounded_and_rate_limited);
  RUN_TEST(test_state_machine_timed_run);
  RUN_TEST(test_state_machine_rejects_non_run_modes);
  RUN_TEST(test_weight_sensors_are_optional_for_safety);
  RUN_TEST(test_actuator_interlock_forces_fan_when_heating);
  RUN_TEST(test_actuator_interlock_idle_without_heat);
  RUN_TEST(test_thermostat_holds_band);
  RUN_TEST(test_thermostat_respects_min_switch_interval);
  RUN_TEST(test_heater_setpoint_ceiling);
  RUN_TEST(test_cascade_bounds_heater_setpoint_and_power);
  RUN_TEST(test_purge_scheduler_cycle);
  RUN_TEST(test_purge_scheduler_respects_min_seal);
  RUN_TEST(test_ah_slope_estimator_linear_rise);
  RUN_TEST(test_ah_slope_estimator_flat_and_reset);
  RUN_TEST(test_control_pulse_ventilation_and_dryness);
  RUN_TEST(test_state_machine_preset_dry_then_hold);
  RUN_TEST(test_state_machine_manual_stops_at_duration);
  RUN_TEST(test_control_dryness_streak_from_sealed_windows);
  RUN_TEST(test_control_flat_ah_completes_without_extra_purges);
  RUN_TEST(test_control_short_plan_still_requires_eight_samples);
  RUN_TEST(test_control_warmup_does_not_count_dry_windows);
  RUN_TEST(test_control_temperature_gap_discards_partial_window);
  RUN_TEST(test_control_resume_starts_fresh_measurement_window);
  RUN_TEST(test_control_new_run_during_drying_resets_windows);
  RUN_TEST(test_control_initial_purge_and_settle_do_not_count_samples);
  RUN_TEST(test_control_time_ceiling_keeps_hold_with_zero_streak);
  RUN_TEST(test_control_hold_requires_quiet_then_new_wet_window);
  RUN_TEST(test_control_zero_threshold_disables_dryness_and_redry);
  RUN_TEST(test_control_hold_cooling_does_not_arm_rewetting);
  RUN_TEST(test_control_deadline_and_purge_count_window_once);
  RUN_TEST(test_control_partial_window_is_not_carried_through_purge);
  RUN_TEST(test_control_paused_hold_requires_a_new_quiet_window);
  RUN_TEST(test_state_machine_zero_threshold_ignores_stale_dryness_events);
  RUN_TEST(test_control_pause_between_ticks_discards_partial_window);
  RUN_TEST(test_control_pause_between_ticks_clears_hold_arming);
  RUN_TEST(test_control_resume_clears_verdict_before_state_machine_update);
  return UNITY_END();
}

#ifdef ARDUINO
void setup() {
  delay(1000);
  runTests();
}

void loop() {}
#else
int main(int argc, char** argv) {
  return runTests();
}
#endif
