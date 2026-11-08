#include "services/ControlService.h"

#include <algorithm>
#include <cmath>

#include "config/Defaults.h"
#include "control/HumidityMath.h"

ControlService::ControlService(const AppConfig& config, SafetyService& safety,
                               DryingStateMachine& stateMachine)
    : config_(config), safety_(safety), stateMachine_(stateMachine) {}

void ControlService::begin() {
  temperaturePid_.begin(config_.temperaturePid, defaults::kControlPeriodMs);
}

void ControlService::restartPulseCycle(DeviceState& state, uint32_t now) {
  PurgeParams params;
  params.minSealMs = state.setpoints.ventilation.minSealSeconds * 1000UL;
  params.settleMs = state.setpoints.ventilation.settleSeconds * 1000UL;
  params.maxPurgeMs = state.setpoints.ventilation.maxPurgeSeconds * 1000UL;
  params.riseTriggerGm3 = defaults::kPurgeRiseTriggerGm3;
  purge_.begin(params, now);
  resetDrynessTracking(state);
}

void ControlService::resetAhWindow(DeviceState& state) {
  ahSlope_.reset();
  ahWindowActive_ = false;
  lastAhSampleAt_ = 0;
  state.ahSlopeGm3PerHour = 0.0f;
  state.ahSlopeSamples = 0;
}

void ControlService::resetDrynessTracking(DeviceState& state) {
  resetAhWindow(state);
  stableWindows_ = 0;
  state.drynessChecks = 0;
  holdRewetArmed_ = false;
  state.rewetDetected = false;
}

void ControlService::evaluateAhWindow(DeviceState& state) {
  const float threshold = state.setpoints.drynessSlopeGm3PerHour;
  if (threshold <= 0.0f) {
    stableWindows_ = 0;
    holdRewetArmed_ = false;
    state.rewetDetected = false;
  } else if (state.ahSlopeGm3PerHour < threshold) {
    if (stableWindows_ < 255) ++stableWindows_;
    if (state.phase == DryingPhase::Hold) holdRewetArmed_ = true;
  } else {
    stableWindows_ = 0;
    if (state.phase == DryingPhase::Hold && holdRewetArmed_) {
      state.rewetDetected = true;
      holdRewetArmed_ = false;
    }
  }
  state.drynessChecks = stableWindows_;
}

void ControlService::updateAhWindow(DeviceState& state, uint32_t now,
                                    bool closing) {
  if (!ahWindowActive_) {
    ahWindowActive_ = true;
    ahWindowStartedAt_ = now;
    ahSlope_.addSample(now, state.air.absoluteHumidityGm3);
    lastAhSampleAt_ = now;
  } else if (static_cast<uint32_t>(now - lastAhSampleAt_) >=
             defaults::kAhSamplePeriodMs) {
    ahSlope_.addSample(now, state.air.absoluteHumidityGm3);
    lastAhSampleAt_ = now;
  }
  state.ahSlopeGm3PerHour = ahSlope_.slopeGm3PerHour();
  state.ahSlopeSamples = ahSlope_.sampleCount();
  const uint32_t minimumSpanMs =
      (defaults::kAhSlopeMinSamples - 1UL) * defaults::kAhSamplePeriodMs;
  const uint32_t windowMs = std::max<uint32_t>(
      state.setpoints.ventilation.minSealSeconds * 1000UL, minimumSpanMs);
  const bool elapsed =
      static_cast<uint32_t>(now - ahWindowStartedAt_) >= windowMs;
  if (ahSlope_.slopeValid() && (elapsed || closing)) {
    // A timed completion and a simultaneous purge are one judgement. The
    // next window shares only this boundary sample, never the whole slope.
    evaluateAhWindow(state);
    resetAhWindow(state);
    if (!closing) {
      ahWindowActive_ = true;
      ahWindowStartedAt_ = now;
      ahSlope_.addSample(now, state.air.absoluteHumidityGm3);
      lastAhSampleAt_ = now;
      state.ahSlopeSamples = ahSlope_.sampleCount();
    }
  }
  if (closing) resetAhWindow(state);
}

void ControlService::update(DeviceState& state, uint32_t now) {
  const bool resetRequested = state.measurementResetPending;
  state.measurementResetPending = false;
  // Clear old verdicts before the state machine can act on them, even when
  // pause/resume restored the same phase between consecutive control ticks.
  if (resetRequested) resetDrynessTracking(state);
  // A fresh start can replace another active run, so capture Precheck before
  // the state machine consumes it and advances to Warmup.
  const bool starting = state.phase == DryingPhase::Precheck;
  stateMachine_.update(state, now);
  // Track phase transitions up front: a run (or resume) must restart the
  // pulse-ventilation cycle, since pauses park the vent at the safe angle.
  // Hold stays active: the chamber keeps controlling at the storage
  // temperature and keeps watching for fresh moisture.
  const bool active = state.phase == DryingPhase::Precheck ||
                      state.phase == DryingPhase::Warmup ||
                      state.phase == DryingPhase::Drying ||
                      state.phase == DryingPhase::Hold;
  const bool wasActive = lastPhase_ == DryingPhase::Precheck ||
                         lastPhase_ == DryingPhase::Warmup ||
                         lastPhase_ == DryingPhase::Drying ||
                         lastPhase_ == DryingPhase::Hold;
  if (starting || (active && (resetRequested || !wasActive))) {
    restartPulseCycle(state, now);
  }
  if (state.phase != lastPhase_) resetDrynessTracking(state);
  lastPhase_ = state.phase;

  if (state.otaInProgress) {
    resetDrynessTracking(state);
    state.actuators.heaterPower = 0.0f;
    state.actuators.fanPower = 60;
    state.actuators.ventAngle = config_.safeVentAngle;
    return;
  }
  if (state.phase == DryingPhase::Idle || state.phase == DryingPhase::Paused ||
      state.phase == DryingPhase::Fault || state.phase == DryingPhase::Finish ||
      state.phase == DryingPhase::Cooldown) {
    state.actuators.heaterPower = 0.0f;
    state.heaterSetpointC = 0.0f;
    state.actuators.fanPower =
        (state.phase == DryingPhase::Finish || state.phase == DryingPhase::Cooldown)
            ? 60
            : 0;
    state.actuators.ventAngle = config_.safeVentAngle;
    state.ahSlopeGm3PerHour = 0.0f;
    state.ahSlopeSamples = 0;
    state.purgePhase = 0;
    return;
  }

  FaultCode fault = FaultCode::None;
  if (!safety_.evaluate(state, now, fault)) {
    resetDrynessTracking(state);
    stateMachine_.fault(state, fault, now);
    state.actuators.heaterPower = 0.0f;
    state.actuators.fanPower = 100;
    state.actuators.ventAngle = config_.safeVentAngle;
    return;
  }

  // Temperature cascade. Outer loop: the air-temperature PID no longer
  // outputs heater power - it outputs a heater (NTC) setpoint bounded by
  // [air target, limit - margin]. Inner loop: the thermostat holds the
  // heater inside a narrow band around that setpoint, so the heater is never
  // driven hotter than the chamber actually needs and the MOSFET switches
  // fully on/off instead of PWM-ing. In Hold the target drops to the preset
  // storage temperature.
  const float airTarget =
      state.phase == DryingPhase::Hold && state.setpoints.holdTemperatureC > 0.0f
          ? state.setpoints.holdTemperatureC
          : state.setpoints.airTemperatureC;
  const float ceiling =
      std::max(airTarget + 1.0f, heaterSetpointCeilingC(state.setpoints.heaterLimitC,
                                                         config_.heaterMaxTemperatureC));
  if (airTarget != cascadeFloor_) resetDrynessTracking(state);
  if (airTarget != cascadeFloor_ || ceiling != cascadeCeiling_) {
    temperaturePid_.setTunings({config_.temperaturePid.kp, config_.temperaturePid.ki,
                                config_.temperaturePid.kd, airTarget, ceiling});
    temperaturePid_.reset(airTarget);
    thermostat_.reset();
    cascadeFloor_ = airTarget;
    cascadeCeiling_ = ceiling;
  }
  state.heaterSetpointC =
      temperaturePid_.compute(state.air.temperatureC, airTarget, now);
  state.actuators.heaterPower =
      thermostat_.update(state.heater.temperatureC, state.heaterSetpointC, now)
          ? 100.0f
          : 0.0f;
  if (!safety_.canHeat(state)) state.actuators.heaterPower = 0.0f;

  // Humidity loop: pulse ventilation. The vent stays closed so moisture
  // released by the filament accumulates in the sealed chamber (the AH rise
  // is the dryness signal); short purges exchange the air once enough
  // moisture has collected.
  const VentilationPlan& plan = state.setpoints.ventilation;
  const float absoluteTarget =
      relativeToAbsoluteHumidity(airTarget, state.setpoints.relativeHumidity);
  const float purgeFloor = absoluteTarget + defaults::kPurgeFloorMarginGm3;
  const PurgePhase previousPurgePhase = purge_.phase();
  const PurgePhase phase =
      purge_.update(state.air.absoluteHumidityGm3, purgeFloor, now);
  state.purgePhase = static_cast<uint8_t>(phase);

  const bool closing = previousPurgePhase == PurgePhase::Sealed &&
                       phase == PurgePhase::Purge;
  const bool measuringPhase = state.phase == DryingPhase::Drying ||
                              state.phase == DryingPhase::Hold;
  // Cooling from Drying to Hold must not arm re-wetting detection. Interrupt
  // the window whenever the air leaves the existing +/-1 C warmup band.
  const bool temperatureReady =
      std::fabs(state.air.temperatureC - airTarget) <= 1.0f;
  if (measuringPhase && temperatureReady &&
      std::isfinite(state.air.absoluteHumidityGm3) &&
      (phase == PurgePhase::Sealed || closing)) {
    updateAhWindow(state, now, closing);
  } else {
    // An interrupted or insufficient window produces no dry/wet verdict.
    resetAhWindow(state);
  }

  const bool ventOpen = phase != PurgePhase::Sealed;
  state.actuators.ventAngle = ventOpen ? plan.ventOpenAngle : plan.ventClosedAngle;
  // Keep a circulation flow whenever the chamber is actively controlled; the
  // fan must never stop while the heater may be powered.
  const float fanDemand =
      static_cast<float>(ventOpen ? plan.purgeFanDuty : plan.circulationFanDuty);
  const float configuredMinimum =
      std::max(0.0f, std::min(100.0f, config_.fanMinimumDuty));
  const float boundedFan =
      std::max(defaults::kActiveFanMinimumDuty,
               std::max(configuredMinimum, std::max(0.0f, std::min(100.0f, fanDemand))));
  state.actuators.fanPower = static_cast<uint8_t>(boundedFan);
}
