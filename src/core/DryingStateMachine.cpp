#include "core/DryingStateMachine.h"

#include <cstdio>

#include "config/Defaults.h"

void DryingStateMachine::begin(DeviceState& state, uint32_t now) {
  state.mode = DryingMode::Idle;
  state.phase = DryingPhase::Idle;
  state.fault = FaultCode::None;
  state.phaseStartedAt = now;
  state.runStartedAt = 0;
  state.remainingSeconds = 0;
  state.drynessChecks = 0;
  state.rewetDetected = false;
  state.measurementResetPending = true;
  state.runLabel[0] = '\0';
  durationSeconds_ = 0;
  pausedAt_ = 0;
  pausedFrom_ = DryingPhase::Idle;
}

bool DryingStateMachine::start(DeviceState& state, DryingMode mode,
                                const Setpoints& setpoints, uint32_t now) {
  if (mode != DryingMode::TimedPreset && mode != DryingMode::TimedManual &&
      mode != DryingMode::Continuous && mode != DryingMode::Calibration) {
    return false;
  }
  durationSeconds_ = setpoints.durationSeconds;
  minDurationSeconds_ = setpoints.minDurationSeconds;
  maxDurationSeconds_ = setpoints.maxDurationSeconds;
  state.mode = mode;
  state.phase = DryingPhase::Precheck;
  state.setpoints = setpoints;
  state.fault = FaultCode::None;
  state.phaseStartedAt = now;
  state.runStartedAt = 0;
  state.drynessChecks = 0;
  state.rewetDetected = false;
  state.measurementResetPending = true;
  // Continuous and calibration modes have no countdown; the UI shows elapsed
  // time instead. Preset runs count down to their safety ceiling.
  if (mode == DryingMode::Continuous || mode == DryingMode::Calibration) {
    state.remainingSeconds = 0UL;
  } else if (mode == DryingMode::TimedPreset) {
    state.remainingSeconds = maxDurationSeconds_;
  } else {
    state.remainingSeconds = durationSeconds_;
  }
  std::snprintf(state.runLabel, sizeof(state.runLabel), "%s",
                mode == DryingMode::Continuous
                    ? "CONTINUOUS"
                    : (mode == DryingMode::Calibration
                           ? "CALIB"
                           : (mode == DryingMode::TimedManual ? "MANUAL"
                                                              : "PRESET")));
  return true;
}

void DryingStateMachine::pause(DeviceState& state, uint32_t now) {
  if (state.phase == DryingPhase::Warmup || state.phase == DryingPhase::Drying ||
      state.phase == DryingPhase::Hold) {
    pausedFrom_ = state.phase;
    state.phase = DryingPhase::Paused;
    state.rewetDetected = false;
    state.measurementResetPending = true;
    pausedAt_ = now;
  }
}

void DryingStateMachine::resume(DeviceState& state, uint32_t now) {
  if (state.phase == DryingPhase::Paused) {
    if (state.runStartedAt != 0 && pausedAt_ != 0) {
      state.runStartedAt += static_cast<uint32_t>(now - pausedAt_);
    }
    state.phase = pausedFrom_;
    state.rewetDetected = false;
    state.measurementResetPending = true;
    state.phaseStartedAt = now;
    pausedAt_ = 0;
    pausedFrom_ = DryingPhase::Idle;
  }
}

void DryingStateMachine::stop(DeviceState& state, uint32_t now) {
  state.mode = DryingMode::Idle;
  state.phase = DryingPhase::Cooldown;
  state.fault = FaultCode::None;
  state.phaseStartedAt = now;
  state.remainingSeconds = 0;
  state.rewetDetected = false;
  state.measurementResetPending = true;
}

void DryingStateMachine::fault(DeviceState& state, FaultCode code, uint32_t now) {
  state.mode = DryingMode::Fault;
  state.phase = DryingPhase::Fault;
  state.fault = code;
  state.rewetDetected = false;
  state.measurementResetPending = true;
  state.phaseStartedAt = now;
}

void DryingStateMachine::update(DeviceState& state, uint32_t now) {
  if (state.phase == DryingPhase::Precheck) {
    state.phase = DryingPhase::Warmup;
    state.phaseStartedAt = now;
  } else if (state.phase == DryingPhase::Warmup) {
    if (state.air.valid &&
        state.air.temperatureC >= state.setpoints.airTemperatureC - 1.0f) {
      state.phase = DryingPhase::Drying;
      state.phaseStartedAt = now;
      state.runStartedAt = now;
    }
  } else if (state.phase == DryingPhase::Drying) {
    if (state.mode == DryingMode::TimedManual) {
      const uint32_t elapsed = (now - state.runStartedAt) / 1000UL;
      state.remainingSeconds =
          elapsed >= durationSeconds_ ? 0 : durationSeconds_ - elapsed;
      if (state.remainingSeconds == 0) {
        state.phase = DryingPhase::Finish;
        state.phaseStartedAt = now;
      }
    } else if (state.mode == DryingMode::TimedPreset) {
      // Dry-to-completion: the preset run keeps drying until the spool is
      // dry (min time served AND the AH slope stayed quiet for
      // kDrynessStableWindows sealed windows) or the safety ceiling expires;
      // either way it settles into Hold instead of finishing.
      const uint32_t elapsed = (now - state.runStartedAt) / 1000UL;
      if (maxDurationSeconds_ > 0UL) {
        state.remainingSeconds = elapsed >= maxDurationSeconds_
                                     ? 0
                                     : maxDurationSeconds_ - elapsed;
      }
      const bool minElapsed = elapsed >= minDurationSeconds_;
      const bool dry = state.setpoints.drynessSlopeGm3PerHour > 0.0f &&
                       state.drynessChecks >= defaults::kDrynessStableWindows;
      const bool maxElapsed =
          maxDurationSeconds_ > 0UL && elapsed >= maxDurationSeconds_;
      if (maxElapsed || (minElapsed && dry)) {
        state.phase = DryingPhase::Hold;
        state.phaseStartedAt = now;
        state.remainingSeconds = 0;
        state.rewetDetected = false;
      }
    }
  } else if (state.phase == DryingPhase::Hold) {
    // Only a new confirmed moisture event may restart drying. In particular,
    // reaching the time ceiling with a zero dry streak must remain in Hold.
    const bool rewet = state.rewetDetected;
    state.rewetDetected = false;
    if (state.mode == DryingMode::TimedPreset &&
        state.setpoints.drynessSlopeGm3PerHour > 0.0f && rewet) {
      state.phase = DryingPhase::Drying;
      state.phaseStartedAt = now;
      state.runStartedAt = now;
      state.remainingSeconds = maxDurationSeconds_;
    }
  } else if (state.phase == DryingPhase::Finish &&
             static_cast<uint32_t>(now - state.phaseStartedAt) >= 30000UL) {
    state.mode = DryingMode::Idle;
    state.phase = DryingPhase::Idle;
  } else if (state.phase == DryingPhase::Cooldown &&
             static_cast<uint32_t>(now - state.phaseStartedAt) >= 30000UL) {
    state.mode = DryingMode::Idle;
    state.phase = DryingPhase::Idle;
  }
}
