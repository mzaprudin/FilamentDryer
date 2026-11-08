#pragma once

#include "config/AppConfig.h"
#include "control/AhSlopeEstimator.h"
#include "control/HeaterThermostat.h"
#include "control/PidController.h"
#include "control/PurgeScheduler.h"
#include "core/DryingStateMachine.h"
#include "domain/Interfaces.h"
#include "services/SafetyService.h"

class ControlService {
 public:
  ControlService(const AppConfig& config, SafetyService& safety,
                 DryingStateMachine& stateMachine);
  void begin();
  void update(DeviceState& state, uint32_t now);

 private:
  void restartPulseCycle(DeviceState& state, uint32_t now);
  void resetAhWindow(DeviceState& state);
  void resetDrynessTracking(DeviceState& state);
  void updateAhWindow(DeviceState& state, uint32_t now, bool closing);
  void evaluateAhWindow(DeviceState& state);

  const AppConfig& config_;
  SafetyService& safety_;
  DryingStateMachine& stateMachine_;
  PidController temperaturePid_;
  HeaterThermostat thermostat_;
  PurgeScheduler purge_;
  AhSlopeEstimator ahSlope_;
  DryingPhase lastPhase_ = DryingPhase::Idle;
  uint8_t stableWindows_ = 0;
  bool holdRewetArmed_ = false;
  bool ahWindowActive_ = false;
  uint32_t ahWindowStartedAt_ = 0;
  uint32_t lastAhSampleAt_ = 0;
  float cascadeFloor_ = 0.0f;
  float cascadeCeiling_ = 0.0f;
};
