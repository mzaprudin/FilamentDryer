#pragma once

#include <stdint.h>

#include "config/AppConfig.h"

enum class DryingMode : uint8_t {
  Idle,
  TimedPreset,
  TimedManual,
  Continuous,
  Cooldown,
  Calibration,
  Fault
};

enum class DryingPhase : uint8_t {
  Idle,
  Precheck,
  Warmup,
  Drying,
  Hold,     // spool dry: keep it at storage temperature, watch for re-wetting
  Paused,
  Finish,
  Cooldown,
  Fault
};

enum class UiScreen : uint8_t {
  Dashboard,
  MainMenu,
  ManualSetup,
  ContinuousSetup,
  CalibrationMenu,
  ScaleSetup,
  DriftConfirm,
  WifiSetup
};

// Temperature compensation band count (see defaults::kWeightCalBandWidthC).
enum : uint8_t { kWeightCalBands = 16 };

// Temperature compensation table for one load cell: a multiplier per 5 °C
// band measured during a heat/cool drift calibration run. coeff[i] corrects
// the raw reading in band i back to the reference (room) temperature.
struct WeightCalTable {
  bool valid = false;
  uint8_t filledBands = 0;
  float coeff[kWeightCalBands];

  WeightCalTable() {
    for (uint8_t i = 0; i < kWeightCalBands; ++i) coeff[i] = 1.0f;
  }
};

// Live status of a drift calibration session.
struct CalibrationInfo {
  bool active = false;
  uint8_t phase = 0;  // 0 idle, 1 heat, 2 cool
  uint16_t points = 0;
  float targetTempC = 0.0f;
  float startTempC = 0.0f;
};

// Snapshot of the local menu the UiService wants on the display. The display
// derives row labels itself so this stays small enough to copy by value.
struct UiState {
  UiScreen screen = UiScreen::Dashboard;
  uint8_t cursor = 0;        // selected row within the current screen
  uint8_t scrollOffset = 0;  // first visible row for scrolling lists
  uint8_t editField = 0;     // 0 = navigation, 1/2 = field being edited
  float manualTemperatureC = 45.0f;
  uint32_t manualDurationSeconds = 6UL * 3600UL;
  float continuousTemperatureC = 45.0f;
  uint8_t targetSpool = 1;      // scale-calibration target: 1 or 2
  uint16_t knownGrams = 1000;   // reference mass for scale calibration
  // Short transient message ("TARED", "SAVED"...); empty when nothing to show.
  char toast[20] = "";
};

enum class FaultCode : uint8_t {
  None,
  NtcInvalid,
  HeaterOverTemperature,
  AirSensorInvalid,
  WeightSensorOneInvalid,
  WeightSensorTwoInvalid,
  WarmupTimeout,
  ConfigurationInvalid,
  WatchdogReset
};

struct AirReading {
  float temperatureC = 0.0f;
  float relativeHumidity = 0.0f;
  float absoluteHumidityGm3 = 0.0f;
  bool valid = false;
  uint32_t timestamp = 0;
};

struct HeaterReading {
  int raw = 0;
  float temperatureC = 0.0f;
  bool valid = false;
  uint32_t timestamp = 0;
};

struct WeightReading {
  int32_t raw = 0;
  float grams = 0.0f;
  bool valid = false;
  uint32_t timestamp = 0;
};

struct ActuatorState {
  float heaterPower = 0.0f;
  uint8_t fanPower = 0;
  uint16_t ventAngle = 0;
};

// Pulse-ventilation plan for the humidity loop: the vent stays sealed while
// moisture released by the filament accumulates (and is measured), and opens
// in short purges to exchange the chamber air. Mirrors the preset
// VentilationProfile with runtime defaults for manual runs.
struct VentilationPlan {
  uint8_t circulationFanDuty = 25;  // fan while sealed (heater airflow)
  uint8_t purgeFanDuty = 100;       // fan while purging/settling
  uint16_t ventClosedAngle = 15;
  uint16_t ventOpenAngle = 90;
  uint32_t minSealSeconds = 600;    // min sealed time before a purge may start
  uint32_t settleSeconds = 180;     // vent stays open; AH sampling remains paused
  uint32_t maxPurgeSeconds = 45;    // hard cap on vent-open time
};

inline VentilationPlan toVentilationPlan(const VentilationProfile& profile) {
  VentilationPlan plan;
  plan.circulationFanDuty = profile.minimumFan;
  plan.purgeFanDuty = profile.maximumFan;
  plan.ventClosedAngle = profile.closedAngle;
  plan.ventOpenAngle = profile.openAngle;
  plan.minSealSeconds = profile.purgePeriodSeconds;
  plan.maxPurgeSeconds = profile.purgeDurationSeconds;
  return plan;
}

struct Setpoints {
  float airTemperatureC = 0.0f;
  // Storage temperature held once the spool is dry (Hold phase). Zero means
  // "same as airTemperatureC".
  float holdTemperatureC = 0.0f;
  float relativeHumidity = 0.0f;
  // Sealed-window AH slope below which the spool counts as dry (g/m^3 per
  // hour). Zero disables the dryness criterion (run to the time ceiling).
  float drynessSlopeGm3PerHour = 0.0f;
  float heaterLimitC = 0.0f;
  // TimedManual: plain countdown. TimedPreset: min = active-drying floor,
  // max = safety ceiling (0 = unlimited).
  uint32_t durationSeconds = 0;
  uint32_t minDurationSeconds = 0;
  uint32_t maxDurationSeconds = 0;
  VentilationPlan ventilation;
};

struct DeviceState {
  AirReading air;
  HeaterReading heater;
  WeightReading spoolOne;
  WeightReading spoolTwo;
  ActuatorState actuators;
  // Cascade inner-loop target for the heater (NTC), derived by the air PID.
  // Zero when no run is active.
  float heaterSetpointC = 0.0f;
  // Dryness telemetry: least-squares slope of absolute humidity over the
  // current sealed window (g/m^3 per hour) and its sample count, plus the
  // pulse-ventilation phase (0 sealed, 1 settle, 2 purge). drynessChecks
  // counts consecutive valid sealed windows whose slope stayed below the
  // preset threshold. Windows also finish by time, without opening the vent.
  float ahSlopeGm3PerHour = 0.0f;
  uint8_t ahSlopeSamples = 0;
  uint8_t purgePhase = 0;
  uint8_t drynessChecks = 0;
  // One-shot internal event: a quiet Hold window has been followed by a new
  // wet window. Not persisted or serialized; a zero streak alone is no event.
  bool rewetDetected = false;
  // Commands can pause and resume between two control ticks. Latch that
  // boundary so the next tick cannot reuse an old window or Hold baseline.
  bool measurementResetPending = false;
  Setpoints setpoints;
  DryingMode mode = DryingMode::Idle;
  DryingPhase phase = DryingPhase::Idle;
  FaultCode fault = FaultCode::None;
  // Human-readable tag of the active run ("PLA", "MANUAL", "CONTINUOUS"...).
  char runLabel[20] = "";
  uint32_t phaseStartedAt = 0;
  uint32_t runStartedAt = 0;
  uint32_t remainingSeconds = 0;
  bool wifiConnected = false;
  bool apActive = false;
  // Setup hotspot raised manually from the menu (WPA2-protected): network
  // name, per-boot random password, softAP address and the remaining window
  // in seconds. Empty when the hotspot is down.
  char apSsid[24] = "";
  char apPassword[13] = "";
  char apAddress[16] = "";
  uint16_t apRemainingSeconds = 0;
  // Wall clock from NTP: epochSeconds (Unix) is valid only while timeSynced.
  bool timeSynced = false;
  uint32_t epochSeconds = 0;
  // Current station or AP address for the web panel, empty when offline.
  char ipAddress[16] = "";
  // Friendlier address to show: "dryer.local" once mDNS is up, otherwise
  // falls back to the raw IP.
  char webAddress[24] = "";
  CalibrationInfo calibration;
  bool otaInProgress = false;
};

struct EventRecord {
  uint32_t timestamp;
  // Unix epoch when the wall clock was synchronized, 0 otherwise.
  uint32_t epoch = 0;
  char type[24];
  char message[96];
};
