#pragma once

#include <stdint.h>

// Pulse-ventilation cycle for the humidity loop. The vent stays closed while
// moisture released by the filament accumulates in the sealed chamber (the
// absolute-humidity rise is the dryness signal); a short purge exchanges the
// air once enough moisture has collected. Sealed -> Purge -> Settle -> Sealed.
enum class PurgePhase : uint8_t {
  Sealed = 0,  // vent closed, AH rising: measurement window
  Settle = 1,  // vent still open, fans mixing before the next sealed window
  Purge = 2    // vent open, fans at boost: exchanging chamber air
};

struct PurgeParams {
  uint32_t minSealMs = 600000;   // min sealed time before a purge may start
  uint32_t settleMs = 180000;    // mixing time with vent open after purging
  uint32_t maxPurgeMs = 45000;   // hard cap on vent-open time
  float riseTriggerGm3 = 0.5f;   // AH rise since seal start justifying a purge
};

class PurgeScheduler {
 public:
  // Starts a fresh cycle. The first purge is allowed immediately (the chamber
  // still holds room air, so exchanging it early is always useful); the
  // baseline for later rise-trigger decisions is captured when the first
  // settle completes.
  void begin(const PurgeParams& params, uint32_t now);
  PurgePhase update(float absoluteHumidityGm3, float purgeFloorGm3, uint32_t now);
  PurgePhase phase() const { return phase_; }
  uint32_t phaseElapsedMs(uint32_t now) const {
    return static_cast<uint32_t>(now - phaseStartedAt_);
  }

 private:
  PurgeParams params_{};
  PurgePhase phase_ = PurgePhase::Sealed;
  uint32_t phaseStartedAt_ = 0;
  uint32_t sealStartedAt_ = 0;
  float sealBaselineGm3_ = 0.0f;
};
