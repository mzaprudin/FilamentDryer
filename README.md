# FilamentDryer

[Русская версия](README.RU.md)

Firmware for a two-spool filament dryer based on the ESP32-WROOM-32D.
The design goal: **keep plastic dry** — a preset does not just hold a
temperature for a fixed time, it dries the spool until it is actually dry,
then keeps it at storage temperature and automatically re-dries when moisture
comes back (lid opened, fresh spool dropped in).

Current firmware version: **0.2.1**

## Key features

- **Dry-to-completion presets** — each preset defines a minimum active-drying
  time, a safety ceiling, a storage (hold) temperature and a dryness
  threshold. Drying runs until the spool stops releasing moisture, then the
  chamber settles into a low-power Hold phase and watches for re-wetting.
- **Dryness estimation from the chamber air** — with the vent sealed, the
  absolute-humidity (AH) rise is the moisture released by the filament. A
  least-squares slope over a sealed measurement window (g/m³ per hour) is the
  dryness signal; weight sensors are not required for it.
- **Temperature cascade** — the air-temperature PID outputs a *heater (NTC)
  setpoint* bounded by `[air target .. heater limit − 5 °C]`; an inner
  thermostat (±1 °C hysteresis) switches the MOSFET fully on/off. The heater
  is never driven hotter than the chamber actually needs.
- **Pulse ventilation** — `Sealed → Purge → Settle` cycle: the vent stays
  closed while moisture accumulates (and is measured); a short purge
  exchanges the air once AH rises ≥ 0.5 g/m³ above the window baseline or
  hits the saturation guard. The fan runs at a quiet circulation duty while
  sealed and at boost duty only during purges.
- **Safety first** — a fan interlock at the actuator layer guarantees airflow
  whenever the heater may be powered (`heating ⇒ fan ≥ floor`, applied
  fan-first), on top of the NTC limit gates and the hardware thermal fuse.
- **Two HX711 load cells** — per-spool weight telemetry with tare/scale
  calibration and a 16-band (5 °C) thermal-drift compensation table.
- **Web panel + REST API** over Wi-Fi with mDNS (`dryer.local`), telemetry
  history in LittleFS, NTP time sync with a configurable UTC offset,
  optional HTTP Basic authentication.
- **OTA updates over Wi-Fi** — both ArduinoOTA and a web endpoint; no USB
  cable needed after the first flash.
- **Local UI** — SH1106 OLED + rotary encoder menu.
- **19 native unit tests** for the control logic, runnable on the host.

## Hardware

| Function | GPIO | Notes |
|---|---:|---|
| I²C SDA / SCL | 21 / 22 | AHT30 + display, 400 kHz |
| Encoder A / B / button | 32 / 33 / 25 | |
| Heater MOSFET | 26 | 24 V PTC via logic-level N-MOSFET |
| Fans MOSFET | 27 | two 24 V fans in parallel, 20 kHz PWM |
| Vent servo | 14 | powered from a separate 5–6 V rail |
| NTC (heater slot) | 34 | ADC1, 10 kΩ / B=3950 divider |
| HX711 #1 DOUT/SCK | 16 / 17 | |
| HX711 #2 DOUT/SCK | 18 / 19 | |

Sensors: AHT30 (chamber air T/RH), NTC in the PTC slot (heater temperature),
2 × HX711 with 4-wire load cells. A 120 °C thermal fuse in series with the
heater remains an independent hardware layer that software does not replace.

Power design notes, NTC divider math and layout recommendations live in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Control architecture

```
air target ──► [air PID] ──► heater (NTC) setpoint ──► [thermostat ±1°C] ──► MOSFET on/off
                   ▲                 bounded by [target .. limit−5°C]
                   │
              AHT30 air T

AH target ──► [PurgeScheduler]  Sealed ──► Purge ──► Settle ──► Sealed ...
                   │                ▲         │
                   │                │         └─ purge: vent open, fan boost
                   │                └─ trigger: AH rise ≥ 0.5 g/m³ or saturation
                   │
             [AhSlopeEstimator]  slope of AH over the sealed window
                   │
             dryness: slope < threshold for 2 consecutive windows
```

- **Fan policy**: circulation duty while sealed (heater airflow + mixing),
  boost during purge/settle; always at least the configured floor when the
  heater is on. The fan never runs wide open just because it can.
- **Hold phase**: after drying, the chamber holds the preset storage
  temperature and keeps measuring. If moisture returns
  (`drynessChecks` reset), the machine automatically re-enters Drying with a
  fresh time budget.
- **Manual mode** stays a plain countdown; **continuous mode** holds
  conditions until stopped.

Dryness thresholds (0.3 g/m³/h) and window timings are **empirical first
guesses** — they are exposed in telemetry (`dryness.*` in `/api/state`) and
should be recalibrated from logs on real spools.

## Building and flashing

```powershell
# full build
pio run -e esp32dev

# flash over USB (first time)
pio run -e esp32dev -t upload
pio run -e esp32dev -t uploadfs        # web panel assets (LittleFS)

# flash over Wi-Fi afterwards (ArduinoOTA)
pio run -e esp32dev_ota -t upload
# or with a raw IP if mDNS is blocked:
pio run -e esp32dev_ota -t upload --upload-port 192.168.1.238
```

During an OTA transfer the firmware temporarily unsubscribes the main task
from the watchdog (ArduinoOTA receives the image inside a blocking
`handle()` call) and pauses LittleFS telemetry writes.

## First start

1. The setup hotspot is never opened automatically: a device without saved
   Wi-Fi credentials (or with an unreachable router) simply stays offline and
   keeps working standalone from the screen and the encoder.
2. To configure Wi-Fi open the main menu, pick `WIFI SETUP` and select
   `Start`. The device raises a WPA2-protected hotspot (SSID
   `dryer-setup-XXXX`, where XXXX identifies the unit) with a random password
   shown on the display and the serial console. The hotspot closes itself
   after 10 minutes; picking `Start` again restarts the window. Connect to
   the hotspot and configure your home network through the web panel at
   `http://192.168.4.1`.
3. Headless fallback: if the display or the encoder is dead, hold the encoder
   button while powering on — the hotspot comes up and its password is
   printed on the serial console.
4. On the home network the panel is reachable at `http://dryer.local`
   (mDNS; the IP is also shown on the display).
5. Authentication is optional: an empty web password disables it. When set,
   the login defaults to `admin` (configurable) and the same password also
   protects ArduinoOTA. Set a password before powering the heater side.

Before connecting power stages, verify that GPIO26 stays low at boot, on
reset, on NTC faults and during OTA.

## REST API (short)

```
GET  /api/state        full status incl. dryness.* telemetry and firmware version
GET  /api/config       settings + preset list
PUT  /api/config       change settings (Wi-Fi, auth, fan floor, scales...)
POST /api/run          start a run {mode, preset | temperatureC, durationSeconds...}
POST /api/stop         stop
POST /api/pause        pause / resume toggle
GET  /api/history      telemetry (JSONL window)
GET  /api/events       event log
GET  /api/calibration  weight-calibration status
POST /api/calibration  tare / known-mass / drift-run operations
POST /api/ota          firmware upload (multipart)
```

## Remote debugging

`tools/remote.ps1` polls the panel over Wi-Fi — no USB cable attached:

```powershell
powershell -File tools\remote.ps1 status              # one full snapshot
powershell -File tools\remote.ps1 watch 3600          # live telemetry -> telemetry-*.jsonl
powershell -File tools\remote.ps1 history out.json    # download telemetry
powershell -File tools\remote.ps1 events out.json     # download event log
powershell -File tools\remote.ps1 ota                 # flash .pio\build\esp32dev_ota\firmware.bin
```

Add `-HostName 192.168.x.x` when mDNS is blocked on your network.

## Tests

Native unit tests cover the actuator interlock, thermostat band, cascade
bounds, purge cycle, AH slope regression, dry→hold transitions and the
manual-mode countdown:

```powershell
pio test -e native
```

## Project layout

```
include/config/    BoardConfig.h (pins), Defaults.h (tunables), AppConfig.h (persisted config)
include/control/   PidController, HeaterThermostat, PurgeScheduler, AhSlopeEstimator
include/core/      DryingStateMachine (phases: Warmup/Drying/Hold/Paused/...)
include/services/  Sensor/Control/Actuator/Safety/Storage/Network/Web/Ui/Calibration
include/drivers/   AHT30, NTC, HX711, SH1106, servo vent, MOSFET outputs, encoder
data/              web panel (LittleFS)
docs/ARCHITECTURE.md  hardware and control design rationale
```

## Status and roadmap

- [x] Control core: cascade, pulse ventilation, dryness estimation, Hold
- [x] Safety: fan interlock, NTC gates, watchdog, safe outputs
- [x] Web panel, REST API, telemetry history, events
- [x] OTA over Wi-Fi, remote tooling
- [ ] Calibrate dryness thresholds on real spools (logged `slope` data)
- [ ] Long-run soak tests, thermal validation of the assembled chamber
- [ ] Optional: per-material thresholds, WebSocket live graphs

## License

[MIT](LICENSE)
