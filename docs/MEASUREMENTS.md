# Capturing and calibrating a drying cycle

[Русская версия](MEASUREMENTS.RU.md) · [Architecture](ARCHITECTURE.md) ·
[Project overview](../README.md)

The telemetry tools record the running device and produce a graph from the
original JSONL samples. A useful cycle includes warm-up, active drying, ventilation
and entry into `Hold`. The data can then guide calibration of the moisture-release
threshold separately from load-cell calibration.

## Record the experiment conditions

Each experiment has its own directory, for example `runs/pla-01/`. A manual
`experiment.md` alongside the capture records:

- material, manufacturer, spool identifier, filament and spool mass;
- the filament's initial condition and preparation;
- chamber construction, sensor positions, number of spools and room conditions;
- the firmware commit that was flashed, its version and selected preset;
- changes to the normal setup, lid openings and other observations with times;
- the independent reference used to assess dryness, when available.

The device reports its version through the API. The flashed commit is recorded
from the build used for the experiment; it is not supplied by `/api/state`.
Photographs of the assembled device and panel can accompany the same experiment.

## Capture the full state

Python 3.11 or newer supports the pinned plotting dependency. Capture itself uses
the Python standard library. From the repository root:

```sh
python -m venv .venv
. .venv/bin/activate
python -m pip install -r tools/requirements.txt
python tools/telemetry.py capture --base-url http://dryer.local --duration 36000 --interval 5 --output runs/pla-01
```

On Windows, the activation command is `.venv\Scripts\Activate.ps1`. An IP address
can replace `dryer.local`. When panel authentication is enabled, adding
`--username admin` prompts for the password without putting it in the command
line.

The example records up to ten hours. Recording starts before the run is started
on the device or panel; its duration needs to cover the selected preset and a
period of storage. `Ctrl+C` ends capture and keeps the accumulated samples. An
existing capture directory is protected from accidental overwrite.

The collector uses GET requests only and creates:

| File | Contents |
|---|---|
| `telemetry.jsonl` | complete `/api/state` responses as one `{ts, state}` record per line |
| `config.json` | exposed control settings and preset list, with network/authentication fields omitted |
| `metadata.json` | capture times, interval, status, sample/error counts, observed versions and initial run settings |

`ts` is the computer's UTC timestamp; the state also retains device uptime and
device time when synchronized. Temporary request failures are reported and
retried until the capture duration ends. Authentication failures stop capture.

`tools/remote.ps1 watch` recordings use the same `{ts, state}` format and can be
plotted too. The device's `/api/history` is a different, compact format: it samples
once per minute, rotates its file, and omits the drying phase and number of quiet
windows. Full state capture preserves the information needed to explain a
transition to `Hold`.

## Plot and inspect the cycle

The plotting command exports either PNG or SVG:

```sh
python tools/telemetry.py plot runs/pla-01/telemetry.jsonl --output runs/pla-01/cycle.png
python tools/telemetry.py plot runs/pla-01/telemetry.jsonl --output runs/pla-01/cycle.svg
```

The graph combines air and heater temperatures, absolute humidity and the
humidity slope with its threshold. Phase bands identify purge, settling and
`Hold`. Invalid sensor readings, insufficient slope samples, missing observations
and device restarts leave gaps rather than interpolated values.

The source fields for interpretation are:

| Signal | `/api/state` field |
|---|---|
| Chamber temperature | `air.temperatureC`, `air.valid` |
| Heater temperature and target | `heater.temperatureC`, `heater.valid`, `outputs.heaterSetpointC` |
| Absolute humidity | `air.absoluteHumidityGm3` |
| Slope and sample count | `dryness.ahSlopeGm3PerHour`, `dryness.ahSlopeSamples` |
| Threshold and quiet windows | `dryness.slopeThreshold`, `dryness.stableWindows` |
| Ventilation phase | `dryness.purgePhase`: 0 sealed, 1 settling, 2 purge |
| Drying/storage phase | `phase` |

A zero slope with fewer than eight samples is not a completed measurement.
Changes in `stableWindows`, the minimum/maximum run durations and `phase` explain
whether the preset criterion or the time ceiling permitted `Hold`.

## Calibrate the drying criterion

The starting threshold is 0.3 g/m³/hour. A calibration report compares eligible
closed-window slopes from the wet and late-cycle portions of several runs with
the independent dryness assessment. It records the proposed threshold, the
minimum time and the observations supporting both.

A candidate setting is then checked on a new run, including its transition into
storage. A separate run can capture a lid opening or replacement spool after a
quiet `Hold` window to check automatic re-drying. The report retains the original
JSONL, configuration, firmware commit, plots and notes for each run.

Thresholds and preset times are compiled in
[AppConfig.cpp](../src/config/AppConfig.cpp). The API exposes them for inspection;
changing them requires a new firmware build. Temperature stabilization and the
measurement-window rules are described in the [architecture](ARCHITECTURE.md).

## Calibrate the two load cells

The local `CALIB` menu and web calibration panel use the same service. A reference
mass supports three separate operations:

1. **Tare:** with both platforms empty, `tare` stores their raw offsets.
2. **Scale:** a known mass on the chosen platform sets its counts-per-gram factor.
   The API action is `scale`, with `spool` equal to 0 or 1 and `knownGrams`.
3. **Thermal drift:** reference masses remain on the platforms during
   `drift_start`. The service heats and then cools the chamber, sampling raw
   readings and air temperature every 30 seconds. `drift_cancel` discards a
   partial session.

The drift target is the lower of 80 °C and the configured air ceiling minus 2 °C
(73 °C with the default ceiling). Each sensor has 16 bands, 5 °C wide from a
20 °C base; outlying temperatures use the end bands. A completed run needs at
least three populated bands and a populated reference band to save its table.
Faults and the six-hour session timeout end the run without saving a new table.

`GET /api/calibration` reports the phase, samples, raw readings, scales and saved
coefficient tables. A weight-calibration report retains the reference masses,
room conditions and readings before and after compensation. Weight calibration
and the chamber's moisture-release threshold are separate experiments.

## Published results

A completed hardware example consists of its conditions, original recordings,
configuration and graph. Synthetic fixtures in the test suite exercise the
tools; they are not device measurements. Real cycle recordings and a calibrated
threshold can be added here after the experiments above.
