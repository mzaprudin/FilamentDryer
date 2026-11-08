#!/usr/bin/env python3
"""Read dryer telemetry over HTTP and plot captured, unmodified samples."""

from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import getpass
from http.client import HTTPException
import json
import math
from pathlib import Path
import statistics
import sys
import time
from urllib import error, parse, request


CONTROL_FIELDS = (
    "heaterMaxTemperatureC", "airMaxTemperatureC", "fanMinimumDuty",
    "servoMinUs", "servoMaxUs",
)
SETPOINT_FIELDS = (
    "airTemperatureC", "relativeHumidity", "durationSeconds",
    "holdTemperatureC", "minDurationSeconds", "maxDurationSeconds",
)
PRESET_FIELDS = (
    "id", "name", "temperatureC", "holdTemperatureC", "relativeHumidity",
    "drynessSlopeGm3PerHour", "minDurationSeconds", "maxDurationSeconds",
)


class TelemetryError(Exception):
    """A user-facing capture or input error."""


class AuthenticationError(TelemetryError):
    pass


class NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # Keep optional credentials on the explicitly selected device.
        return None


class Client:
    def __init__(self, base_url: str, username: str | None = None,
                 password: str | None = None):
        url = parse.urlsplit(base_url)
        if (url.scheme not in ("http", "https") or not url.hostname
                or url.username is not None or url.password is not None
                or url.query or url.fragment or url.path not in ("", "/")):
            raise TelemetryError("--base-url must be an HTTP(S) origin without credentials or a path")
        self.base_url = base_url.rstrip("/")
        self.opener = request.build_opener(NoRedirect())
        self.headers = {"Accept": "application/json"}
        if username is not None:
            token = base64.b64encode(f"{username}:{password or ''}".encode("utf-8"))
            self.headers["Authorization"] = "Basic " + token.decode("ascii")

    def get(self, endpoint: str, timeout: float) -> dict:
        req = request.Request(self.base_url + endpoint, headers=self.headers, method="GET")
        try:
            with self.opener.open(req, timeout=timeout) as response:
                data = json.load(response)
        except error.HTTPError as exc:
            if exc.code in (401, 403):
                raise AuthenticationError(
                    f"HTTP {exc.code}: access denied; use --username and check the device password"
                ) from None
            raise TelemetryError(f"HTTP {exc.code}") from None
        except (error.URLError, HTTPException, OSError, ValueError):
            raise TelemetryError("request failed or returned invalid JSON") from None
        if not isinstance(data, dict):
            raise TelemetryError("response must be a JSON object")
        return data


def select_fields(data: dict, fields: tuple[str, ...]) -> dict:
    # API additions never accidentally export network settings or credentials.
    return {key: data[key] for key in fields
            if key in data and isinstance(data[key], (str, int, float, bool, type(None)))
            and (not isinstance(data[key], float) or math.isfinite(data[key]))}


def control_config(config: dict) -> dict:
    result = select_fields(config, CONTROL_FIELDS)
    presets = config.get("presets", [])
    if isinstance(presets, list):
        result["presets"] = [select_fields(preset, PRESET_FIELDS)
                             for preset in presets if isinstance(preset, dict)]
    return result


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def save_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def capture(client: Client, output: Path, duration: float, interval: float,
            timeout: float = 10.0) -> dict:
    if not all(math.isfinite(value) and value > 0 for value in (duration, interval, timeout)):
        raise TelemetryError("duration, interval and timeout must be positive finite numbers")
    if any((output / name).exists() for name in ("telemetry.jsonl", "config.json", "metadata.json")):
        raise TelemetryError("output already contains capture files; choose another directory")
    output.mkdir(parents=True, exist_ok=True)
    metadata = {"startedAt": utc_now(), "durationSeconds": duration,
                "intervalSeconds": interval, "firmwareVersions": [],
                "samples": 0, "errors": 0, "status": "capturing"}
    save_json(output / "metadata.json", metadata)
    deadline = time.monotonic() + duration
    next_poll = time.monotonic()
    got_config = False
    try:
        with (output / "telemetry.jsonl").open("x", encoding="utf-8") as stream:
            while time.monotonic() < deadline:
                for endpoint in (("/api/state", "/api/config") if not got_config else ("/api/state",)):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        state = client.get(endpoint, min(timeout, remaining))
                        if endpoint == "/api/config":
                            save_json(output / "config.json", control_config(state))
                            got_config = True
                            continue
                        stream.write(json.dumps({"ts": utc_now(), "state": state},
                                                ensure_ascii=False, allow_nan=False) + "\n")
                        stream.flush()
                        metadata["samples"] += 1
                        version = state.get("version")
                        if isinstance(version, str) and version not in metadata["firmwareVersions"]:
                            metadata["firmwareVersions"].append(version)
                        if "initialRun" not in metadata:
                            initial = select_fields(state, ("mode", "phase", "runLabel"))
                            if isinstance(state.get("setpoints"), dict):
                                initial["setpoints"] = select_fields(state["setpoints"], SETPOINT_FIELDS)
                            if isinstance(state.get("dryness"), dict):
                                threshold = number(state["dryness"].get("slopeThreshold"))
                                if math.isfinite(threshold):
                                    initial["slopeThresholdGm3PerHour"] = threshold
                            metadata["initialRun"] = initial
                    except AuthenticationError:
                        metadata["status"] = "access_denied"
                        raise
                    except (TelemetryError, ValueError) as exc:
                        metadata["errors"] += 1
                        print(f"{utc_now()} {endpoint}: {exc}; retrying", file=sys.stderr)
                next_poll += interval
                now = time.monotonic()
                if next_poll <= now:
                    next_poll = now + interval
                time.sleep(max(0.0, min(next_poll, deadline) - now))
        metadata["status"] = "complete" if metadata["samples"] else "no_samples"
    except KeyboardInterrupt:
        metadata["status"] = "interrupted"
    finally:
        metadata["finishedAt"] = utc_now()
        metadata["configCaptured"] = got_config
        save_json(output / "metadata.json", metadata)
    if not metadata["samples"]:
        raise TelemetryError("no state samples captured; partial files were saved")
    return metadata


def load_records(path: Path) -> list[dict]:
    records = []
    with path.open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                if not isinstance(record, dict) or not isinstance(record.get("state"), dict):
                    raise ValueError()
                stamp = datetime.fromisoformat(record["ts"].replace("Z", "+00:00"))
                if stamp.tzinfo is None:
                    raise ValueError()
                record["stamp"] = stamp.timestamp()
            except (ValueError, TypeError, KeyError, AttributeError):
                raise TelemetryError(f"line {line_number}: expected {{ts: ISO timestamp with timezone, state: object}}") from None
            records.append(record)
    if not records:
        raise TelemetryError("telemetry file contains no samples")
    return records


def number(value) -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return float(value)
    return math.nan


def prepare_plot(records: list[dict]) -> dict:
    if not records:
        raise TelemetryError("telemetry file contains no samples")
    differences = [right["stamp"] - left["stamp"] for left, right in zip(records, records[1:])]
    if any(delta <= 0 for delta in differences):
        raise TelemetryError("sample timestamps must increase; split recordings with clock changes")
    # A lower-half median tolerates occasional missing polls without drawing across them.
    cadence = statistics.median(sorted(differences)[:max(1, len(differences) // 2)]) if differences else 5.0
    result = {"minutes": [], "cadenceSeconds": cadence, "spans": [],
              "series": {name: [] for name in ("air", "heater", "heaterTarget", "ah", "slope", "threshold")}}
    start = records[0]["stamp"]
    previous = None
    for index, record in enumerate(records):
        state = record["state"]
        minute = (record["stamp"] - start) / 60.0
        disconnected = False
        if previous is not None:
            uptime = number(state.get("uptimeMs"))
            last_uptime = number(previous["state"].get("uptimeMs"))
            if record["stamp"] - previous["stamp"] > 1.5 * cadence or uptime < last_uptime:
                disconnected = True
                result["minutes"].append(minute)
                for series in result["series"].values():
                    series.append(math.nan)
        result["minutes"].append(minute)
        air = state.get("air") if isinstance(state.get("air"), dict) else {}
        heater = state.get("heater") if isinstance(state.get("heater"), dict) else {}
        outputs = state.get("outputs") if isinstance(state.get("outputs"), dict) else {}
        dryness = state.get("dryness") if isinstance(state.get("dryness"), dict) else {}
        air_valid = air.get("valid") is True
        slope_valid = (air_valid and dryness.get("purgePhase") == 0
                       and number(dryness.get("ahSlopeSamples")) >= 8)
        values = {
            "air": number(air.get("temperatureC")) if air_valid else math.nan,
            "heater": number(heater.get("temperatureC")) if heater.get("valid") is True else math.nan,
            "heaterTarget": number(outputs.get("heaterSetpointC")),
            "ah": number(air.get("absoluteHumidityGm3")) if air_valid else math.nan,
            "slope": number(dryness.get("ahSlopeGm3PerHour")) if slope_valid else math.nan,
            "threshold": number(dryness.get("slopeThreshold")),
        }
        if values["heaterTarget"] <= 0:
            values["heaterTarget"] = math.nan
        if values["threshold"] <= 0:
            values["threshold"] = math.nan
        for name, value in values.items():
            result["series"][name].append(value)
        # Join adjacent equal states so long recordings stay compact in SVG.
        # Stop spans at missing polls and never merge across a device restart.
        right = minute + cadence / 60.0
        if index + 1 < len(records):
            next_stamp = records[index + 1]["stamp"]
            if next_stamp - record["stamp"] <= 1.5 * cadence:
                right = (next_stamp - start) / 60.0
        span = (minute, right, dryness.get("purgePhase"),
                "hold" if state.get("phase") == "hold" else "")
        spans = result["spans"]
        if (spans and not disconnected and spans[-1][2:] == span[2:]
                and math.isclose(spans[-1][1], minute, abs_tol=1e-8)):
            spans[-1] = (spans[-1][0], right, *span[2:])
        else:
            spans.append(span)
        previous = record
    if not any(math.isfinite(value) for key in ("air", "heater", "ah")
               for value in result["series"][key]):
        raise TelemetryError("recording contains no valid air or heater measurements")
    return result


def plot(input_path: Path, output: Path) -> None:
    if output.suffix.lower() not in (".png", ".svg"):
        raise TelemetryError("plot output must end in .png or .svg")
    data = prepare_plot(load_records(input_path))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except ImportError:
        raise TelemetryError("plotting needs matplotlib: python -m pip install -r tools/requirements.txt") from None
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, layout="constrained")
    try:
        series = data["series"]
        x = data["minutes"]
        for key, label in (("air", "Chamber air"), ("heater", "Heater NTC"),
                           ("heaterTarget", "Heater setpoint")):
            axes[0].plot(x, series[key], label=label, linewidth=1.2)
        axes[1].plot(x, series["ah"], label="Absolute humidity", color="#226b65")
        axes[2].plot(x, series["slope"], label="Sealed-window slope (n ≥ 8)", color="#925125", marker=".", markersize=2)
        axes[2].plot(x, series["threshold"], label="Dryness threshold", linestyle="--", color="#555555")
        shades = {0: ("#aaaaaa", "Sealed (0)"), 1: ("#e5b64f", "Settle (1)"), 2: ("#5599cc", "Purge (2)")}
        for left, right, phase, drying_phase in data["spans"]:
            for axis in axes:
                if phase in shades:
                    axis.axvspan(left, right, color=shades[phase][0], alpha=0.10, linewidth=0)
                if drying_phase == "hold":
                    axis.axvspan(left, right, facecolor="none", edgecolor="#38804c", hatch="///", linewidth=0)
        axes[0].set_ylabel("Temperature (°C)")
        axes[1].set_ylabel("AH (g/m³)")
        axes[2].set_ylabel("AH slope (g/m³/h)")
        axes[2].set_xlabel("Elapsed time (minutes)")
        for axis in axes:
            axis.grid(alpha=0.2)
            axis.legend(loc="upper left", fontsize="small")
        handles = [Patch(facecolor=color, alpha=0.3, label=label) for color, label in shades.values()]
        handles.append(Patch(facecolor="none", edgecolor="#38804c", hatch="///", label="Hold"))
        fig.legend(handles=handles, loc="outside lower center", ncol=4, fontsize="small")
        fig.suptitle("FilamentDryer — recorded drying cycle")
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=160)
    finally:
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collector = commands.add_parser("capture", help="capture full /api/state using GET requests only")
    collector.add_argument("--base-url", default="http://dryer.local")
    collector.add_argument("--duration", type=float, default=3600, help="capture duration in seconds")
    collector.add_argument("--interval", type=float, default=5, help="polling interval in seconds")
    collector.add_argument("--timeout", type=float, default=10, help="per-request timeout in seconds")
    collector.add_argument("--output", type=Path, required=True, help="directory for a new recording")
    collector.add_argument("--username", help="HTTP Basic username; prompts for the password")
    renderer = commands.add_parser("plot", help="plot capture or remote.ps1 watch JSONL")
    renderer.add_argument("input", type=Path)
    renderer.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "capture":
            password = getpass.getpass("Device password: ") if args.username is not None else None
            metadata = capture(Client(args.base_url, args.username, password), args.output,
                               args.duration, args.interval, args.timeout)
            print(f"Saved {metadata['samples']} samples to {args.output / 'telemetry.jsonl'} ({metadata['status']})")
            return 130 if metadata["status"] == "interrupted" else 0
        plot(args.input, args.output)
        print(f"Saved plot to {args.output}")
        return 0
    except (TelemetryError, OSError) as exc:
        print(f"telemetry: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
