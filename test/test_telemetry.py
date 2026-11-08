"""Synthetic fixtures below exercise tooling only; they are not device measurements."""

import base64
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import telemetry


def synthetic_state(**overrides):
    state = {"version": "synthetic-test-only", "mode": "timed_preset", "phase": "drying",
             "runLabel": "TEST", "uptimeMs": 10000,
             "air": {"valid": True, "temperatureC": 45.0, "absoluteHumidityGm3": 4.0},
             "heater": {"valid": True, "temperatureC": 60.0},
             "outputs": {"heaterSetpointC": 62.0},
             "dryness": {"ahSlopeSamples": 8, "ahSlopeGm3PerHour": 0.2,
                         "purgePhase": 0, "slopeThreshold": 0.3, "stableWindows": 1},
             "setpoints": {"airTemperatureC": 45.0, "holdTemperatureC": 35.0}}
    state.update(overrides)
    return state


def synthetic_records(seconds=(0, 5, 10)):
    start = datetime(2026, 1, 1, tzinfo=timezone.utc)
    return [{"ts": (start + timedelta(seconds=second)).isoformat(),
             "stamp": start.timestamp() + second,
             "state": synthetic_state(uptimeMs=10000 + second * 1000)} for second in seconds]


@contextmanager
def device(*, authenticate=False, failures=0, truncate_state=None):
    calls = []
    expected = "Basic " + base64.b64encode(b"tester:test-password").decode("ascii")
    remaining = failures
    state_requests = 0

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            nonlocal remaining, state_requests
            calls.append((self.command, self.path))
            if authenticate and self.headers.get("Authorization") != expected:
                self.send_response(401)
                self.end_headers()
                return
            if self.path == "/api/state":
                state_requests += 1
                if state_requests == truncate_state:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", "100")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    self.wfile.write(b'{"air":')
                    self.close_connection = True
                    return
            if self.path == "/api/state" and remaining:
                remaining -= 1
                self.send_response(503)
                self.end_headers()
                return
            if self.path == "/api/config":
                body = {"heaterMaxTemperatureC": 115.0, "wifiSsid": "private-network",
                        "wifiPassword": "never-save-this", "webLogin": "tester",
                        "hasWebPassword": True, "futureSecret": "nope",
                        "presets": [{"id": "PLA", "temperatureC": 45.0,
                                     "wifiPassword": "nested-secret"}]}
            else:
                body = synthetic_state()
            encoded = json.dumps(body).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=0.01), daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", calls
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


class CollectorTests(unittest.TestCase):
    def test_authenticated_capture_preserves_state_and_filters_config(self):
        with device(authenticate=True) as (url, calls), tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "recording"
            metadata = telemetry.capture(telemetry.Client(url, "tester", "test-password"),
                                         output, duration=0.40, interval=0.02)
            rows = telemetry.load_records(output / "telemetry.jsonl")
            self.assertGreaterEqual(len(rows), 2)
            self.assertEqual(rows[0]["state"], synthetic_state())
            config = json.loads((output / "config.json").read_text())
            self.assertEqual(config, {"heaterMaxTemperatureC": 115.0,
                                      "presets": [{"id": "PLA", "temperatureC": 45.0}]})
            for path in output.iterdir():
                text = path.read_text()
                for private in ("test-password", "private-network", "never-save-this", "nested-secret", "futureSecret", "webLogin"):
                    self.assertNotIn(private, text)
            self.assertEqual(metadata["firmwareVersions"], ["synthetic-test-only"])
            self.assertEqual(metadata["initialRun"]["setpoints"]["holdTemperatureC"], 35.0)
            self.assertEqual({method for method, path in calls}, {"GET"})
            self.assertEqual(sum(path == "/api/config" for _, path in calls), 1)

    def test_transient_errors_keep_partial_recording_and_retry(self):
        with device(failures=1) as (url, calls), tempfile.TemporaryDirectory() as tmp:
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                metadata = telemetry.capture(telemetry.Client(url), Path(tmp), 0.10, 0.02)
            self.assertEqual(metadata["errors"], 1)
            self.assertGreater(metadata["samples"], 0)
            self.assertIn("HTTP 503", stderr.getvalue())
            self.assertEqual(metadata["status"], "complete")

    def test_truncated_http_body_after_a_good_sample_retries_and_recovers(self):
        with device(truncate_state=2) as (url, calls), tempfile.TemporaryDirectory() as tmp:
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                metadata = telemetry.capture(telemetry.Client(url), Path(tmp), 0.40, 0.02)
            saved = json.loads((Path(tmp) / "metadata.json").read_text())
            rows = telemetry.load_records(Path(tmp) / "telemetry.jsonl")
            self.assertGreaterEqual(len(rows), 2)
            self.assertEqual(metadata["errors"], 1)
            self.assertEqual(saved["status"], "complete")
            self.assertEqual(saved["samples"], len(rows))
            self.assertGreaterEqual(sum(path == "/api/state" for _, path in calls), 3)
            self.assertIn("request failed", stderr.getvalue())
            self.assertNotIn("Traceback", stderr.getvalue())

    def test_access_denied_stops_immediately_and_saves_metadata(self):
        with device(authenticate=True) as (url, calls), tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(telemetry.AuthenticationError, "access denied"):
                telemetry.capture(telemetry.Client(url), Path(tmp), 1, 0.1)
            metadata = json.loads((Path(tmp) / "metadata.json").read_text())
            self.assertEqual(metadata["status"], "access_denied")
            self.assertEqual(len(calls), 1)

    def test_interrupt_keeps_completed_samples(self):
        client = mock.Mock()
        client.get.side_effect = [synthetic_state(), {}, KeyboardInterrupt()]
        with tempfile.TemporaryDirectory() as tmp:
            metadata = telemetry.capture(client, Path(tmp), 1, 0.001)
            self.assertEqual(metadata["status"], "interrupted")
            self.assertEqual(metadata["samples"], 1)
            self.assertEqual(len(telemetry.load_records(Path(tmp) / "telemetry.jsonl")), 1)

    def test_existing_capture_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "telemetry.jsonl"
            path.write_text("existing-data")
            with self.assertRaisesRegex(telemetry.TelemetryError, "already contains"):
                telemetry.capture(mock.Mock(), Path(tmp), 1, 1)
            self.assertEqual(path.read_text(), "existing-data")

    def test_credentials_are_prompted_and_not_cli_arguments(self):
        with mock.patch.object(telemetry.getpass, "getpass", return_value="test-password") as prompt, \
                mock.patch.object(telemetry, "capture", return_value={"samples": 1, "status": "complete"}) as capture, \
                redirect_stdout(io.StringIO()):
            result = telemetry.main(["capture", "--username", "tester", "--output", "unused"])
            self.assertEqual(result, 0)
            prompt.assert_called_once()
            self.assertNotIn("test-password", str(capture.call_args))

    def test_url_and_numeric_validation(self):
        for url in ("http://user:pass@dryer.local", "http://dryer.local/api", "file:///tmp/x"):
            with self.assertRaises(telemetry.TelemetryError):
                telemetry.Client(url)
        with tempfile.TemporaryDirectory() as tmp:
            for value in (0, -1, math.nan, math.inf):
                with self.assertRaises(telemetry.TelemetryError):
                    telemetry.capture(mock.Mock(), Path(tmp), value, 5)


class PlotTests(unittest.TestCase):
    def test_validity_and_window_phase_mask_slope(self):
        rows = synthetic_records((0, 5, 10, 15, 20))
        rows[1]["state"]["air"]["valid"] = False
        rows[2]["state"]["heater"]["valid"] = False
        rows[2]["state"]["dryness"]["ahSlopeSamples"] = 7
        rows[3]["state"]["dryness"]["purgePhase"] = 1
        rows[4]["state"]["dryness"]["purgePhase"] = 2
        series = telemetry.prepare_plot(rows)["series"]
        self.assertAlmostEqual(series["slope"][0], 0.2)
        self.assertTrue(all(math.isnan(value) for value in series["slope"][1:]))
        self.assertTrue(math.isnan(series["air"][1]))
        self.assertTrue(math.isnan(series["ah"][1]))
        self.assertTrue(math.isnan(series["heater"][2]))

    def test_missing_polls_and_device_restart_break_lines(self):
        rows = synthetic_records((0, 5, 10, 20, 25))
        rows[-1]["state"]["uptimeMs"] = 100
        data = telemetry.prepare_plot(rows)
        self.assertEqual(sum(math.isnan(value) for value in data["series"]["air"]), 2)
        self.assertEqual(len(data["minutes"]), len(rows) + 2)
        self.assertEqual(data["cadenceSeconds"], 5)

    def test_hold_and_numeric_ventilation_are_preserved(self):
        rows = synthetic_records()
        rows[0]["state"]["dryness"]["purgePhase"] = 0
        rows[1]["state"]["dryness"]["purgePhase"] = 1
        rows[2]["state"]["dryness"]["purgePhase"] = 2
        rows[2]["state"]["phase"] = "hold"
        spans = telemetry.prepare_plot(rows)["spans"]
        self.assertEqual([span[2] for span in spans], [0, 1, 2])
        self.assertEqual(spans[-1][3], "hold")

    def test_equal_spans_merge_but_gaps_restarts_and_hold_split_them(self):
        rows = synthetic_records((0, 5, 10, 20, 25, 30, 35))
        rows[4]["state"]["uptimeMs"] = 100
        rows[5]["state"]["uptimeMs"] = 5100
        rows[6]["state"]["uptimeMs"] = 10100
        rows[6]["state"]["phase"] = "hold"
        spans = telemetry.prepare_plot(rows)["spans"]
        self.assertEqual(len(spans), 4)
        self.assertEqual([(round(left * 60), round(right * 60)) for left, right, _, _ in spans],
                         [(0, 15), (20, 25), (25, 35), (35, 40)])

    def test_invalid_and_empty_input_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "input.jsonl"
            for text, message in (("", "no samples"), ("invalid\n", "line 1"),
                                   ('{"ts":"2026-01-01T00:00:00","state":{}}', "timezone")):
                path.write_text(text)
                with self.assertRaisesRegex(telemetry.TelemetryError, message):
                    telemetry.load_records(path)
        with self.assertRaisesRegex(telemetry.TelemetryError, "no valid"):
            telemetry.prepare_plot([{"stamp": 0, "state": {}}])

    def test_png_svg_render_and_powershell_bom_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "synthetic-test-only.jsonl"
            rows = synthetic_records()
            rows[-1]["state"]["phase"] = "hold"
            source.write_text("\ufeff" + "\n".join(json.dumps({"ts": row["ts"], "state": row["state"]})
                                                  for row in rows), encoding="utf-8")
            for extension in ("png", "svg"):
                output = Path(tmp) / f"synthetic-test-only.{extension}"
                telemetry.plot(source, output)
                self.assertGreater(output.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
