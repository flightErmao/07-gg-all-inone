from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from imu_usb_tool import (
    DEFAULT_BAUDRATE,
    parse_noise_prepare_response,
    parse_noise_status_response,
    wait_for_any_target_port,
)
from serial_client import DeviceClient


def _log(message: str) -> None:
    stamp = time.strftime("%H:%M:%S")
    print(f"[{stamp}] {message}", flush=True)


def _send_with_client(client: DeviceClient, command: str, timeout: float) -> str:
    return client.send_command(command, timeout=timeout)


def _query_noise_status_with_retry(client: DeviceClient, port_device: str, timeouts: tuple[float, ...]):
    last_error: Exception | None = None

    for attempt, timeout in enumerate(timeouts):
        try:
            if attempt > 0:
                client.open(port_device, DEFAULT_BAUDRATE)
            response = _send_with_client(client, "noise_test_status", timeout=timeout)
            return parse_noise_status_response(response)
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            _log(f"status query attempt {attempt + 1} failed: {exc}")
            time.sleep(0.2)

    if last_error is not None:
        raise last_error
    raise RuntimeError("noise_test_status query failed")


def _completed_normally(status, elapsed: int, duration_s: int, update_interval: int) -> bool:
    if status.last_error != 0:
        return False
    if status.last_run_ok == 1:
        return True
    return (duration_s - elapsed) <= max(1, update_interval)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run and monitor long noise test")
    parser.add_argument("--minutes", type=int, default=40)
    parser.add_argument("--seconds", type=int, default=0)
    parser.add_argument("--poll-interval", type=int, default=15)
    parser.add_argument("--query-during-run", action="store_true")
    args = parser.parse_args()

    duration_s = (args.minutes * 60) + args.seconds
    if duration_s <= 0:
        raise SystemExit("duration must be positive")

    port = wait_for_any_target_port(timeout=10.0)
    client = DeviceClient()
    client.open(port.device, DEFAULT_BAUDRATE)
    _log(f"connected port: {port.device}")

    try:
        prepare_response = _send_with_client(client, "noise_test_prepare", timeout=3.0)
        prepare = parse_noise_prepare_response(prepare_response)
        _log(f"prepare: detected={prepare.detected} duration_s={prepare.duration_s} status={prepare.status}")
        if prepare.detected <= 0 or prepare.status != "ok":
            _log("prepare failed")
            return 2

        start_response = _send_with_client(client, f"noise_test_start {args.minutes} {args.seconds}", timeout=3.0)
        _log(f"start: {start_response.strip()}")
        if "status=accepted" not in start_response:
            _log("start not accepted")
            return 3

        elapsed = 0
        while elapsed < duration_s:
            sleep_s = min(args.poll_interval, duration_s - elapsed)
            time.sleep(sleep_s)
            elapsed += sleep_s
            _log(f"progress checkpoint: {elapsed}/{duration_s}s")
            if not args.query_during_run:
                continue

            try:
                status = _query_noise_status_with_retry(client, port.device, (4.0, 8.0))
            except Exception as exc:  # noqa: BLE001
                _log(f"status query failed permanently at {elapsed}s: {exc}")
                return 10

            _log(
                "status: "
                f"recording={status.recording} frames={status.frames} "
                f"last_error={status.last_error} last_run_ok={status.last_run_ok}"
            )

            if status.last_error != 0:
                _log(f"device reported last_error={status.last_error}")
                return 11

            if status.recording == 0 and elapsed < duration_s:
                if _completed_normally(status, elapsed, duration_s, args.poll_interval):
                    _log("device already finished normally before final checkpoint")
                    break
                _log("device stopped early")
                return 12

        _log("collecting final status")
        status = _query_noise_status_with_retry(client, port.device, (6.0, 10.0, 12.0))
        _log(
            "final status: "
            f"recording={status.recording} frames={status.frames} "
            f"duration_s={status.duration_s} last_error={status.last_error} "
            f"last_run_ok={status.last_run_ok} file={status.file}"
        )
        if status.last_error != 0:
            return 13
        if status.last_run_ok != 1:
            _log("final status does not mark last_run_ok=1")
            return 14

        _log("40min noise test completed successfully")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
