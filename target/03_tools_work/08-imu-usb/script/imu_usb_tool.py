from __future__ import annotations

import argparse
import ctypes
import shutil
import string
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

from serial_client import DeviceClient, PortInfo


DEFAULT_VID = 0x0FFE
DEFAULT_PID_CDC = 0x0001
DEFAULT_PID_COMPOSITE = 0x0003
DEFAULT_LOG_FILE = "IMU_LOG.CSV"
DEFAULT_BAUDRATE = 115200
DEFAULT_WAIT_SECONDS = 25.0
JLINK_CANDIDATES = [
    Path(r"C:\Program Files\SEGGER\JLink_V844a\JLink.exe"),
    Path(r"C:\Program Files\SEGGER\JLink\JLink.exe"),
]


@dataclass(frozen=True)
class VolumeInfo:
    drive: str
    label: str
    filesystem: str


def list_ports() -> list[PortInfo]:
    return DeviceClient.list_port_infos()


def find_cdc_port(port_name: str | None) -> PortInfo:
    if port_name:
        for port in list_ports():
            if port.device.upper() == port_name.upper():
                return port
        raise RuntimeError(f"port not found: {port_name}")

    preferred = DeviceClient.find_preferred_port(DEFAULT_VID, (DEFAULT_PID_CDC, DEFAULT_PID_COMPOSITE))
    if preferred:
        return preferred

    ports = list_ports()
    if not ports:
        raise RuntimeError("no serial ports found")
    raise RuntimeError("target CDC port not found, please pass --port COMx")


def get_logical_drives() -> list[str]:
    bitmask = ctypes.windll.kernel32.GetLogicalDrives()
    drives: list[str] = []
    for index, letter in enumerate(string.ascii_uppercase):
        if bitmask & (1 << index):
            drives.append(f"{letter}:")
    return drives


def get_volume_info(drive: str) -> VolumeInfo | None:
    volume_name = ctypes.create_unicode_buffer(261)
    filesystem_name = ctypes.create_unicode_buffer(261)
    serial_number = ctypes.c_uint32()
    max_component_length = ctypes.c_uint32()
    filesystem_flags = ctypes.c_uint32()

    ok = ctypes.windll.kernel32.GetVolumeInformationW(
        ctypes.c_wchar_p(f"{drive}\\"),
        volume_name,
        len(volume_name),
        ctypes.byref(serial_number),
        ctypes.byref(max_component_length),
        ctypes.byref(filesystem_flags),
        filesystem_name,
        len(filesystem_name),
    )
    if not ok:
        return None

    return VolumeInfo(
        drive=drive,
        label=volume_name.value,
        filesystem=filesystem_name.value,
    )


def snapshot_volumes() -> dict[str, VolumeInfo]:
    volumes: dict[str, VolumeInfo] = {}
    for drive in get_logical_drives():
        info = get_volume_info(drive)
        if info is not None:
            volumes[drive.upper()] = info
    return volumes


def wait_for_port_presence(port_name: str, present: bool, timeout: float) -> bool:
    deadline = time.time() + timeout
    target = port_name.upper()
    while time.time() < deadline:
        current = {port.device.upper() for port in list_ports()}
        if (target in current) == present:
            return True
        time.sleep(0.3)
    return False


def wait_for_any_target_port(timeout: float) -> PortInfo:
    deadline = time.time() + timeout
    while time.time() < deadline:
        preferred = DeviceClient.find_preferred_port(
            DEFAULT_VID,
            (DEFAULT_PID_CDC, DEFAULT_PID_COMPOSITE),
        )
        if preferred is not None:
            return preferred
        time.sleep(0.3)
    raise TimeoutError("timed out waiting for target CDC port")


def wait_for_mode_transition(
    before: dict[str, VolumeInfo],
    wait: float,
    expected_drive_file: str | None = None,
    expect_cdc: bool = True,
) -> tuple[PortInfo | None, str | None]:
    cdc_port: PortInfo | None = None
    drive: str | None = None
    deadline = time.time() + wait

    while time.time() < deadline:
        if expect_cdc and cdc_port is None:
            cdc_port = DeviceClient.find_preferred_port(
                DEFAULT_VID,
                (DEFAULT_PID_CDC, DEFAULT_PID_COMPOSITE),
            )

        if drive is None:
            try:
                drive = find_new_drive(before, timeout=0.5, expected_file=expected_drive_file)
            except TimeoutError:
                drive = None

        if ((expect_cdc is False) or (cdc_port is not None)) and (
            (expected_drive_file is None) or (drive is not None)
        ):
            return cdc_port, drive

        time.sleep(0.2)

    raise TimeoutError("timed out waiting for target mode transition")


def find_new_drive(
    before: dict[str, VolumeInfo],
    timeout: float,
    expected_file: str | None = None,
) -> str:
    deadline = time.time() + timeout
    expected_upper = expected_file.upper() if expected_file else None

    while time.time() < deadline:
        current = snapshot_volumes()
        new_drives = sorted(set(current) - set(before))
        for drive in new_drives:
            if expected_upper is None:
                return drive
            candidate = Path(f"{drive}\\{expected_file}")
            if candidate.exists():
                return drive

        for drive in sorted(current):
            if expected_upper is None:
                continue
            candidate = Path(f"{drive}\\{expected_file}")
            if drive not in before and candidate.exists():
                return drive

        time.sleep(0.4)

    raise TimeoutError("timed out waiting for MSC drive")


def send_cdc_command(
    port_name: str | None,
    command: str,
    timeout: float,
    allow_disconnect: bool = False,
) -> tuple[PortInfo, str]:
    port = find_cdc_port(port_name)
    client = DeviceClient()
    try:
        client.open(port.device, DEFAULT_BAUDRATE)
        response = client.send_command(command, timeout=timeout, allow_disconnect=allow_disconnect)
        return port, response
    finally:
        client.close()


def locate_jlink() -> Path:
    for candidate in JLINK_CANDIDATES:
        if candidate.exists():
            return candidate
    raise RuntimeError("JLink.exe not found, please install SEGGER J-Link")


def jlink_reset(device: str = "STM32H743VI", speed: int = 4000) -> None:
    script = "\n".join(
        [
            f"device {device}",
            "if SWD",
            f"speed {speed}",
            "si 1",
            "r",
            "g",
            "q",
        ]
    )
    with tempfile.NamedTemporaryFile("w", suffix=".jlink", delete=False, encoding="ascii") as handle:
        handle.write(script)
        command_file = Path(handle.name)

    try:
        subprocess.run(
            [str(locate_jlink()), "-CommandFile", str(command_file)],
            check=True,
        )
    finally:
        command_file.unlink(missing_ok=True)


def command_ports(_args: argparse.Namespace) -> int:
    ports = list_ports()
    if not ports:
        print("no serial ports found")
        return 1

    for port in ports:
        print(f"{port.device:>6}  {port.description}  VID={port.vid!s} PID={port.pid!s}")
    return 0


def command_status(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, "status", timeout=args.timeout)
    print(f"port: {port.device}")
    print(response)
    return 0


def command_enter_msc(args: argparse.Namespace) -> int:
    before = snapshot_volumes()
    port, response = send_cdc_command(args.port, "enter_msc", timeout=args.timeout, allow_disconnect=True)
    print(f"port: {port.device}")
    if response:
        print(response)

    composite_port, drive = wait_for_mode_transition(
        before,
        wait=args.wait,
        expected_drive_file=DEFAULT_LOG_FILE,
        expect_cdc=True,
    )
    if composite_port is not None:
        print(f"cdc still available: {composite_port.device}")
    if drive is not None:
        print(f"msc drive: {drive}")
    return 0


def command_export_log(args: argparse.Namespace) -> int:
    output_dir = Path(args.output).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    before = snapshot_volumes()
    port, response = send_cdc_command(args.port, "enter_msc", timeout=args.timeout, allow_disconnect=True)
    print(f"port: {port.device}")
    if response:
        print(response)

    composite_port, drive = wait_for_mode_transition(
        before,
        wait=args.wait,
        expected_drive_file=DEFAULT_LOG_FILE,
        expect_cdc=True,
    )
    if composite_port is not None:
        print(f"cdc still available: {composite_port.device}")

    if drive is None:
        raise TimeoutError("timed out waiting for MSC drive")
    source = Path(f"{drive}\\{DEFAULT_LOG_FILE}")
    if not source.exists():
        raise FileNotFoundError(f"log file not found: {source}")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    destination = output_dir / f"imu_log_{timestamp}.csv"
    shutil.copy2(source, destination)
    print(f"copied: {destination}")

    if args.reset_to_cdc:
        jlink_reset()
        wait_for_port_presence(port.device, present=True, timeout=args.wait)
        print(f"cdc restored: {port.device}")

    return 0


def command_enter_cdc(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, "enter_cdc", timeout=args.timeout, allow_disconnect=True)
    print(f"port: {port.device}")
    if response:
        print(response)
    try:
        restored = wait_for_any_target_port(timeout=args.wait)
        print(f"cdc port: {restored.device}")
    except TimeoutError:
        pass
    return 0


def command_fake_sd_test(args: argparse.Namespace) -> int:
    command = f"fake_imu_sd_test {args.lines} {1 if args.overwrite else 0}"
    port, response = send_cdc_command(args.port, command, timeout=args.timeout)
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_reset_to_cdc(args: argparse.Namespace) -> int:
    jlink_reset(device=args.device, speed=args.speed)
    if args.port:
        wait_for_port_presence(args.port, present=True, timeout=args.wait)
        print(f"cdc restored: {args.port}")
    else:
        print("device reset completed")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32H750 IMU USB helper")
    subparsers = parser.add_subparsers(dest="command", required=True)

    parser_ports = subparsers.add_parser("ports", help="list serial ports")
    parser_ports.set_defaults(func=command_ports)

    parser_status = subparsers.add_parser("status", help="query firmware usb status")
    parser_status.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_status.add_argument("--timeout", type=float, default=2.0)
    parser_status.set_defaults(func=command_status)

    parser_enter_msc = subparsers.add_parser("enter-msc", help="switch firmware to CDC+MSC export mode")
    parser_enter_msc.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_enter_msc.add_argument("--timeout", type=float, default=2.0)
    parser_enter_msc.add_argument("--wait", type=float, default=DEFAULT_WAIT_SECONDS)
    parser_enter_msc.set_defaults(func=command_enter_msc)

    parser_enter_cdc = subparsers.add_parser("enter-cdc", help="switch firmware back to CDC-only mode")
    parser_enter_cdc.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_enter_cdc.add_argument("--timeout", type=float, default=2.0)
    parser_enter_cdc.add_argument("--wait", type=float, default=DEFAULT_WAIT_SECONDS)
    parser_enter_cdc.set_defaults(func=command_enter_cdc)

    parser_fake_sd = subparsers.add_parser("fake-sd-test", help="write fake imu data into IMU_TEST.CSV on sd card")
    parser_fake_sd.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_fake_sd.add_argument("--lines", type=int, default=64, help="number of fake imu rows to write")
    parser_fake_sd.add_argument("--timeout", type=float, default=5.0)
    parser_fake_sd.add_argument(
        "--overwrite",
        action="store_true",
        help="overwrite IMU_TEST.CSV instead of appending",
    )
    parser_fake_sd.set_defaults(func=command_fake_sd_test)

    parser_export = subparsers.add_parser("export-log", help="switch to CDC+MSC export mode and copy IMU_LOG.CSV")
    parser_export.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_export.add_argument("--timeout", type=float, default=2.0)
    parser_export.add_argument("--wait", type=float, default=DEFAULT_WAIT_SECONDS)
    parser_export.add_argument(
        "--output",
        default=str(Path(__file__).resolve().parent / "downloads"),
        help="directory for exported csv",
    )
    parser_export.add_argument(
        "--reset-to-cdc",
        action="store_true",
        help="reset target via J-Link after export so CDC comes back",
    )
    parser_export.set_defaults(func=command_export_log)

    parser_reset = subparsers.add_parser("reset-to-cdc", help="reset target through J-Link")
    parser_reset.add_argument("--port", help="expected CDC port after reset, e.g. COM78")
    parser_reset.add_argument("--wait", type=float, default=DEFAULT_WAIT_SECONDS)
    parser_reset.add_argument("--device", default="STM32H743VI")
    parser_reset.add_argument("--speed", type=int, default=4000)
    parser_reset.set_defaults(func=command_reset_to_cdc)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except Exception as exc:  # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
