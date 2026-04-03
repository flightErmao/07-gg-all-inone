from __future__ import annotations

import argparse
import ctypes
import csv
import math
import re
import shutil
import string
import struct
import subprocess
import sys
import tempfile
import time
from array import array
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from math import sqrt
from pathlib import Path

from serial_client import DeviceClient, PortInfo


DEFAULT_VID = 0x0FFE
DEFAULT_PID_CDC = 0x0001
DEFAULT_PID_COMPOSITE = 0x0003
DEFAULT_LOG_FILE = "IMU_LOG.CSV"
DEFAULT_BAUDRATE = 115200
DEFAULT_WAIT_SECONDS = 25.0
SCRIPT_ROOT = Path(__file__).resolve().parent
TOOL_PROJECT_ROOT = SCRIPT_ROOT.parent
ANALYSIS_OUTPUT_DIRS = {
    "bias": TOOL_PROJECT_ROOT / "data" / "01_bias",
    "temp": TOOL_PROJECT_ROOT / "data" / "02_temp",
    "arw": TOOL_PROJECT_ROOT / "data" / "03_arw",
    "vibe": TOOL_PROJECT_ROOT / "data" / "04_vibration",
    "shock": TOOL_PROJECT_ROOT / "data" / "05_shock",
}
TEST_OPTIONS: list[tuple[str, str]] = [
    ("bias", "测试项目 1：零偏及零偏稳定性"),
    ("temp", "测试项目 2：温漂"),
    ("arw", "测试项目 3：角度随机游走 ARW / 噪声"),
    ("vibe", "测试项目 4：振动条件下输出连续性"),
    ("shock", "测试项目 5：冲击后数据恢复"),
]
REAL_IMU_FRAME_STRUCT = struct.Struct("<HBQHBHhhhhhhhH")
REAL_IMU_FRAME_MAGIC_HEAD = 0x55AA
REAL_IMU_RAW_RECORD_HEADER = struct.Struct("<HHHQIH")
REAL_IMU_RAW_MAGIC_HEAD = 0x55AB
ICM42688_ACCEL_SCALER_MPS2 = 16.0 / float(1 << 15) * 9.80665
ICM42688_GYRO_SCALER_RAD_S = 2000.0 / float(1 << 15) * math.pi / 180.0
ICM42688_TEMP_SCALER_LSB_PER_C = 2.07
ICM42688_TEMP_OFFSET_C = 25.0
ICM45686_ACCEL_SCALER_MPS2 = 16.0 / float(1 << 15) * 9.80665
ICM45686_GYRO_SCALER_RAD_S = 2000.0 / float(1 << 15) * math.pi / 180.0
ICM45686_TEMP_SCALER_LSB_PER_C = 2.0
ICM45686_TEMP_OFFSET_C = 25.0
IMU_ID_TO_NAME = {
    1: "42688_A",
    2: "42688_B",
    3: "45686_A",
    4: "45686_B",
}
IMU_CONFIGS = {
    1: {"odr_hz": 1000.0, "accel_range_g": 16.0, "gyro_range_dps": 2000.0},
    2: {"odr_hz": 1000.0, "accel_range_g": 16.0, "gyro_range_dps": 2000.0},
    3: {"odr_hz": 1600.0, "accel_range_g": 16.0, "gyro_range_dps": 2000.0},
    4: {"odr_hz": 1600.0, "accel_range_g": 16.0, "gyro_range_dps": 2000.0},
}
IMU_PAPER_METRICS = {
    1: {
        "accel_zero_bias_mg": 20.0,
        "gyro_zero_bias_dps": 0.5,
        "accel_bias_tc_mg_c": 0.15,
        "gyro_bias_tc_dps_c": 0.005,
        "gyro_arw_deg_sqrt_hr": 0.0028,   # deg/s/sqrt(Hz) noise density
        "accel_noise_ug_sqhz": 70.0,       # µg/sqrt(Hz) noise density
    },
    2: {
        "accel_zero_bias_mg": 20.0,
        "gyro_zero_bias_dps": 0.5,
        "accel_bias_tc_mg_c": 0.15,
        "gyro_bias_tc_dps_c": 0.005,
        "gyro_arw_deg_sqrt_hr": 0.0028,
        "accel_noise_ug_sqhz": 70.0,
    },
    3: {
        "accel_zero_bias_mg": 20.0,
        "gyro_zero_bias_dps": 0.4,
        "accel_bias_tc_mg_c": 0.15,
        "gyro_bias_tc_dps_c": 0.005,
        "gyro_arw_deg_sqrt_hr": 0.0038,
        "accel_noise_ug_sqhz": 70.0,
    },
    4: {
        "accel_zero_bias_mg": 20.0,
        "gyro_zero_bias_dps": 0.4,
        "accel_bias_tc_mg_c": 0.15,
        "gyro_bias_tc_dps_c": 0.005,
        "gyro_arw_deg_sqrt_hr": 0.0038,
        "accel_noise_ug_sqhz": 70.0,
    },
}
PAPER_TARGETS = {
    "gyro_arw_deg_sqrt_hr_max": 0.0050,
}
PROGRESS_REPORT_INTERVAL_S = 2.0
MAX_ANALYSIS_SERIES_POINTS = 120_000
ARW_REPORT_POLL_PERIOD_S = 0.002
JLINK_CANDIDATES = [
    Path(r"C:\Program Files\SEGGER\JLink_V844a\JLink.exe"),
    Path(r"C:\Program Files\SEGGER\JLink\JLink.exe"),
]


@dataclass(frozen=True)
class VolumeInfo:
    drive: str
    label: str
    filesystem: str


@dataclass(frozen=True)
class RealImuFrame:
    some_flag: int
    fifo_packet_count: int
    timestamp_us: int
    fifo_header: int
    fifo_timestamp: int
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    temp_raw: int
    same_header: int
    same_fifo_timestamp: int
    same_acc_x: int
    same_acc_y: int
    same_acc_z: int
    same_gyro_x: int
    same_gyro_y: int
    same_gyro_z: int


@dataclass(frozen=True)
class RawImuCapture:
    timestamp_us: int
    poll_count: int
    imu_id: int
    packet_size: int
    fifo_byte_count: int
    payload: bytes


@dataclass(frozen=True)
class ParsedRealImuBin:
    frames: list[RealImuFrame]
    raw_captures: list[RawImuCapture]


@dataclass
class ParseProgressReporter:
    total_bytes: int
    callback: Callable[[str], None] | None = None
    interval_s: float = PROGRESS_REPORT_INTERVAL_S
    stage: str = "解析"
    bytes_processed: int = 0
    frame_count: int = 0
    raw_capture_count: int = 0
    packet_count: int = 0
    started_at_s: float = 0.0
    last_report_at_s: float = 0.0

    def __post_init__(self) -> None:
        now = time.monotonic()
        self.started_at_s = now
        self.last_report_at_s = now

    def log(self, message: str) -> None:
        if self.callback is not None:
            self.callback(message)
            return
        print(message)

    def maybe_report(self, *, force: bool = False, extra: str = "") -> None:
        now = time.monotonic()
        if not force and (now - self.last_report_at_s) < self.interval_s:
            return

        processed_mb = self.bytes_processed / (1024.0 * 1024.0)
        total_mb = self.total_bytes / (1024.0 * 1024.0) if self.total_bytes > 0 else 0.0
        ratio = (self.bytes_processed / self.total_bytes * 100.0) if self.total_bytes > 0 else 0.0
        elapsed_s = max(now - self.started_at_s, 1e-6)
        speed_mb_s = processed_mb / elapsed_s
        extra_text = f"，{extra}" if extra else ""
        self.log(
            f"{self.stage}进度: {ratio:.1f}% ({processed_mb:.1f}/{total_mb:.1f} MB)"
            f"，raw记录 {self.raw_capture_count}，frame {self.frame_count}，包 {self.packet_count}"
            f"，速度 {speed_mb_s:.1f} MB/s{extra_text}"
        )
        self.last_report_at_s = now


@dataclass
class PacketRowBuilderState:
    wrap_count_by_imu: dict[int, int]
    last_timestamp_by_imu: dict[int, int]
    global_packet_index: int = 0
    skipped_packets_by_imu: dict[int, int] = field(default_factory=dict)


@dataclass
class DecimatedImuSeries:
    max_points: int = MAX_ANALYSIS_SERIES_POINTS
    raw_index: int = 0
    decimation_factor: int = 1
    timestamps_us: array = None  # type: ignore[assignment]
    gyro_x_deg_s: array = None  # type: ignore[assignment]
    gyro_y_deg_s: array = None  # type: ignore[assignment]
    gyro_z_deg_s: array = None  # type: ignore[assignment]
    acc_x_mps2: array = None  # type: ignore[assignment]
    acc_y_mps2: array = None  # type: ignore[assignment]
    acc_z_mps2: array = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        self.timestamps_us = array("Q")
        self.gyro_x_deg_s = array("d")
        self.gyro_y_deg_s = array("d")
        self.gyro_z_deg_s = array("d")
        self.acc_x_mps2 = array("d")
        self.acc_y_mps2 = array("d")
        self.acc_z_mps2 = array("d")

    def add(
        self,
        timestamp_us: int,
        gyro_x_deg_s: float,
        gyro_y_deg_s: float,
        gyro_z_deg_s: float,
        acc_x_mps2: float,
        acc_y_mps2: float,
        acc_z_mps2: float,
    ) -> None:
        if self.raw_index % self.decimation_factor == 0:
            self.timestamps_us.append(int(timestamp_us))
            self.gyro_x_deg_s.append(float(gyro_x_deg_s))
            self.gyro_y_deg_s.append(float(gyro_y_deg_s))
            self.gyro_z_deg_s.append(float(gyro_z_deg_s))
            self.acc_x_mps2.append(float(acc_x_mps2))
            self.acc_y_mps2.append(float(acc_y_mps2))
            self.acc_z_mps2.append(float(acc_z_mps2))
            if len(self.timestamps_us) > self.max_points:
                self._downsample()
        self.raw_index += 1

    def _downsample(self) -> None:
        self.timestamps_us = array("Q", self.timestamps_us[::2])
        self.gyro_x_deg_s = array("d", self.gyro_x_deg_s[::2])
        self.gyro_y_deg_s = array("d", self.gyro_y_deg_s[::2])
        self.gyro_z_deg_s = array("d", self.gyro_z_deg_s[::2])
        self.acc_x_mps2 = array("d", self.acc_x_mps2[::2])
        self.acc_y_mps2 = array("d", self.acc_y_mps2[::2])
        self.acc_z_mps2 = array("d", self.acc_z_mps2[::2])
        self.decimation_factor *= 2

def _imu_uses_little_endian_packets(imu_id: int) -> bool:
    return imu_id in (3, 4)


def _packet_all_byte_value(packet: bytes, value: int) -> bool:
    return len(packet) > 0 and all(byte == value for byte in packet)


def _should_skip_packet_for_analysis(packet: bytes, imu_id: int) -> bool:
    return imu_id in (3, 4) and _packet_all_byte_value(packet, 0x7F)


def _raw_accel_to_mps2(raw_value: int, imu_id: int) -> float:
    scaler = ICM45686_ACCEL_SCALER_MPS2 if imu_id in (3, 4) else ICM42688_ACCEL_SCALER_MPS2
    return float(raw_value) * scaler


def _raw_gyro_to_rad_s(raw_value: int, imu_id: int) -> float:
    scaler = ICM45686_GYRO_SCALER_RAD_S if imu_id in (3, 4) else ICM42688_GYRO_SCALER_RAD_S
    return float(raw_value) * scaler


def _raw_temp_to_celsius(raw_value: int, imu_id: int) -> float:
    if imu_id in (3, 4):
        return float(raw_value) / ICM45686_TEMP_SCALER_LSB_PER_C + ICM45686_TEMP_OFFSET_C
    return float(raw_value) / ICM42688_TEMP_SCALER_LSB_PER_C + ICM42688_TEMP_OFFSET_C


def _format_float(value: float) -> str:
    return f"{value:.6f}"


@dataclass(frozen=True)
class DeviceStatus:
    mode: str
    sd: str
    test: str


@dataclass(frozen=True)
class ImuProbeResult:
    imu: int
    name: str
    status: str


@dataclass(frozen=True)
class NoisePrepareResult:
    detected: int
    duration_s: int
    recording: int
    status: str


@dataclass(frozen=True)
class NoiseStatusResult:
    recording: int
    frames: int
    duration_s: int
    file: str
    flushes: int
    max_gap_ms: int
    last_error: int
    last_run_ok: int
    reason: str
    fault_imu: int
    fault_poll: int
    fault_packet: int
    status: str


_INT_PREFIX_RE = re.compile(r"[-+]?\d+")


def _sanitize_shell_response(response: str) -> str:
    return response.replace("\x00", "\n").replace("\r\n", "\n").replace("\r", "\n")


def _parse_response_parts(payload: str) -> dict[str, str]:
    parts: dict[str, str] = {}
    for token in payload.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parts[key] = value
    return parts


def _parse_int_field(parts: dict[str, str], key: str, default: int = 0) -> int:
    raw_value = parts.get(key)
    if raw_value is None:
        return default

    match = _INT_PREFIX_RE.search(raw_value)
    if match is None:
        return default

    return int(match.group(0))


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


def parse_status_response(response: str) -> DeviceStatus:
    for line in _sanitize_shell_response(response).splitlines():
        stripped = line.strip()
        if not stripped.startswith("STATUS "):
            continue

        parts = _parse_response_parts(stripped[7:])

        return DeviceStatus(
            mode=parts.get("mode", "unknown"),
            sd=parts.get("sd", "unknown"),
            test=parts.get("test", "unknown"),
        )

    raise ValueError("STATUS line not found")


def parse_imu_probe_response(response: str) -> list[ImuProbeResult]:
    results: list[ImuProbeResult] = []

    for line in _sanitize_shell_response(response).splitlines():
        stripped = line.strip()
        if not stripped.startswith("IMU_PROBE "):
            continue

        parts = _parse_response_parts(stripped[10:])

        imu_text = parts.get("imu")
        if imu_text is None:
            continue

        results.append(
            ImuProbeResult(
                imu=_parse_int_field(parts, "imu"),
                name=parts.get("name", "unknown"),
                status=parts.get("status", "unknown"),
            )
        )

    if not results:
        raise ValueError("IMU_PROBE line not found")

    return results


def parse_noise_prepare_response(response: str) -> NoisePrepareResult:
    status_text = "unknown"
    sanitized = _sanitize_shell_response(response)

    for line in sanitized.splitlines():
        stripped = line.strip()
        if stripped.startswith("RESULT cmd=noise_test_prepare "):
            for token in stripped.split():
                if token.startswith("status="):
                    status_text = token.split("=", 1)[1]

    for line in sanitized.splitlines():
        stripped = line.strip()
        if not stripped.startswith("NOISE_TEST "):
            continue

        parts = _parse_response_parts(stripped[11:])

        return NoisePrepareResult(
            detected=_parse_int_field(parts, "detected"),
            duration_s=_parse_int_field(parts, "duration_s"),
            recording=_parse_int_field(parts, "recording"),
            status=status_text,
        )

    raise ValueError("NOISE_TEST line not found")


def parse_noise_status_response(response: str) -> NoiseStatusResult:
    status_text = "unknown"
    sanitized = _sanitize_shell_response(response)

    for line in sanitized.splitlines():
        stripped = line.strip()
        if stripped.startswith("RESULT cmd=noise_test_status "):
            for token in stripped.split():
                if token.startswith("status="):
                    status_text = token.split("=", 1)[1]

    for line in sanitized.splitlines():
        stripped = line.strip()
        if not stripped.startswith("NOISE_TEST_STATUS "):
            continue

        parts = _parse_response_parts(stripped[18:])

        return NoiseStatusResult(
            recording=_parse_int_field(parts, "recording"),
            frames=_parse_int_field(parts, "frames"),
            duration_s=_parse_int_field(parts, "duration_s"),
            file=parts.get("file", ""),
            flushes=_parse_int_field(parts, "flushes", 0),
            max_gap_ms=_parse_int_field(parts, "max_gap_ms", 0),
            last_error=_parse_int_field(parts, "last_error", 0),
            last_run_ok=_parse_int_field(parts, "last_run_ok", 0),
            reason=parts.get("reason", ""),
            fault_imu=_parse_int_field(parts, "fault_imu", 0),
            fault_poll=_parse_int_field(parts, "fault_poll", 0),
            fault_packet=_parse_int_field(parts, "fault_packet", 0),
            status=status_text,
        )

    raise ValueError("NOISE_TEST_STATUS line not found")


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


def _crc16_ccitt(data: bytes, init: int = 0) -> int:
    crc = init & 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_real_imu_bin(payload: bytes) -> ParsedRealImuBin:
    frames: list[RealImuFrame] = []
    raw_captures: list[RawImuCapture] = []
    offset = 0
    frame_size = REAL_IMU_FRAME_STRUCT.size

    while offset < len(payload):
        if offset + 2 > len(payload):
            raise ValueError(f"truncated record head at offset {offset}")

        magic_head = struct.unpack_from("<H", payload, offset)[0]
        if magic_head == REAL_IMU_FRAME_MAGIC_HEAD:
            if offset + frame_size > len(payload):
                raise ValueError(f"truncated summary frame at offset {offset}")

            fields = REAL_IMU_FRAME_STRUCT.unpack_from(payload, offset)
            (
                _magic_head,
                some_flag,
                timestamp_us,
                fifo_packet_count,
                fifo_header,
                fifo_timestamp,
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
                temp_raw,
                crc16,
            ) = fields

            calc_crc = _crc16_ccitt(payload[offset + 2 : offset + frame_size - 2])
            if crc16 != calc_crc:
                raise ValueError(
                    f"invalid real frame crc at offset {offset}: 0x{crc16:04X} != 0x{calc_crc:04X}"
                )

            frames.append(
                RealImuFrame(
                    some_flag=some_flag,
                    fifo_packet_count=fifo_packet_count,
                    timestamp_us=timestamp_us,
                    fifo_header=fifo_header,
                    fifo_timestamp=fifo_timestamp,
                    accel_x=accel_x,
                    accel_y=accel_y,
                    accel_z=accel_z,
                    gyro_x=gyro_x,
                    gyro_y=gyro_y,
                    gyro_z=gyro_z,
                    temp_raw=temp_raw,
                    same_header=1 if (some_flag & (1 << 7)) else 0,
                    same_fifo_timestamp=1 if (some_flag & (1 << 6)) else 0,
                    same_acc_x=1 if (some_flag & (1 << 5)) else 0,
                    same_acc_y=1 if (some_flag & (1 << 4)) else 0,
                    same_acc_z=1 if (some_flag & (1 << 3)) else 0,
                    same_gyro_x=1 if (some_flag & (1 << 2)) else 0,
                    same_gyro_y=1 if (some_flag & (1 << 1)) else 0,
                    same_gyro_z=1 if (some_flag & (1 << 0)) else 0,
                )
            )
            offset += frame_size
            continue

        if magic_head == REAL_IMU_RAW_MAGIC_HEAD:
            if offset + REAL_IMU_RAW_RECORD_HEADER.size + 2 > len(payload):
                raise ValueError(f"truncated raw capture header at offset {offset}")

            (
                _magic_head,
                fifo_byte_count,
                packet_size,
                timestamp_us,
                poll_count,
                imu_id,
            ) = REAL_IMU_RAW_RECORD_HEADER.unpack_from(payload, offset)

            total_size = REAL_IMU_RAW_RECORD_HEADER.size + fifo_byte_count + 2
            if packet_size <= 0:
                raise ValueError(f"invalid raw capture packet size {packet_size} at offset {offset}")
            if offset + total_size > len(payload):
                raise ValueError(f"truncated raw capture payload at offset {offset}")

            crc16 = struct.unpack_from("<H", payload, offset + total_size - 2)[0]
            calc_crc = _crc16_ccitt(payload[offset + 2 : offset + total_size - 2])
            if crc16 != calc_crc:
                raise ValueError(
                    f"invalid raw capture crc at offset {offset}: 0x{crc16:04X} != 0x{calc_crc:04X}"
                )

            raw_captures.append(
                RawImuCapture(
                    timestamp_us=timestamp_us,
                    poll_count=poll_count,
                    imu_id=imu_id,
                    packet_size=packet_size,
                    fifo_byte_count=fifo_byte_count,
                    payload=payload[
                        offset + REAL_IMU_RAW_RECORD_HEADER.size : offset + REAL_IMU_RAW_RECORD_HEADER.size + fifo_byte_count
                    ],
                )
            )
            offset += total_size
            continue

        raise ValueError(f"unknown record head at offset {offset}: 0x{magic_head:04X}")

    return ParsedRealImuBin(frames=frames, raw_captures=raw_captures)


def _read_exact(handle, size: int, *, offset: int, context: str) -> bytes:
    data = handle.read(size)
    if len(data) != size:
        raise ValueError(f"truncated {context} at offset {offset}")
    return data


def iter_real_imu_bin_file_records(
    source: Path, progress: ParseProgressReporter | None = None
) -> Iterator[tuple[str, RealImuFrame | RawImuCapture]]:
    with source.open("rb") as handle:
        offset = 0
        while True:
            magic_bytes = handle.read(2)
            if not magic_bytes:
                break
            if len(magic_bytes) != 2:
                raise ValueError(f"truncated record head at offset {offset}")

            magic_head = struct.unpack("<H", magic_bytes)[0]
            if magic_head == REAL_IMU_FRAME_MAGIC_HEAD:
                rest = _read_exact(
                    handle,
                    REAL_IMU_FRAME_STRUCT.size - 2,
                    offset=offset + 2,
                    context="frame payload",
                )
                record = magic_bytes + rest
                (
                    _magic_head,
                    some_flag,
                    timestamp_us,
                    fifo_packet_count,
                    fifo_header,
                    fifo_timestamp,
                    accel_x,
                    accel_y,
                    accel_z,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    temp_raw,
                    same_flags,
                ) = REAL_IMU_FRAME_STRUCT.unpack(record)
                frame = RealImuFrame(
                    some_flag=some_flag,
                    fifo_packet_count=fifo_packet_count,
                    timestamp_us=timestamp_us,
                    fifo_header=fifo_header,
                    fifo_timestamp=fifo_timestamp,
                    accel_x=accel_x,
                    accel_y=accel_y,
                    accel_z=accel_z,
                    gyro_x=gyro_x,
                    gyro_y=gyro_y,
                    gyro_z=gyro_z,
                    temp_raw=temp_raw,
                    same_header=1 if (same_flags & (1 << 7)) else 0,
                    same_fifo_timestamp=1 if (same_flags & (1 << 6)) else 0,
                    same_acc_x=1 if (same_flags & (1 << 5)) else 0,
                    same_acc_y=1 if (same_flags & (1 << 4)) else 0,
                    same_acc_z=1 if (same_flags & (1 << 3)) else 0,
                    same_gyro_x=1 if (same_flags & (1 << 2)) else 0,
                    same_gyro_y=1 if (same_flags & (1 << 1)) else 0,
                    same_gyro_z=1 if (same_flags & (1 << 0)) else 0,
                )
                offset += REAL_IMU_FRAME_STRUCT.size
                if progress is not None:
                    progress.bytes_processed = offset
                    progress.frame_count += 1
                    progress.maybe_report()
                yield "frame", frame
                continue

            if magic_head == REAL_IMU_RAW_MAGIC_HEAD:
                header_rest = _read_exact(
                    handle,
                    REAL_IMU_RAW_RECORD_HEADER.size - 2,
                    offset=offset + 2,
                    context="raw capture header",
                )
                header = magic_bytes + header_rest
                (
                    _magic_head,
                    fifo_byte_count,
                    packet_size,
                    timestamp_us,
                    poll_count,
                    imu_id,
                ) = REAL_IMU_RAW_RECORD_HEADER.unpack(header)
                if packet_size <= 0:
                    raise ValueError(f"invalid raw capture packet size {packet_size} at offset {offset}")
                payload = _read_exact(
                    handle,
                    fifo_byte_count,
                    offset=offset + REAL_IMU_RAW_RECORD_HEADER.size,
                    context="raw capture payload",
                )
                crc_bytes = _read_exact(
                    handle,
                    2,
                    offset=offset + REAL_IMU_RAW_RECORD_HEADER.size + fifo_byte_count,
                    context="raw capture crc",
                )
                crc16 = struct.unpack("<H", crc_bytes)[0]
                calc_crc = _crc16_ccitt(header[2:] + payload)
                if crc16 != calc_crc:
                    raise ValueError(
                        f"invalid raw capture crc at offset {offset}: 0x{crc16:04X} != 0x{calc_crc:04X}"
                    )
                capture = RawImuCapture(
                    timestamp_us=timestamp_us,
                    poll_count=poll_count,
                    imu_id=imu_id,
                    packet_size=packet_size,
                    fifo_byte_count=fifo_byte_count,
                    payload=payload,
                )
                offset += REAL_IMU_RAW_RECORD_HEADER.size + fifo_byte_count + 2
                if progress is not None:
                    progress.bytes_processed = offset
                    progress.raw_capture_count += 1
                    progress.maybe_report()
                yield "raw_capture", capture
                continue

            raise ValueError(f"unknown record head at offset {offset}: 0x{magic_head:04X}")

    if progress is not None:
        progress.bytes_processed = progress.total_bytes
        progress.maybe_report(force=True, extra="文件扫描完成")


def iter_real_imu_frames(payload: bytes) -> list[RealImuFrame]:
    parsed = parse_real_imu_bin(payload)
    return parsed.frames if parsed.frames else _build_synthetic_frames_from_raw_captures(parsed.raw_captures)


def iter_real_imu_raw_captures(payload: bytes) -> list[RawImuCapture]:
    return parse_real_imu_bin(payload).raw_captures


def decode_real_imu_bin_to_csv(source: Path, destination: Path) -> int:
    payload = source.read_bytes()
    parsed = parse_real_imu_bin(payload)
    frames = parsed.frames if parsed.frames else _build_synthetic_frames_from_raw_captures(parsed.raw_captures)
    destination.parent.mkdir(parents=True, exist_ok=True)

    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "timestamp_us",
                "fifo_packet_count",
                "fifo_header",
                "fifo_timestamp",
                "raw_accel_x",
                "raw_accel_y",
                "raw_accel_z",
                "raw_gyro_x",
                "raw_gyro_y",
                "raw_gyro_z",
                "temp_raw",
                "accel_x_mps2",
                "accel_y_mps2",
                "accel_z_mps2",
                "gyro_x_rad_s",
                "gyro_y_rad_s",
                "gyro_z_rad_s",
                "temp_c",
                "same_flag",
                "same_header",
                "same_fifo_timestamp",
                "same_acc_x",
                "same_acc_y",
                "same_acc_z",
                "same_gyro_x",
                "same_gyro_y",
                "same_gyro_z",
            ]
        )
        for frame in frames:
            writer.writerow(
                [
                    frame.timestamp_us,
                    frame.fifo_packet_count,
                    frame.fifo_header,
                    frame.fifo_timestamp,
                    frame.accel_x,
                    frame.accel_y,
                    frame.accel_z,
                    frame.gyro_x,
                    frame.gyro_y,
                    frame.gyro_z,
                    frame.temp_raw,
                    _format_float(_raw_accel_to_mps2(frame.accel_x, 1)),
                    _format_float(_raw_accel_to_mps2(frame.accel_y, 1)),
                    _format_float(_raw_accel_to_mps2(frame.accel_z, 1)),
                    _format_float(_raw_gyro_to_rad_s(frame.gyro_x, 1)),
                    _format_float(_raw_gyro_to_rad_s(frame.gyro_y, 1)),
                    _format_float(_raw_gyro_to_rad_s(frame.gyro_z, 1)),
                    _format_float(_raw_temp_to_celsius(frame.temp_raw, 1)),
                    frame.some_flag,
                    frame.same_header,
                    frame.same_fifo_timestamp,
                    frame.same_acc_x,
                    frame.same_acc_y,
                    frame.same_acc_z,
                    frame.same_gyro_x,
                    frame.same_gyro_y,
                    frame.same_gyro_z,
                ]
            )

    return len(frames)


def _packet_timestamp_u16(packet: bytes, imu_id: int) -> int:
    if len(packet) < 16:
        return 0
    byteorder = "little" if _imu_uses_little_endian_packets(imu_id) else "big"
    return int.from_bytes(packet[14:16], byteorder=byteorder, signed=False)


def _packet_hex(packet: bytes) -> str:
    return packet.hex(" ").upper()


def _imu_name_from_id(imu_id: int) -> str:
    return IMU_ID_TO_NAME.get(imu_id, f"IMU{imu_id}")


def _imu_config(imu_id: int) -> dict[str, float]:
    return IMU_CONFIGS.get(imu_id, {"odr_hz": 0.0, "accel_range_g": 0.0, "gyro_range_dps": 0.0})


def _positive_steps(values: list[int]) -> list[int]:
    if len(values) < 2:
        return []
    return [values[index] - values[index - 1] for index in range(1, len(values)) if values[index] - values[index - 1] > 0]


def _packet_s16(packet: bytes, start: int, imu_id: int) -> int:
    if len(packet) < start + 2:
        return 0
    byteorder = "little" if _imu_uses_little_endian_packets(imu_id) else "big"
    return int.from_bytes(packet[start : start + 2], byteorder=byteorder, signed=True)


def _packet_s8(packet: bytes, start: int) -> int:
    if len(packet) < start + 1:
        return 0
    return int.from_bytes(packet[start : start + 1], byteorder="big", signed=True)


def _div_round_s32(total: int, count: int) -> int:
    if count <= 0:
        return 0
    if total >= 0:
        return (total + count // 2) // count
    return (total - count // 2) // count


def _build_raw_packet_rows(captures: list[RawImuCapture]) -> list[dict[str, int | float | str]]:
    rows: list[dict[str, int | float | str]] = []
    state = PacketRowBuilderState(wrap_count_by_imu={}, last_timestamp_by_imu={})

    for capture in captures:
        rows.extend(_iter_capture_packet_rows(capture, state))
    return rows


def _iter_capture_packet_rows(
    capture: RawImuCapture, state: PacketRowBuilderState
) -> Iterator[dict[str, int | float | str]]:
    if capture.packet_size <= 0:
        return

    packet_count = capture.fifo_byte_count // capture.packet_size
    for packet_index in range(packet_count):
        start = packet_index * capture.packet_size
        end = start + capture.packet_size
        packet = capture.payload[start:end]
        if len(packet) != capture.packet_size:
            continue
        if _should_skip_packet_for_analysis(packet, capture.imu_id):
            state.skipped_packets_by_imu[capture.imu_id] = state.skipped_packets_by_imu.get(capture.imu_id, 0) + 1
            continue

        state.global_packet_index += 1
        packet_timestamp = _packet_timestamp_u16(packet, capture.imu_id)
        wrap_count = state.wrap_count_by_imu.get(capture.imu_id, 0)
        last_timestamp = state.last_timestamp_by_imu.get(capture.imu_id)
        if last_timestamp is not None and packet_timestamp < last_timestamp:
            wrap_count += 1
        state.wrap_count_by_imu[capture.imu_id] = wrap_count
        state.last_timestamp_by_imu[capture.imu_id] = packet_timestamp
        packet_timestamp_continuous = wrap_count * 65536 + packet_timestamp

        fifo_header = packet[0] if packet else 0
        accel_x = _packet_s16(packet, 1, capture.imu_id)
        accel_y = _packet_s16(packet, 3, capture.imu_id)
        accel_z = _packet_s16(packet, 5, capture.imu_id)
        gyro_x = _packet_s16(packet, 7, capture.imu_id)
        gyro_y = _packet_s16(packet, 9, capture.imu_id)
        gyro_z = _packet_s16(packet, 11, capture.imu_id)
        temp_raw = int.from_bytes(packet[13:14], byteorder="big", signed=True) if len(packet) >= 14 else 0
        accel_x_mps2 = _raw_accel_to_mps2(accel_x, capture.imu_id)
        accel_y_mps2 = _raw_accel_to_mps2(accel_y, capture.imu_id)
        accel_z_mps2 = _raw_accel_to_mps2(accel_z, capture.imu_id)
        gyro_x_deg_s = math.degrees(_raw_gyro_to_rad_s(gyro_x, capture.imu_id))
        gyro_y_deg_s = math.degrees(_raw_gyro_to_rad_s(gyro_y, capture.imu_id))
        gyro_z_deg_s = math.degrees(_raw_gyro_to_rad_s(gyro_z, capture.imu_id))
        temp_c = _raw_temp_to_celsius(temp_raw, capture.imu_id)

        yield {
            "packet_index": state.global_packet_index,
            "poll_count": capture.poll_count,
            "imu_id": capture.imu_id,
            "imu_name": _imu_name_from_id(capture.imu_id),
            "poll_timestamp_us": capture.timestamp_us,
            "packet_index_in_capture": packet_index + 1,
            "packet_size": capture.packet_size,
            "packet_timestamp_u16": packet_timestamp,
            "packet_timestamp_continuous": packet_timestamp_continuous,
            "fifo_header": fifo_header,
            "raw_accel_x": accel_x,
            "raw_accel_y": accel_y,
            "raw_accel_z": accel_z,
            "raw_gyro_x": gyro_x,
            "raw_gyro_y": gyro_y,
            "raw_gyro_z": gyro_z,
            "temp_raw": temp_raw,
            "accel_x_mps2": accel_x_mps2,
            "accel_y_mps2": accel_y_mps2,
            "accel_z_mps2": accel_z_mps2,
            "gyro_x_deg_s": gyro_x_deg_s,
            "gyro_y_deg_s": gyro_y_deg_s,
            "gyro_z_deg_s": gyro_z_deg_s,
            "temp_c": temp_c,
        }


def _build_synthetic_frame_from_raw_capture(capture: RawImuCapture) -> RealImuFrame | None:
    if capture.packet_size <= 0:
        return None

    packet_count = capture.fifo_byte_count // capture.packet_size
    if packet_count <= 0:
        return None

    packets = [
        capture.payload[index * capture.packet_size : (index + 1) * capture.packet_size]
        for index in range(packet_count)
        if len(capture.payload[index * capture.packet_size : (index + 1) * capture.packet_size]) == capture.packet_size
        and not _should_skip_packet_for_analysis(
            capture.payload[index * capture.packet_size : (index + 1) * capture.packet_size],
            capture.imu_id,
        )
    ]
    if not packets:
        return None

    same_flag = 0
    acc_sum = [0, 0, 0]
    gyro_sum = [0, 0, 0]
    temp_sum = 0
    header_sum = 0
    parsed_count = 0

    for index, packet_i in enumerate(packets):
        if len(packet_i) < 14:
            break

        acc_i = [_packet_s16(packet_i, 0x01, capture.imu_id), _packet_s16(packet_i, 0x03, capture.imu_id), _packet_s16(packet_i, 0x05, capture.imu_id)]
        gyro_i = [_packet_s16(packet_i, 0x07, capture.imu_id), _packet_s16(packet_i, 0x09, capture.imu_id), _packet_s16(packet_i, 0x0B, capture.imu_id)]
        acc_sum = [acc_sum[axis] + acc_i[axis] for axis in range(3)]
        gyro_sum = [gyro_sum[axis] + gyro_i[axis] for axis in range(3)]
        temp_sum += _packet_s8(packet_i, 0x0D)
        header_sum += packet_i[0]
        parsed_count += 1

        for packet_j in packets[index + 1 :]:
            acc_j = [_packet_s16(packet_j, 0x01, capture.imu_id), _packet_s16(packet_j, 0x03, capture.imu_id), _packet_s16(packet_j, 0x05, capture.imu_id)]
            gyro_j = [_packet_s16(packet_j, 0x07, capture.imu_id), _packet_s16(packet_j, 0x09, capture.imu_id), _packet_s16(packet_j, 0x0B, capture.imu_id)]
            ts_i = _packet_timestamp_u16(packet_i, capture.imu_id)
            ts_j = _packet_timestamp_u16(packet_j, capture.imu_id)

            if acc_i[0] == acc_j[0]:
                same_flag |= 1 << 5
            if acc_i[1] == acc_j[1]:
                same_flag |= 1 << 4
            if acc_i[2] == acc_j[2]:
                same_flag |= 1 << 3
            if gyro_i[0] == gyro_j[0]:
                same_flag |= 1 << 2
            if gyro_i[1] == gyro_j[1]:
                same_flag |= 1 << 1
            if gyro_i[2] == gyro_j[2]:
                same_flag |= 1 << 0
            if packet_i[0] == packet_j[0]:
                same_flag |= 1 << 7
            if ts_i == ts_j:
                same_flag |= 1 << 6

    if parsed_count <= 0:
        return None

    last_packet = packets[parsed_count - 1]
    fifo_timestamp = _packet_timestamp_u16(last_packet, capture.imu_id) if capture.packet_size >= 16 else 0
    avg_accel = [_div_round_s32(value, parsed_count) for value in acc_sum]
    avg_gyro = [_div_round_s32(value, parsed_count) for value in gyro_sum]
    avg_temp = _div_round_s32(temp_sum, parsed_count)
    fifo_header = _div_round_s32(header_sum, parsed_count) & 0xFF

    return RealImuFrame(
        some_flag=same_flag,
        fifo_packet_count=parsed_count,
        timestamp_us=capture.timestamp_us,
        fifo_header=fifo_header,
        fifo_timestamp=fifo_timestamp,
        accel_x=avg_accel[0],
        accel_y=avg_accel[1],
        accel_z=avg_accel[2],
        gyro_x=avg_gyro[0],
        gyro_y=avg_gyro[1],
        gyro_z=avg_gyro[2],
        temp_raw=avg_temp,
        same_header=1 if (same_flag & (1 << 7)) else 0,
        same_fifo_timestamp=1 if (same_flag & (1 << 6)) else 0,
        same_acc_x=1 if (same_flag & (1 << 5)) else 0,
        same_acc_y=1 if (same_flag & (1 << 4)) else 0,
        same_acc_z=1 if (same_flag & (1 << 3)) else 0,
        same_gyro_x=1 if (same_flag & (1 << 2)) else 0,
        same_gyro_y=1 if (same_flag & (1 << 1)) else 0,
        same_gyro_z=1 if (same_flag & (1 << 0)) else 0,
    )


def _build_synthetic_frames_from_raw_captures(raw_captures: list[RawImuCapture]) -> list[RealImuFrame]:
    frames: list[RealImuFrame] = []
    for capture in raw_captures:
        frame = _build_synthetic_frame_from_raw_capture(capture)
        if frame is not None:
            frames.append(frame)
    return frames


def _write_per_imu_packet_csvs(raw_rows: list[dict[str, int | float | str]], source: Path, output_dir: Path) -> list[Path]:
    output_paths: list[Path] = []
    imu_ids = sorted({int(row["imu_id"]) for row in raw_rows})
    for imu_id in imu_ids:
        imu_name = _imu_name_from_id(imu_id)
        destination = output_dir / f"{source.stem}_{imu_name}.csv"
        output_paths.append(destination)
        prefix = f"{imu_name}_"
        imu_rows = [row for row in raw_rows if int(row["imu_id"]) == imu_id]
        with destination.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "packet_index",
                    f"{prefix}poll_timestamp_us",
                    f"{prefix}packet_index_in_capture",
                    f"{prefix}packet_size",
                    f"{prefix}packet_timestamp_u16",
                    f"{prefix}packet_timestamp_continuous",
                    f"{prefix}fifo_header",
                    f"{prefix}raw_accel_x",
                    f"{prefix}raw_accel_y",
                    f"{prefix}raw_accel_z",
                    f"{prefix}raw_gyro_x",
                    f"{prefix}raw_gyro_y",
                    f"{prefix}raw_gyro_z",
                    f"{prefix}temp_raw",
                    f"{prefix}accel_x_mps2",
                    f"{prefix}accel_y_mps2",
                    f"{prefix}accel_z_mps2",
                    f"{prefix}gyro_x_deg_s",
                    f"{prefix}gyro_y_deg_s",
                    f"{prefix}gyro_z_deg_s",
                    f"{prefix}temp_c",
                ]
            )
            for row in imu_rows:
                writer.writerow(
                    [
                        row["packet_index"],
                        row["poll_timestamp_us"],
                        row["packet_index_in_capture"],
                        row["packet_size"],
                        row["packet_timestamp_u16"],
                        row["packet_timestamp_continuous"],
                        row["fifo_header"],
                        row["raw_accel_x"],
                        row["raw_accel_y"],
                        row["raw_accel_z"],
                        row["raw_gyro_x"],
                        row["raw_gyro_y"],
                        row["raw_gyro_z"],
                        row["temp_raw"],
                        _format_float(float(row["accel_x_mps2"])),
                        _format_float(float(row["accel_y_mps2"])),
                        _format_float(float(row["accel_z_mps2"])),
                        _format_float(float(row["gyro_x_deg_s"])),
                        _format_float(float(row["gyro_y_deg_s"])),
                        _format_float(float(row["gyro_z_deg_s"])),
                        _format_float(float(row["temp_c"])),
                    ]
                )
    return output_paths


def _write_poll_comparison_csv(raw_rows: list[dict[str, int | float | str]], source: Path, output_dir: Path) -> Path:
    destination = output_dir / f"{source.stem}_poll_compare.csv"
    imu_ids = sorted({int(row["imu_id"]) for row in raw_rows})
    rows_by_poll: dict[int, dict[int, list[dict[str, int | float | str]]]] = {}

    for row in raw_rows:
        poll_count = int(row["poll_count"])
        imu_id = int(row["imu_id"])
        rows_by_poll.setdefault(poll_count, {}).setdefault(imu_id, []).append(row)

    header = ["poll_count"]
    for imu_id in imu_ids:
        imu_name = _imu_name_from_id(imu_id)
        header.extend(
            [
                f"{imu_name}_pkt_count",
                f"{imu_name}_accel_x_mps2",
                f"{imu_name}_accel_y_mps2",
                f"{imu_name}_accel_z_mps2",
                f"{imu_name}_gyro_x_deg_s",
                f"{imu_name}_gyro_y_deg_s",
                f"{imu_name}_gyro_z_deg_s",
                f"{imu_name}_temp_c",
            ]
        )

    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for poll_count in sorted(rows_by_poll):
            row_out: list[str | int] = [poll_count]
            per_imu_rows = rows_by_poll[poll_count]
            for imu_id in imu_ids:
                imu_rows = per_imu_rows.get(imu_id, [])
                if not imu_rows:
                    row_out.extend([0] + [""] * 7)
                    continue
                row_out.extend(
                    [
                        len(imu_rows),
                        _format_float(_mean([float(item["accel_x_mps2"]) for item in imu_rows])),
                        _format_float(_mean([float(item["accel_y_mps2"]) for item in imu_rows])),
                        _format_float(_mean([float(item["accel_z_mps2"]) for item in imu_rows])),
                        _format_float(_mean([float(item["gyro_x_deg_s"]) for item in imu_rows])),
                        _format_float(_mean([float(item["gyro_y_deg_s"]) for item in imu_rows])),
                        _format_float(_mean([float(item["gyro_z_deg_s"]) for item in imu_rows])),
                        _format_float(_mean([float(item["temp_c"]) for item in imu_rows])),
                    ]
                )
            writer.writerow(row_out)

    return destination


@dataclass
class _PollAggregate:
    packet_count: int = 0
    accel_x_sum: float = 0.0
    accel_y_sum: float = 0.0
    accel_z_sum: float = 0.0
    gyro_x_sum: float = 0.0
    gyro_y_sum: float = 0.0
    gyro_z_sum: float = 0.0
    temp_sum: float = 0.0

    def add(self, row: dict[str, int | float | str]) -> None:
        self.packet_count += 1
        self.accel_x_sum += float(row["accel_x_mps2"])
        self.accel_y_sum += float(row["accel_y_mps2"])
        self.accel_z_sum += float(row["accel_z_mps2"])
        self.gyro_x_sum += float(row["gyro_x_deg_s"])
        self.gyro_y_sum += float(row["gyro_y_deg_s"])
        self.gyro_z_sum += float(row["gyro_z_deg_s"])
        self.temp_sum += float(row["temp_c"])


class StreamingPerImuCsvWriter:
    def __init__(self, source: Path, output_dir: Path) -> None:
        self.source = source
        self.output_dir = output_dir
        self.handles: dict[int, object] = {}
        self.writers: dict[int, csv.writer] = {}
        self.paths: dict[int, Path] = {}

    def _ensure_writer(self, imu_id: int) -> csv.writer:
        writer = self.writers.get(imu_id)
        if writer is not None:
            return writer

        imu_name = _imu_name_from_id(imu_id)
        destination = self.output_dir / f"{self.source.stem}_{imu_name}.csv"
        handle = destination.open("w", newline="", encoding="utf-8")
        writer = csv.writer(handle)
        prefix = f"{imu_name}_"
        writer.writerow(
            [
                "packet_index",
                f"{prefix}poll_timestamp_us",
                f"{prefix}packet_index_in_capture",
                f"{prefix}packet_size",
                f"{prefix}packet_timestamp_u16",
                f"{prefix}packet_timestamp_continuous",
                f"{prefix}fifo_header",
                f"{prefix}raw_accel_x",
                f"{prefix}raw_accel_y",
                f"{prefix}raw_accel_z",
                f"{prefix}raw_gyro_x",
                f"{prefix}raw_gyro_y",
                f"{prefix}raw_gyro_z",
                f"{prefix}temp_raw",
                f"{prefix}accel_x_mps2",
                f"{prefix}accel_y_mps2",
                f"{prefix}accel_z_mps2",
                f"{prefix}gyro_x_deg_s",
                f"{prefix}gyro_y_deg_s",
                f"{prefix}gyro_z_deg_s",
                f"{prefix}temp_c",
            ]
        )
        self.handles[imu_id] = handle
        self.writers[imu_id] = writer
        self.paths[imu_id] = destination
        return writer

    def write_row(self, row: dict[str, int | float | str]) -> None:
        imu_id = int(row["imu_id"])
        writer = self._ensure_writer(imu_id)
        writer.writerow(
            [
                row["packet_index"],
                row["poll_timestamp_us"],
                row["packet_index_in_capture"],
                row["packet_size"],
                row["packet_timestamp_u16"],
                row["packet_timestamp_continuous"],
                row["fifo_header"],
                row["raw_accel_x"],
                row["raw_accel_y"],
                row["raw_accel_z"],
                row["raw_gyro_x"],
                row["raw_gyro_y"],
                row["raw_gyro_z"],
                row["temp_raw"],
                _format_float(float(row["accel_x_mps2"])),
                _format_float(float(row["accel_y_mps2"])),
                _format_float(float(row["accel_z_mps2"])),
                _format_float(float(row["gyro_x_deg_s"])),
                _format_float(float(row["gyro_y_deg_s"])),
                _format_float(float(row["gyro_z_deg_s"])),
                _format_float(float(row["temp_c"])),
            ]
        )

    def close(self) -> None:
        for handle in self.handles.values():
            handle.close()


class StreamingPollComparisonWriter:
    def __init__(self, source: Path, output_dir: Path) -> None:
        self.destination = output_dir / f"{source.stem}_poll_compare.csv"
        self.imu_ids = sorted(IMU_ID_TO_NAME)
        self.handle = self.destination.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.handle)
        header = ["poll_count"]
        for imu_id in self.imu_ids:
            imu_name = _imu_name_from_id(imu_id)
            header.extend(
                [
                    f"{imu_name}_pkt_count",
                    f"{imu_name}_accel_x_mps2",
                    f"{imu_name}_accel_y_mps2",
                    f"{imu_name}_accel_z_mps2",
                    f"{imu_name}_gyro_x_deg_s",
                    f"{imu_name}_gyro_y_deg_s",
                    f"{imu_name}_gyro_z_deg_s",
                    f"{imu_name}_temp_c",
                ]
            )
        self.writer.writerow(header)
        self.current_poll_count: int | None = None
        self.current_aggregates: dict[int, _PollAggregate] = {}

    def write_row(self, row: dict[str, int | float | str]) -> None:
        poll_count = int(row["poll_count"])
        imu_id = int(row["imu_id"])
        if self.current_poll_count is None:
            self.current_poll_count = poll_count
        if poll_count != self.current_poll_count:
            self._flush_current()
            self.current_poll_count = poll_count
        self.current_aggregates.setdefault(imu_id, _PollAggregate()).add(row)

    def _flush_current(self) -> None:
        if self.current_poll_count is None:
            return
        row_out: list[str | int] = [self.current_poll_count]
        for imu_id in self.imu_ids:
            agg = self.current_aggregates.get(imu_id)
            if agg is None or agg.packet_count <= 0:
                row_out.extend([0] + [""] * 7)
                continue
            row_out.extend(
                [
                    agg.packet_count,
                    _format_float(agg.accel_x_sum / agg.packet_count),
                    _format_float(agg.accel_y_sum / agg.packet_count),
                    _format_float(agg.accel_z_sum / agg.packet_count),
                    _format_float(agg.gyro_x_sum / agg.packet_count),
                    _format_float(agg.gyro_y_sum / agg.packet_count),
                    _format_float(agg.gyro_z_sum / agg.packet_count),
                    _format_float(agg.temp_sum / agg.packet_count),
                ]
            )
        self.writer.writerow(row_out)
        self.current_aggregates = {}
        self.current_poll_count = None

    def close(self) -> None:
        self._flush_current()
        self.handle.close()


def decode_real_imu_raw_packets_to_csv(
    source: Path, destination: Path, progress_callback: Callable[[str], None] | None = None
) -> int:
    destination.parent.mkdir(parents=True, exist_ok=True)
    progress = ParseProgressReporter(
        total_bytes=source.stat().st_size,
        callback=progress_callback,
        stage="逐包CSV导出",
    )
    progress.log(f"开始导出逐包 CSV: `{source}`")
    state = PacketRowBuilderState(wrap_count_by_imu={}, last_timestamp_by_imu={})
    row_count = 0

    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "packet_index",
                "imu_id",
                "imu_name",
                "poll_count",
                "poll_timestamp_us",
                "packet_index_in_capture",
                "packet_size",
                "packet_timestamp_u16",
                "packet_timestamp_continuous",
                "fifo_header",
                "raw_accel_x",
                "raw_accel_y",
                "raw_accel_z",
                "raw_gyro_x",
                "raw_gyro_y",
                "raw_gyro_z",
                "temp_raw",
                "accel_x_mps2",
                "accel_y_mps2",
                "accel_z_mps2",
                "gyro_x_deg_s",
                "gyro_y_deg_s",
                "gyro_z_deg_s",
                "temp_c",
            ]
        )
        for record_type, record in iter_real_imu_bin_file_records(source, progress):
            if record_type != "raw_capture":
                continue
            for row in _iter_capture_packet_rows(record, state):
                row_count += 1
                progress.packet_count = row_count
                writer.writerow(
                    [
                        row["packet_index"],
                        row["imu_id"],
                        row["imu_name"],
                        row["poll_count"],
                        row["poll_timestamp_us"],
                        row["packet_index_in_capture"],
                        row["packet_size"],
                        row["packet_timestamp_u16"],
                        row["packet_timestamp_continuous"],
                        row["fifo_header"],
                        row["raw_accel_x"],
                        row["raw_accel_y"],
                        row["raw_accel_z"],
                        row["raw_gyro_x"],
                        row["raw_gyro_y"],
                        row["raw_gyro_z"],
                        row["temp_raw"],
                        _format_float(float(row["accel_x_mps2"])),
                        _format_float(float(row["accel_y_mps2"])),
                        _format_float(float(row["accel_z_mps2"])),
                        _format_float(float(row["gyro_x_deg_s"])),
                        _format_float(float(row["gyro_y_deg_s"])),
                        _format_float(float(row["gyro_z_deg_s"])),
                        _format_float(float(row["temp_c"])),
                    ]
                )
                progress.maybe_report()

    progress.bytes_processed = progress.total_bytes
    progress.maybe_report(force=True, extra="导出完成")
    return row_count


def _mean(values: list[float]) -> float:
    return float(sum(values)) / float(len(values)) if values else 0.0


def _std(values: list[float]) -> float:
    if not values:
        return 0.0
    avg = _mean(values)
    return sqrt(sum((float(v) - avg) ** 2 for v in values) / float(len(values)))


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return sqrt(sum(float(v) * float(v) for v in values) / float(len(values)))


def _remove_mean(values: list[float]) -> list[float]:
    avg = _mean(values)
    return [float(value) - avg for value in values]


def _estimate_sample_period_s(timestamps_us: list[int]) -> float:
    if len(timestamps_us) < 2:
        return 0.0
    steps = [timestamps_us[index] - timestamps_us[index - 1] for index in range(1, len(timestamps_us))]
    valid_steps = [step for step in steps if step > 0]
    if not valid_steps:
        return 0.0
    return _mean(valid_steps) / 1_000_000.0


def _build_allan_cluster_sizes(sample_count: int) -> list[int]:
    max_cluster = sample_count // 4
    if max_cluster < 1:
        return []

    clusters: list[int] = []
    cluster = 1
    while cluster <= max_cluster:
        if not clusters or cluster != clusters[-1]:
            clusters.append(cluster)
        next_cluster = max(cluster + 1, int(round(cluster * 1.6)))
        if next_cluster == cluster:
            next_cluster += 1
        cluster = next_cluster
    return clusters


def _allan_deviation_from_rate(rate_values: list[float], sample_period_s: float) -> list[tuple[float, float]]:
    if len(rate_values) < 4 or sample_period_s <= 0.0:
        return []

    centered = _remove_mean(rate_values)
    result: list[tuple[float, float]] = []
    for cluster_size in _build_allan_cluster_sizes(len(centered)):
        cluster_count = len(centered) // cluster_size
        if cluster_count < 2:
            continue

        cluster_averages = [
            _mean(centered[index * cluster_size : (index + 1) * cluster_size]) for index in range(cluster_count)
        ]
        diffs = [
            cluster_averages[index + 1] - cluster_averages[index] for index in range(len(cluster_averages) - 1)
        ]
        if not diffs:
            continue

        allan_variance = 0.5 * _mean([diff * diff for diff in diffs])
        if allan_variance < 0.0:
            continue
        result.append((cluster_size * sample_period_s, sqrt(allan_variance)))
    return result


def _estimate_arw_metrics(rate_values_rad_s: list[float], sample_period_s: float) -> dict[str, float]:
    rate_values_deg_s = [math.degrees(value) for value in rate_values_rad_s]
    centered_deg_s = _remove_mean(rate_values_deg_s)
    centered_rad_s = _remove_mean(rate_values_rad_s)
    allan_points = _allan_deviation_from_rate(rate_values_deg_s, sample_period_s)

    best_tau_s = 0.0
    best_slope = 0.0
    best_arw_deg_sqrt_hr = 0.0
    best_error = float("inf")
    for index in range(len(allan_points) - 1):
        tau0_s, adev0_deg_s = allan_points[index]
        tau1_s, adev1_deg_s = allan_points[index + 1]
        if tau0_s <= 0.0 or tau1_s <= 0.0 or adev0_deg_s <= 0.0 or adev1_deg_s <= 0.0:
            continue
        slope = math.log(adev1_deg_s / adev0_deg_s) / math.log(tau1_s / tau0_s)
        error = abs(slope + 0.5)
        if error < best_error:
            best_error = error
            best_tau_s = tau0_s
            best_slope = slope
            best_arw_deg_sqrt_hr = adev0_deg_s * math.sqrt(tau0_s) * 60.0

    allan_min_tau_s = 0.0
    allan_min_deg_s = 0.0
    if allan_points:
        allan_min_tau_s, allan_min_deg_s = min(allan_points, key=lambda item: item[1])

    return {
        "rms_rad_s": _rms(centered_rad_s),
        "rms_deg_s": _rms(centered_deg_s),
        "arw_deg_sqrt_hr": best_arw_deg_sqrt_hr,
        "arw_fit_tau_s": best_tau_s,
        "arw_fit_slope": best_slope,
        "allan_min_tau_s": allan_min_tau_s,
        "allan_min_deg_s": allan_min_deg_s,
        "allan_point_count": float(len(allan_points)),
    }


def _estimate_allan_white_noise_metrics(values: list[float], sample_period_s: float) -> dict[str, float]:
    centered_values = _remove_mean(values)
    allan_points = _allan_deviation_from_rate(values, sample_period_s)

    best_tau_s = 0.0
    best_slope = 0.0
    best_random_walk = 0.0
    best_error = float("inf")
    for index in range(len(allan_points) - 1):
        tau0_s, adev0 = allan_points[index]
        tau1_s, adev1 = allan_points[index + 1]
        if tau0_s <= 0.0 or tau1_s <= 0.0 or adev0 <= 0.0 or adev1 <= 0.0:
            continue
        slope = math.log(adev1 / adev0) / math.log(tau1_s / tau0_s)
        error = abs(slope + 0.5)
        if error < best_error:
            best_error = error
            best_tau_s = tau0_s
            best_slope = slope
            best_random_walk = adev0 * math.sqrt(tau0_s) * 60.0

    allan_min_tau_s = 0.0
    allan_min_value = 0.0
    if allan_points:
        allan_min_tau_s, allan_min_value = min(allan_points, key=lambda item: item[1])

    noise_density_mps2_sqhz = best_random_walk / 60.0 if best_random_walk > 0.0 else 0.0
    noise_density_ug_sqhz = noise_density_mps2_sqhz / 9.80665 * 1_000_000.0 if noise_density_mps2_sqhz > 0.0 else 0.0

    return {
        "rms": _rms(centered_values),
        "random_walk": best_random_walk,
        "noise_density_mps2_sqhz": noise_density_mps2_sqhz,
        "noise_density_ug_sqhz": noise_density_ug_sqhz,
        "fit_tau_s": best_tau_s,
        "fit_slope": best_slope,
        "allan_min_tau_s": allan_min_tau_s,
        "allan_min_value": allan_min_value,
        "allan_point_count": float(len(allan_points)),
    }


def _extract_raw_capture_axes_by_imu(raw_captures: list[RawImuCapture]) -> dict[int, dict[str, list[float]]]:
    metrics_by_imu: dict[int, dict[str, list[float]]] = {}
    wrap_count_by_imu: dict[int, int] = {}
    last_timestamp_by_imu: dict[int, int] = {}
    for capture in raw_captures:
        if capture.packet_size <= 0:
            continue

        imu_axes = metrics_by_imu.setdefault(
            capture.imu_id,
            {
                "poll_timestamp_us": [],
                "packet_timestamp_u16": [],
                "packet_timestamp_continuous": [],
                "temp_c": [],
                "gyro_x_rad_s": [],
                "gyro_y_rad_s": [],
                "gyro_z_rad_s": [],
                "acc_x_mps2": [],
                "acc_y_mps2": [],
                "acc_z_mps2": [],
            },
        )
        packet_count = capture.fifo_byte_count // capture.packet_size
        for packet_index in range(packet_count):
            start = packet_index * capture.packet_size
            end = start + capture.packet_size
            packet = capture.payload[start:end]
            if len(packet) != capture.packet_size or len(packet) < 13:
                continue
            if _should_skip_packet_for_analysis(packet, capture.imu_id):
                continue

            gyro_x = _packet_s16(packet, 7, capture.imu_id)
            gyro_y = _packet_s16(packet, 9, capture.imu_id)
            gyro_z = _packet_s16(packet, 11, capture.imu_id)
            acc_x = _packet_s16(packet, 1, capture.imu_id)
            acc_y = _packet_s16(packet, 3, capture.imu_id)
            acc_z = _packet_s16(packet, 5, capture.imu_id)
            packet_timestamp = _packet_timestamp_u16(packet, capture.imu_id)
            wrap_count = wrap_count_by_imu.get(capture.imu_id, 0)
            last_timestamp = last_timestamp_by_imu.get(capture.imu_id)
            if last_timestamp is not None and packet_timestamp < last_timestamp:
                wrap_count += 1
            wrap_count_by_imu[capture.imu_id] = wrap_count
            last_timestamp_by_imu[capture.imu_id] = packet_timestamp
            packet_timestamp_continuous = wrap_count * 65536 + packet_timestamp

            imu_axes["gyro_x_rad_s"].append(_raw_gyro_to_rad_s(gyro_x, capture.imu_id))
            imu_axes["gyro_y_rad_s"].append(_raw_gyro_to_rad_s(gyro_y, capture.imu_id))
            imu_axes["gyro_z_rad_s"].append(_raw_gyro_to_rad_s(gyro_z, capture.imu_id))
            imu_axes["acc_x_mps2"].append(_raw_accel_to_mps2(acc_x, capture.imu_id))
            imu_axes["acc_y_mps2"].append(_raw_accel_to_mps2(acc_y, capture.imu_id))
            imu_axes["acc_z_mps2"].append(_raw_accel_to_mps2(acc_z, capture.imu_id))
            imu_axes["temp_c"].append(_raw_temp_to_celsius(_packet_s8(packet, 0x0D), capture.imu_id))
            imu_axes["poll_timestamp_us"].append(capture.timestamp_us + packet_index)
            imu_axes["packet_timestamp_u16"].append(packet_timestamp)
            imu_axes["packet_timestamp_continuous"].append(packet_timestamp_continuous)
    return metrics_by_imu


def _analyze_arw_per_imu(raw_captures: list[RawImuCapture]) -> dict[int, dict[str, object]]:
    axes_by_imu = _extract_raw_capture_axes_by_imu(raw_captures)
    report: dict[int, dict[str, object]] = {}

    for imu_id, axes in sorted(axes_by_imu.items()):
        timestamps = [int(value) for value in axes["poll_timestamp_us"]]
        packet_timestamps = [int(value) for value in axes["packet_timestamp_u16"]]
        packet_timestamps_continuous = [int(value) for value in axes["packet_timestamp_continuous"]]
        cfg = _imu_config(imu_id)
        poll_period_s = _estimate_sample_period_s(timestamps)
        packet_steps = _positive_steps(packet_timestamps_continuous)
        packet_period_s = _mean(packet_steps) / 1_000_000.0 if packet_steps else 0.0
        if packet_period_s <= 0.0:
            packet_period_s = 1.0 / cfg["odr_hz"] if cfg["odr_hz"] > 0.0 else 0.0
        if poll_period_s <= 0.0:
            poll_period_s = packet_period_s
        duration_s = packet_period_s * max(len(packet_timestamps_continuous) - 1, 0)
        packet_rate_hz = 1.0 / packet_period_s if packet_period_s > 0.0 else 0.0
        poll_rate_hz = 1.0 / poll_period_s if poll_period_s > 0.0 else 0.0

        gyro_metrics = {
            axis: _estimate_arw_metrics([float(value) for value in axes[f"gyro_{axis}_rad_s"]], packet_period_s)
            for axis in ("x", "y", "z")
        }
        accel_metrics = {
            axis: _estimate_allan_white_noise_metrics([float(value) for value in axes[f"acc_{axis}_mps2"]], packet_period_s)
            for axis in ("x", "y", "z")
        }
        gyro_arw_values = [float(gyro_metrics[axis]["arw_deg_sqrt_hr"]) for axis in ("x", "y", "z") if float(gyro_metrics[axis]["arw_deg_sqrt_hr"]) > 0.0]
        accel_rw_values = [float(accel_metrics[axis]["random_walk"]) for axis in ("x", "y", "z") if float(accel_metrics[axis]["random_walk"]) > 0.0]
        paper_metrics = IMU_PAPER_METRICS.get(imu_id, {})
        # gyro_arw_deg_sqrt_hr 是数据手册噪声密度 [deg/s/sqrt(Hz)]
        # ARW [deg/sqrt(hr)] = N [deg/s/sqrt(Hz)] * 60
        paper_gyro_noise_density = float(paper_metrics.get("gyro_arw_deg_sqrt_hr", 0.0))  # deg/s/sqrt(Hz)
        paper_gyro_arw_deg_sqrt_hr = paper_gyro_noise_density * 60.0  # deg/sqrt(hr)

        # accel_noise_ug_sqhz 是数据手册加速度计噪声密度 [µg/sqrt(Hz)]
        # Accel RW [m/s/sqrt(hr)] = N [m/s²/sqrt(Hz)] * 60
        paper_accel_noise_ug_sqhz = float(paper_metrics.get("accel_noise_ug_sqhz", 0.0))
        paper_accel_noise_mps2_sqhz = paper_accel_noise_ug_sqhz * 1e-6 * 9.80665  # m/s²/sqrt(Hz)
        paper_accel_rw_mps_sqrthr = paper_accel_noise_mps2_sqhz * 60.0            # m/s/sqrt(hr)
        measured_gyro_arw_mean = _mean(gyro_arw_values)
        temp_values = [float(value) for value in axes["temp_c"]]
        paper_accel_zero_bias_mps2 = float(paper_metrics.get("accel_zero_bias_mg", 0.0)) / 1000.0 * 9.80665
        paper_gyro_zero_bias_rad_s = math.radians(float(paper_metrics.get("gyro_zero_bias_dps", 0.0)))
        paper_accel_bias_tc_mps2_c = float(paper_metrics.get("accel_bias_tc_mg_c", 0.0)) / 1000.0 * 9.80665
        paper_gyro_bias_tc_rad_s_c = math.radians(float(paper_metrics.get("gyro_bias_tc_dps_c", 0.0)))
        report[imu_id] = {
            "imu_name": _imu_name_from_id(imu_id),
            "sample_count": len(timestamps),
            "duration_s": duration_s,
            "packet_period_s": packet_period_s,
            "packet_rate_hz": packet_rate_hz,
            "packet_period_mean_us": packet_period_s * 1_000_000.0,
            "packet_period_std_us": float(_std(packet_steps)) if packet_steps else 0.0,
            "packet_timestamp_nonzero_count": sum(1 for value in packet_timestamps if value > 0),
            "configured_odr_hz": cfg["odr_hz"],
            "accel_range_g": cfg["accel_range_g"],
            "gyro_range_dps": cfg["gyro_range_dps"],
            "accel_resolution_mg_lsb": (cfg["accel_range_g"] * 1000.0 / float(1 << 15)) if cfg["accel_range_g"] > 0.0 else 0.0,
            "gyro_resolution_mdps_lsb": (cfg["gyro_range_dps"] * 1000.0 / float(1 << 15)) if cfg["gyro_range_dps"] > 0.0 else 0.0,
            "temp_mean_c": _mean(temp_values),
            "temp_min_c": min(temp_values) if temp_values else 0.0,
            "temp_max_c": max(temp_values) if temp_values else 0.0,
            "temp_range_c": (max(temp_values) - min(temp_values)) if temp_values else 0.0,
            "paper_accel_zero_bias_mps2": paper_accel_zero_bias_mps2,
            "paper_gyro_zero_bias_rad_s": paper_gyro_zero_bias_rad_s,
            "paper_accel_bias_tc_mps2_c": paper_accel_bias_tc_mps2_c,
            "paper_gyro_bias_tc_rad_s_c": paper_gyro_bias_tc_rad_s_c,
            "paper_gyro_noise_density_deg_s_sqhz": paper_gyro_noise_density,
            "paper_gyro_arw_deg_sqrt_hr": paper_gyro_arw_deg_sqrt_hr,
            "paper_gyro_arw_target_deg_sqrt_hr": PAPER_TARGETS["gyro_arw_deg_sqrt_hr_max"] * 60.0,
            "paper_accel_noise_ug_sqhz": paper_accel_noise_ug_sqhz,
            "paper_accel_noise_mps2_sqhz": paper_accel_noise_mps2_sqhz,
            "paper_accel_rw_mps_sqrthr": paper_accel_rw_mps_sqrthr,
            "measured_gyro_arw_mean_deg_sqrt_hr": measured_gyro_arw_mean,
            "measured_acc_rw_mean_mps_sqrt_hr": _mean(accel_rw_values),
            "gyro_arw_change_pct": ((measured_gyro_arw_mean - paper_gyro_arw_deg_sqrt_hr) / paper_gyro_arw_deg_sqrt_hr * 100.0) if paper_gyro_arw_deg_sqrt_hr > 0.0 else 0.0,
            "accel_rw_axis_change_pct": {
                axis: (
                    (float(accel_metrics[axis]["random_walk"]) - paper_accel_rw_mps_sqrthr)
                    / paper_accel_rw_mps_sqrthr * 100.0
                ) if paper_accel_rw_mps_sqrthr > 0.0 else 0.0
                for axis in ("x", "y", "z")
            },
            "gyro": gyro_metrics,
            "accel": accel_metrics,
        }

    return report


def _estimate_std_from_sums(count: int, total: float, total_sq: float) -> float:
    if count <= 0:
        return 0.0
    mean_value = total / count
    variance = max(total_sq / count - mean_value * mean_value, 0.0)
    return sqrt(variance)


def _count_csv_data_rows(csv_path: Path) -> int:
    newline_count = 0
    with csv_path.open("rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            newline_count += chunk.count(b"\n")
    return max(newline_count - 1, 0)


def _collect_poll_comparison_stats(comparison_csv_path: Path) -> dict[str, object]:
    poll_row_count = 0
    packet_sum_by_imu = {imu_id: 0 for imu_id in IMU_ID_TO_NAME}

    with comparison_csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            poll_row_count += 1
            for imu_id, imu_name in IMU_ID_TO_NAME.items():
                pkt_count_field = f"{imu_name}_pkt_count"
                packet_sum_by_imu[imu_id] += int(float(row.get(pkt_count_field, "0") or 0))

    avg_packets_per_poll = {
        imu_id: (packet_sum_by_imu[imu_id] / poll_row_count if poll_row_count > 0 else 0.0)
        for imu_id in IMU_ID_TO_NAME
    }
    return {
        "poll_row_count": poll_row_count,
        "poll_total_duration_s": poll_row_count * ARW_REPORT_POLL_PERIOD_S,
        "avg_packets_per_poll": avg_packets_per_poll,
    }


def _choose_common_decimation_factor(per_imu_csv_paths: dict[int, Path]) -> tuple[int, dict[int, int]]:
    sample_counts = {imu_id: _count_csv_data_rows(path) for imu_id, path in per_imu_csv_paths.items()}
    max_sample_count = max(sample_counts.values(), default=0)
    decimation_factor = 1
    while max_sample_count > 0 and (max_sample_count / decimation_factor) > MAX_ANALYSIS_SERIES_POINTS:
        decimation_factor *= 2
    return decimation_factor, sample_counts


def _format_file_size(byte_count: int) -> str:
    value = float(byte_count)
    units = ["B", "KB", "MB", "GB"]
    unit_index = 0
    while value >= 1024.0 and unit_index < len(units) - 1:
        value /= 1024.0
        unit_index += 1
    return f"{value:.2f} {units[unit_index]}"


def _analyze_single_imu_csv(
    imu_id: int,
    csv_path: Path,
    progress: ParseProgressReporter | None = None,
    decimation_factor: int = 1,
) -> dict[str, object]:
    imu_name = _imu_name_from_id(imu_id)
    prefix = f"{imu_name}_"
    ts_field = f"{prefix}packet_timestamp_continuous"
    poll_ts_field = f"{prefix}poll_timestamp_us"
    temp_field = f"{prefix}temp_c"
    gyro_fields = {axis: f"{prefix}gyro_{axis}_deg_s" for axis in ("x", "y", "z")}
    accel_fields = {axis: f"{prefix}accel_{axis}_mps2" for axis in ("x", "y", "z")}

    series = DecimatedImuSeries(decimation_factor=max(decimation_factor, 1))
    sample_count = 0
    nonzero_timestamp_count = 0
    first_timestamp_us = 0
    last_timestamp_us = 0
    last_poll_timestamp_us = 0
    step_count = 0
    step_sum_us = 0.0
    step_sum_sq_us = 0.0
    prev_timestamp_us: int | None = None
    temp_sum_c = 0.0
    temp_min_c = float("inf")
    temp_max_c = float("-inf")

    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row_index, row in enumerate(reader, start=1):
            timestamp_us = int(float(row.get(ts_field, "0") or 0))
            poll_timestamp_us = int(float(row.get(poll_ts_field, "0") or 0))
            gyro_x_deg_s = float(row.get(gyro_fields["x"], "0") or 0.0)
            gyro_y_deg_s = float(row.get(gyro_fields["y"], "0") or 0.0)
            gyro_z_deg_s = float(row.get(gyro_fields["z"], "0") or 0.0)
            acc_x_mps2 = float(row.get(accel_fields["x"], "0") or 0.0)
            acc_y_mps2 = float(row.get(accel_fields["y"], "0") or 0.0)
            acc_z_mps2 = float(row.get(accel_fields["z"], "0") or 0.0)
            temp_c = float(row.get(temp_field, "0") or 0.0)

            if sample_count == 0:
                first_timestamp_us = timestamp_us
            last_timestamp_us = timestamp_us
            last_poll_timestamp_us = poll_timestamp_us
            sample_count += 1
            temp_sum_c += temp_c
            temp_min_c = min(temp_min_c, temp_c)
            temp_max_c = max(temp_max_c, temp_c)
            if timestamp_us > 0:
                nonzero_timestamp_count += 1
            if prev_timestamp_us is not None:
                step_us = timestamp_us - prev_timestamp_us
                if step_us > 0:
                    step_count += 1
                    step_sum_us += step_us
                    step_sum_sq_us += float(step_us) * float(step_us)
            prev_timestamp_us = timestamp_us

            series.add(
                timestamp_us=timestamp_us,
                gyro_x_deg_s=gyro_x_deg_s,
                gyro_y_deg_s=gyro_y_deg_s,
                gyro_z_deg_s=gyro_z_deg_s,
                acc_x_mps2=acc_x_mps2,
                acc_y_mps2=acc_y_mps2,
                acc_z_mps2=acc_z_mps2,
            )
            if progress is not None and row_index % 2048 == 0:
                progress.bytes_processed = handle.buffer.tell()
                progress.packet_count = row_index
                progress.maybe_report()

    if sample_count <= 0:
        return {}

    if progress is not None:
        progress.bytes_processed = progress.total_bytes
        progress.packet_count = sample_count

    cfg = _imu_config(imu_id)
    packet_period_mean_us = step_sum_us / step_count if step_count > 0 else 0.0
    packet_period_std_us = _estimate_std_from_sums(step_count, step_sum_us, step_sum_sq_us)
    packet_period_s = packet_period_mean_us / 1_000_000.0 if packet_period_mean_us > 0.0 else 0.0
    if packet_period_s <= 0.0 and cfg["odr_hz"] > 0.0:
        packet_period_s = 1.0 / cfg["odr_hz"]

    decimated_timestamps = [int(value) for value in series.timestamps_us]
    analysis_packet_period_s = _estimate_sample_period_s(decimated_timestamps)
    if analysis_packet_period_s <= 0.0 and packet_period_s > 0.0:
        analysis_packet_period_s = packet_period_s * series.decimation_factor

    gyro_metrics = {
        "x": _estimate_arw_metrics([math.radians(value) for value in series.gyro_x_deg_s], analysis_packet_period_s),
        "y": _estimate_arw_metrics([math.radians(value) for value in series.gyro_y_deg_s], analysis_packet_period_s),
        "z": _estimate_arw_metrics([math.radians(value) for value in series.gyro_z_deg_s], analysis_packet_period_s),
    }
    accel_metrics = {
        "x": _estimate_allan_white_noise_metrics(list(series.acc_x_mps2), analysis_packet_period_s),
        "y": _estimate_allan_white_noise_metrics(list(series.acc_y_mps2), analysis_packet_period_s),
        "z": _estimate_allan_white_noise_metrics(list(series.acc_z_mps2), analysis_packet_period_s),
    }

    gyro_arw_values = [
        float(gyro_metrics[axis]["arw_deg_sqrt_hr"])
        for axis in ("x", "y", "z")
        if float(gyro_metrics[axis]["arw_deg_sqrt_hr"]) > 0.0
    ]
    accel_noise_values = [
        float(accel_metrics[axis]["noise_density_ug_sqhz"])
        for axis in ("x", "y", "z")
        if float(accel_metrics[axis]["noise_density_ug_sqhz"]) > 0.0
    ]
    paper_metrics = IMU_PAPER_METRICS.get(imu_id, {})
    paper_gyro_noise_density = float(paper_metrics.get("gyro_arw_deg_sqrt_hr", 0.0))
    paper_gyro_arw_deg_sqrt_hr = paper_gyro_noise_density * 60.0
    paper_accel_noise_ug_sqhz = float(paper_metrics.get("accel_noise_ug_sqhz", 0.0))
    paper_accel_noise_mps2_sqhz = paper_accel_noise_ug_sqhz * 1e-6 * 9.80665
    paper_accel_rw_mps_sqrthr = paper_accel_noise_mps2_sqhz * 60.0

    return {
        "imu_name": imu_name,
        "csv_path": str(csv_path),
        "sample_count": sample_count,
        "analysis_sample_count": len(series.timestamps_us),
        "analysis_decimation_factor": series.decimation_factor,
        "duration_s": (last_timestamp_us - first_timestamp_us) / 1_000_000.0 if sample_count >= 2 else 0.0,
        "packet_period_s": packet_period_s,
        "packet_rate_hz": 1.0 / packet_period_s if packet_period_s > 0.0 else 0.0,
        "packet_period_mean_us": packet_period_mean_us,
        "packet_period_std_us": packet_period_std_us,
        "packet_timestamp_nonzero_count": nonzero_timestamp_count,
        "configured_odr_hz": cfg["odr_hz"],
        "actual_odr_ratio": (1.0 / packet_period_s / cfg["odr_hz"]) if packet_period_s > 0.0 and cfg["odr_hz"] > 0.0 else 0.0,
        "accel_range_g": cfg["accel_range_g"],
        "gyro_range_dps": cfg["gyro_range_dps"],
        "accel_resolution_mg_lsb": (cfg["accel_range_g"] * 1000.0 / float(1 << 15)) if cfg["accel_range_g"] > 0.0 else 0.0,
        "gyro_resolution_mdps_lsb": (cfg["gyro_range_dps"] * 1000.0 / float(1 << 15)) if cfg["gyro_range_dps"] > 0.0 else 0.0,
        "temp_mean_c": temp_sum_c / sample_count,
        "temp_min_c": temp_min_c if temp_min_c != float("inf") else 0.0,
        "temp_max_c": temp_max_c if temp_max_c != float("-inf") else 0.0,
        "temp_range_c": (temp_max_c - temp_min_c) if temp_max_c != float("-inf") and temp_min_c != float("inf") else 0.0,
        "last_poll_timestamp_us": last_poll_timestamp_us,
        "paper_gyro_noise_density_deg_s_sqhz": paper_gyro_noise_density,
        "paper_gyro_arw_deg_sqrt_hr": paper_gyro_arw_deg_sqrt_hr,
        "paper_gyro_arw_target_deg_sqrt_hr": PAPER_TARGETS["gyro_arw_deg_sqrt_hr_max"] * 60.0,
        "paper_accel_noise_ug_sqhz": paper_accel_noise_ug_sqhz,
        "paper_accel_noise_mps2_sqhz": paper_accel_noise_mps2_sqhz,
        "paper_accel_rw_mps_sqrthr": paper_accel_rw_mps_sqrthr,
        "measured_gyro_arw_mean_deg_sqrt_hr": _mean(gyro_arw_values),
        "measured_acc_noise_mean_ug_sqhz": _mean(accel_noise_values),
        "gyro": gyro_metrics,
        "accel": accel_metrics,
    }


def _analyze_arw_per_imu_from_csvs(
    per_imu_csv_paths: dict[int, Path], progress_callback: Callable[[str], None] | None = None
) -> dict[int, dict[str, object]]:
    report: dict[int, dict[str, object]] = {}
    common_decimation_factor, sample_counts = _choose_common_decimation_factor(per_imu_csv_paths)
    if progress_callback is not None:
        progress_callback(
            "统计阶段使用统一降采样倍率: "
            f"`x{common_decimation_factor}`，用于控制 Allan/ARW 内存占用；"
            "各 IMU 统一使用同倍率，不影响横向排序，实际点数/包周期统计仍基于全量数据。"
        )
    for imu_id in sorted(per_imu_csv_paths):
        csv_path = per_imu_csv_paths[imu_id]
        progress = ParseProgressReporter(
            total_bytes=csv_path.stat().st_size,
            callback=progress_callback,
            stage=f"{_imu_name_from_id(imu_id)} 统计",
        )
        progress.log(f"{_imu_name_from_id(imu_id)}: 开始基于 CSV 统计实际采样率和噪声指标")
        report[imu_id] = _analyze_single_imu_csv(
            imu_id,
            csv_path,
            progress,
            decimation_factor=common_decimation_factor,
        )
        report[imu_id]["full_sample_count"] = sample_counts.get(imu_id, int(report[imu_id].get("sample_count", 0)))
        report[imu_id]["analysis_common_decimation_factor"] = common_decimation_factor
        progress.bytes_processed = progress.total_bytes
        progress.maybe_report(force=True, extra="统计完成")
    return report


def _coefficient_of_variation_pct(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = _mean(values)
    if abs(avg) < 1e-12:
        return 0.0
    return _std(values) / abs(avg) * 100.0


def analyze_real_imu_frames(
    test_key: str, frames: list[RealImuFrame], raw_captures: list[RawImuCapture] | None = None
) -> dict[str, str]:
    if not frames:
        raise ValueError("no frames found")

    acc_x = [_raw_accel_to_mps2(frame.accel_x, 1) for frame in frames]
    acc_y = [_raw_accel_to_mps2(frame.accel_y, 1) for frame in frames]
    acc_z = [_raw_accel_to_mps2(frame.accel_z, 1) for frame in frames]
    gyro_x = [_raw_gyro_to_rad_s(frame.gyro_x, 1) for frame in frames]
    gyro_y = [_raw_gyro_to_rad_s(frame.gyro_y, 1) for frame in frames]
    gyro_z = [_raw_gyro_to_rad_s(frame.gyro_z, 1) for frame in frames]
    temp_c = [_raw_temp_to_celsius(frame.temp_raw, 1) for frame in frames]
    timestamps = [frame.timestamp_us for frame in frames]
    fifo_counts = [frame.fifo_packet_count for frame in frames]
    same_flag_count = sum(1 for frame in frames if frame.some_flag != 0)
    same_header = sum(frame.same_header for frame in frames)
    same_fifo_timestamp = sum(frame.same_fifo_timestamp for frame in frames)
    same_acc_x = sum(frame.same_acc_x for frame in frames)
    same_acc_y = sum(frame.same_acc_y for frame in frames)
    same_acc_z = sum(frame.same_acc_z for frame in frames)
    same_gyro_x = sum(frame.same_gyro_x for frame in frames)
    same_gyro_y = sum(frame.same_gyro_y for frame in frames)
    same_gyro_z = sum(frame.same_gyro_z for frame in frames)
    timestamp_steps = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]

    common = {
        "test": test_key,
        "frames": str(len(frames)),
        "timestamp_start_us": str(timestamps[0]),
        "timestamp_end_us": str(timestamps[-1]),
        "timestamp_step_mean_us": f"{_mean(timestamp_steps):.3f}" if timestamp_steps else "0.000",
        "timestamp_step_std_us": f"{_std(timestamp_steps):.3f}" if timestamp_steps else "0.000",
        "fifo_packet_mean": f"{_mean(fifo_counts):.3f}",
        "fifo_packet_max": str(max(fifo_counts)),
        "same_flag_frames": str(same_flag_count),
        "same_header_count": str(same_header),
        "same_fifo_timestamp_count": str(same_fifo_timestamp),
        "same_acc_x_count": str(same_acc_x),
        "same_acc_y_count": str(same_acc_y),
        "same_acc_z_count": str(same_acc_z),
        "same_gyro_x_count": str(same_gyro_x),
        "same_gyro_y_count": str(same_gyro_y),
        "same_gyro_z_count": str(same_gyro_z),
    }

    if test_key == "bias":
        common.update(
            {
                "acc_mean_x_mps2": f"{_mean(acc_x):.3f}",
                "acc_mean_y_mps2": f"{_mean(acc_y):.3f}",
                "acc_mean_z_mps2": f"{_mean(acc_z):.3f}",
                "gyro_mean_x_rad_s": f"{_mean(gyro_x):.3f}",
                "gyro_mean_y_rad_s": f"{_mean(gyro_y):.3f}",
                "gyro_mean_z_rad_s": f"{_mean(gyro_z):.3f}",
            }
        )
    elif test_key == "temp":
        common.update(
            {
                "temp_mean_c": f"{_mean(temp_c):.3f}",
                "temp_std_c": f"{_std(temp_c):.3f}",
            }
        )
    elif test_key == "arw":
        sample_period_s = _estimate_sample_period_s(timestamps)
        duration_s = (timestamps[-1] - timestamps[0]) / 1_000_000.0 if len(timestamps) >= 2 else 0.0
        sample_rate_hz = 1.0 / sample_period_s if sample_period_s > 0.0 else 0.0
        axis_metrics = {
            "x": _estimate_arw_metrics(gyro_x, sample_period_s),
            "y": _estimate_arw_metrics(gyro_y, sample_period_s),
            "z": _estimate_arw_metrics(gyro_z, sample_period_s),
        }
        common.update(
            {
                "duration_s": f"{duration_s:.3f}",
                "sample_period_s": f"{sample_period_s:.6f}",
                "sample_rate_hz": f"{sample_rate_hz:.3f}",
                "gyro_rms_x_rad_s": f"{axis_metrics['x']['rms_rad_s']:.6f}",
                "gyro_rms_y_rad_s": f"{axis_metrics['y']['rms_rad_s']:.6f}",
                "gyro_rms_z_rad_s": f"{axis_metrics['z']['rms_rad_s']:.6f}",
                "gyro_rms_x_deg_s": f"{axis_metrics['x']['rms_deg_s']:.6f}",
                "gyro_rms_y_deg_s": f"{axis_metrics['y']['rms_deg_s']:.6f}",
                "gyro_rms_z_deg_s": f"{axis_metrics['z']['rms_deg_s']:.6f}",
                "gyro_arw_x_deg_sqrt_hr": f"{axis_metrics['x']['arw_deg_sqrt_hr']:.6f}",
                "gyro_arw_y_deg_sqrt_hr": f"{axis_metrics['y']['arw_deg_sqrt_hr']:.6f}",
                "gyro_arw_z_deg_sqrt_hr": f"{axis_metrics['z']['arw_deg_sqrt_hr']:.6f}",
                "allan_min_x_deg_s": f"{axis_metrics['x']['allan_min_deg_s']:.6f}",
                "allan_min_y_deg_s": f"{axis_metrics['y']['allan_min_deg_s']:.6f}",
                "allan_min_z_deg_s": f"{axis_metrics['z']['allan_min_deg_s']:.6f}",
                "allan_min_tau_x_s": f"{axis_metrics['x']['allan_min_tau_s']:.6f}",
                "allan_min_tau_y_s": f"{axis_metrics['y']['allan_min_tau_s']:.6f}",
                "allan_min_tau_z_s": f"{axis_metrics['z']['allan_min_tau_s']:.6f}",
                "arw_fit_tau_x_s": f"{axis_metrics['x']['arw_fit_tau_s']:.6f}",
                "arw_fit_tau_y_s": f"{axis_metrics['y']['arw_fit_tau_s']:.6f}",
                "arw_fit_tau_z_s": f"{axis_metrics['z']['arw_fit_tau_s']:.6f}",
                "arw_fit_slope_x": f"{axis_metrics['x']['arw_fit_slope']:.6f}",
                "arw_fit_slope_y": f"{axis_metrics['y']['arw_fit_slope']:.6f}",
                "arw_fit_slope_z": f"{axis_metrics['z']['arw_fit_slope']:.6f}",
                "allan_point_count_x": str(int(axis_metrics["x"]["allan_point_count"])),
                "allan_point_count_y": str(int(axis_metrics["y"]["allan_point_count"])),
                "allan_point_count_z": str(int(axis_metrics["z"]["allan_point_count"])),
                "acc_rms_x_mps2": f"{_rms(acc_x):.3f}",
                "acc_rms_y_mps2": f"{_rms(acc_y):.3f}",
                "acc_rms_z_mps2": f"{_rms(acc_z):.3f}",
            }
        )
        if raw_captures:
            gyro_by_imu = _extract_raw_capture_axes_by_imu(raw_captures)
            imu_rms_values_deg_s: list[float] = []
            for imu_index, axes in sorted(gyro_by_imu.items()):
                axis_rms_values = [
                    _estimate_arw_metrics([float(value) for value in axes[f"gyro_{axis}_rad_s"]], sample_period_s)["rms_deg_s"]
                    for axis in ("x", "y", "z")
                ]
                axis_rms_values = [value for value in axis_rms_values if value > 0.0]
                if not axis_rms_values:
                    continue
                imu_rms_values_deg_s.append(_mean(axis_rms_values))
                common[f"imu_{imu_index}_gyro_rms_mean_deg_s"] = f"{_mean(axis_rms_values):.6f}"

            if imu_rms_values_deg_s:
                common.update(
                    {
                        "same_model_consistency_sample_count": str(len(imu_rms_values_deg_s)),
                        "same_model_rms_mean_deg_s": f"{_mean(imu_rms_values_deg_s):.6f}",
                        "same_model_rms_std_deg_s": f"{_std(imu_rms_values_deg_s):.6f}",
                        "same_model_rms_cv_pct": f"{_coefficient_of_variation_pct(imu_rms_values_deg_s):.3f}",
                    }
                )
    elif test_key == "vibe":
        common.update(
            {
                "fifo_abnormal_count": str(sum(1 for value in fifo_counts if value != 1)),
                "acc_peak_x_mps2": f"{max(abs(v) for v in acc_x):.3f}",
                "acc_peak_y_mps2": f"{max(abs(v) for v in acc_y):.3f}",
                "acc_peak_z_mps2": f"{max(abs(v) for v in acc_z):.3f}",
            }
        )
    elif test_key == "shock":
        common.update(
            {
                "gyro_peak_x_rad_s": f"{max(abs(v) for v in gyro_x):.3f}",
                "gyro_peak_y_rad_s": f"{max(abs(v) for v in gyro_y):.3f}",
                "gyro_peak_z_rad_s": f"{max(abs(v) for v in gyro_z):.3f}",
            }
        )
    else:
        raise ValueError(f"unsupported test key: {test_key}")

    return common


def _build_arw_metric_rankings(per_imu_report: dict) -> list[dict[str, object]]:
    metric_specs = [
        ("gyro", "x", "Gyro X", "ARW", "°/√hr", "arw_deg_sqrt_hr", "paper_gyro_arw_deg_sqrt_hr"),
        ("gyro", "y", "Gyro Y", "ARW", "°/√hr", "arw_deg_sqrt_hr", "paper_gyro_arw_deg_sqrt_hr"),
        ("gyro", "z", "Gyro Z", "ARW", "°/√hr", "arw_deg_sqrt_hr", "paper_gyro_arw_deg_sqrt_hr"),
        ("accel", "x", "Acc X", "Noise", "ug/sqrt(Hz)", "noise_density_ug_sqhz", "paper_accel_noise_ug_sqhz"),
        ("accel", "y", "Acc Y", "Noise", "ug/sqrt(Hz)", "noise_density_ug_sqhz", "paper_accel_noise_ug_sqhz"),
        ("accel", "z", "Acc Z", "Noise", "ug/sqrt(Hz)", "noise_density_ug_sqhz", "paper_accel_noise_ug_sqhz"),
    ]

    def _sr(imu_id: int) -> dict:
        return per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}  # type: ignore

    rankings: list[dict[str, object]] = []
    for metric_type, axis, axis_label, metric_label, unit, value_key, paper_key in metric_specs:
        ranked_items: list[dict[str, object]] = []
        for imu_id in sorted(int(k) for k in per_imu_report):
            sr = _sr(imu_id)
            if not sr:
                continue
            imu_name = str(sr.get("imu_name", _imu_name_from_id(imu_id)))
            family = "42688" if "42688" in imu_name else "45686"
            value = float(sr[metric_type][axis][value_key])
            ranked_items.append(
                {
                    "imu_id": imu_id,
                    "imu_name": imu_name,
                    "family": family,
                    "value": value,
                }
            )
        ranked_items.sort(key=lambda item: float(item["value"]))
        family_best = {
            family: next((item for item in ranked_items if item["family"] == family), None)
            for family in ("42688", "45686")
        }
        family_winner = "tie"
        red_item = family_best["42688"]
        green_item = family_best["45686"]
        if red_item and green_item:
            red_value = float(red_item["value"])
            green_value = float(green_item["value"])
            if red_value < green_value:
                family_winner = "42688"
            elif green_value < red_value:
                family_winner = "45686"
        elif red_item:
            family_winner = "42688"
        elif green_item:
            family_winner = "45686"

        paper_values = {
            "42688": next(
                (
                    float((_sr(int(item["imu_id"])) or {}).get(paper_key, 0.0))
                    for item in ranked_items
                    if str(item["family"]) == "42688"
                ),
                0.0,
            ),
            "45686": next(
                (
                    float((_sr(int(item["imu_id"])) or {}).get(paper_key, 0.0))
                    for item in ranked_items
                    if str(item["family"]) == "45686"
                ),
                0.0,
            ),
        }

        rankings.append(
            {
                "metric_type": metric_type,
                "axis": axis,
                "axis_label": axis_label,
                "metric_label": metric_label,
                "title": f"{axis_label} {metric_label}",
                "unit": unit,
                "paper_key": paper_key,
                "paper_values": paper_values,
                "ranked_items": ranked_items,
                "family_best": family_best,
                "family_winner": family_winner,
            }
        )
    return rankings


def _format_axis_list(labels: list[str]) -> str:
    if not labels:
        return "无"
    return " / ".join(labels)


def _paper_rank_position_text(ranked_items: list[dict[str, object]], paper_value: float) -> str:
    if paper_value <= 0.0 or not ranked_items:
        return "未提供"

    better_count = sum(1 for item in ranked_items if float(item["value"]) < paper_value)
    equal_count = sum(1 for item in ranked_items if abs(float(item["value"]) - paper_value) < 1e-12)
    if better_count <= 0:
        return "位于 #1 前"
    if better_count >= len(ranked_items):
        return f"位于 #{len(ranked_items)} 后"
    if equal_count > 0:
        return f"约等于 #{better_count}"
    return f"位于 #{better_count} 与 #{better_count + 1} 之间"


def _format_duration_minutes_seconds(duration_s: float) -> str:
    total_seconds = max(int(round(float(duration_s))), 0)
    minutes, seconds = divmod(total_seconds, 60)
    return f"{minutes} 分 {seconds} 秒"


def _build_family_competition_summary(rankings: list[dict[str, object]]) -> list[str]:
    family_axes: dict[str, list[str]] = {"42688": [], "45686": [], "tie": []}
    for ranking in rankings:
        family_axes[str(ranking["family_winner"])].append(str(ranking["title"]))

    lines = [
        "- 45686 领先轴项: "
        f"`{len(family_axes['45686'])}` 项，分别是 `{_format_axis_list(family_axes['45686'])}`。",
        "- 42688 领先轴项: "
        f"`{len(family_axes['42688'])}` 项，分别是 `{_format_axis_list(family_axes['42688'])}`。",
    ]
    if family_axes["tie"]:
        lines.append(f"- 持平轴项: `{_format_axis_list(family_axes['tie'])}`。")
    return lines


def _generate_arw_charts(per_imu_report: dict, destination: Path) -> list[str]:
    """Generate compact duel charts for the report."""
    rankings = _build_arw_metric_rankings(per_imu_report)
    if not rankings:
        return []

    stem = destination.stem
    out_dir = destination.parent

    def _imu_color_from_family(family: str) -> str:
        return "#D64B45" if family == "42688" else "#2E9F5B"

    markdown_lines: list[str] = ["", "## 对比图", ""]

    try:
        import matplotlib  # type: ignore
        matplotlib.use("Agg")
        import matplotlib.font_manager as _fm  # type: ignore
        import matplotlib.pyplot as plt  # type: ignore
        import numpy as np  # type: ignore
    except Exception as exc:  # noqa: BLE001
        markdown_lines.extend([f"> 图表生成失败，仅保留排序表: `{exc}`", ""])
        return markdown_lines

    cjk_candidates = ["SimHei", "Microsoft YaHei", "SimSun", "Arial Unicode MS", "Noto Sans JP"]
    cjk_font = next(
        (font.name for candidate in cjk_candidates for font in _fm.fontManager.ttflist if font.name == candidate),
        None,
    )
    if cjk_font:
        plt.rcParams["font.family"] = cjk_font
    plt.rcParams["axes.unicode_minus"] = False

    def _render_duel_chart(metric_type: str, title: str, filename_suffix: str) -> Path:
        chart_rankings = [ranking for ranking in rankings if str(ranking["metric_type"]) == metric_type]
        fig, ax = plt.subplots(figsize=(11.8, 4.6))
        y_pos = np.arange(len(chart_rankings))
        for idx, ranking in enumerate(chart_rankings):
            red_item = ranking["family_best"]["42688"]
            green_item = ranking["family_best"]["45686"]
            if not red_item or not green_item:
                continue
            red_value = float(red_item["value"])
            green_value = float(green_item["value"])
            ax.plot([red_value, green_value], [idx, idx], color="#BDC3C7", linewidth=2.0, zorder=1)
            ax.scatter(red_value, idx, s=95, color=_imu_color_from_family("42688"), zorder=3)
            ax.scatter(green_value, idx, s=95, color=_imu_color_from_family("45686"), zorder=3)
            ax.text(red_value, idx + 0.12, str(red_item["imu_name"]), fontsize=8, color=_imu_color_from_family("42688"))
            ax.text(green_value, idx - 0.18, str(green_item["imu_name"]), fontsize=8, color=_imu_color_from_family("45686"))
            for family in ("42688", "45686"):
                paper_value = float(ranking["paper_values"].get(family, 0.0))
                if paper_value > 0.0:
                    ax.scatter(
                        paper_value,
                        idx,
                        marker="|",
                        s=220,
                        color=_imu_color_from_family(family),
                        linewidths=2.2,
                        zorder=2,
                    )
        ax.set_yticks(y_pos)
        ax.set_yticklabels([str(ranking["axis_label"]) for ranking in chart_rankings], fontsize=10)
        ax.invert_yaxis()
        ax.grid(axis="x", linestyle=":", linewidth=0.7, alpha=0.6)
        ax.set_axisbelow(True)
        ax.set_xlabel("越靠左越好，圆点=实测最优样本，竖线=纸面值", fontsize=10)
        ax.set_title(title, fontsize=12)
        fig.tight_layout()
        chart_path = out_dir / f"{stem}_{filename_suffix}.png"
        fig.savefig(chart_path, dpi=130, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        return chart_path

    gyro_duel_path = _render_duel_chart("gyro", "Gyro 红绿对决图", "gyro_family_duel")
    accel_duel_path = _render_duel_chart("accel", "Accel 红绿对决图", "accel_family_duel")

    markdown_lines.extend(
        [
            f"![Gyro 红绿对决图](./{gyro_duel_path.name})",
            "",
            f"![Accel 红绿对决图](./{accel_duel_path.name})",
            "",
        ]
    )
    return markdown_lines


def _family_best_average(rankings: list[dict[str, object]], family: str, metric_type: str) -> float:
    values = []
    for ranking in rankings:
        if str(ranking["metric_type"]) != metric_type:
            continue
        best_item = ranking["family_best"].get(family)
        if best_item:
            values.append(float(best_item["value"]))
    return _mean(values)


def _build_arw_markdown_lines(source: Path, summary: dict[str, object], destination: Path) -> list[str]:
    per_imu_report = summary.get("per_imu_report", {})
    if not isinstance(per_imu_report, dict):
        per_imu_report = {}
    filtered_bad_packets = summary.get("filtered_bad_packets", {})
    if not isinstance(filtered_bad_packets, dict):
        filtered_bad_packets = {}
    rankings = _build_arw_metric_rankings(per_imu_report)
    all_imu_ids = [
        imu_id
        for imu_id in sorted(int(k) for k in per_imu_report)
        if per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id))
    ]
    if not all_imu_ids:
        return ["# 角度随机游走ARW对比结果", "", f"- 源文件: `{source.name}`", "", "- 未解析到可用于噪声分析的 IMU 数据。"]

    gyro_42688_best = _family_best_average(rankings, "42688", "gyro")
    gyro_45686_best = _family_best_average(rankings, "45686", "gyro")
    accel_42688_best = _family_best_average(rankings, "42688", "accel")
    accel_45686_best = _family_best_average(rankings, "45686", "accel")
    common_decimation_factor = max(
        int(
            next(
                (
                    float((per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}).get("analysis_common_decimation_factor", 1))
                    for imu_id in all_imu_ids
                ),
                1.0,
            )
        ),
        1,
    )
    if source.exists():
        source_size_text = _format_file_size(source.stat().st_size)
    else:
        source_size_text = str(summary.get("source_size_text", "未知"))
    poll_total_duration_s = float(summary.get("poll_total_duration_s", 0.0) or 0.0)
    example_imu_id = all_imu_ids[0]
    example_report = per_imu_report.get(example_imu_id) or per_imu_report.get(str(example_imu_id)) or {}
    example_imu_name = str(example_report.get("imu_name", _imu_name_from_id(example_imu_id)))
    example_dt_us = float(example_report.get("packet_period_mean_us", 0.0))
    example_dt_s = example_dt_us / 1_000_000.0 if example_dt_us > 0.0 else 0.0

    lines = [
        "# 角度随机游走ARW对比结果",
        "",
        "## 结论先看",
        "",
        f"- BIN 文件: `{source.name}`，序号 `{source.stem}`，文件大小 `{source_size_text}`。",
        (
            f"- 本次记录的 poll 总时长: `{_format_duration_minutes_seconds(poll_total_duration_s)}`。"
            if poll_total_duration_s > 0.0
            else "- 本次记录的 poll 总时长: `0 分 0 秒`。"
        ),
        (
            "- 解析阶段已自动剔除整包 `0x7F` 的异常 FIFO 包，不参与 CSV 导出、采样率统计和噪声计算；"
            + "本次共剔除 "
            + "，".join(f"`{imu_name}` {int(count)} 包" for imu_name, count in filtered_bad_packets.items())
            + "。"
        ) if filtered_bad_packets else "- 解析阶段未检测到需要剔除的整包 `0x7F` 异常 FIFO 包。",
        *_build_family_competition_summary(rankings),
        (
            f"- 按各型号最优样本求 Gyro ARW 三轴均值，42688 为 `{gyro_42688_best:.6f} °/√hr`，"
            f"45686 为 `{gyro_45686_best:.6f} °/√hr`，"
            f"{'42688 更好' if 0.0 < gyro_42688_best < gyro_45686_best else '45686 更好' if 0.0 < gyro_45686_best < gyro_42688_best else '两者接近'}。"
        ),
        (
            f"- 按各型号最优样本求 Acc Noise 三轴均值，42688 为 `{accel_42688_best:.6f} µg/√Hz`，"
            f"45686 为 `{accel_45686_best:.6f} µg/√Hz`，"
            f"{'42688 更好' if 0.0 < accel_42688_best < accel_45686_best else '45686 更好' if 0.0 < accel_45686_best < accel_42688_best else '两者接近'}。"
        ),
        (
            f"- Allan/ARW 统计阶段统一使用 `x{common_decimation_factor}` 时间序列降采样；"
            "四颗 IMU 使用相同倍率，因此不会引入型号间比较偏置。"
        ) if common_decimation_factor > 1 else "- Allan/ARW 统计阶段本次未触发降采样。",
        "",
        "### 基础配置与实测采样",
        "",
    ]

    for imu_id in all_imu_ids:
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id))
        if not sr:
            continue
        filtered_bad_packet_count = int(sr.get("filtered_bad_packet_count", 0))
        lines.append(
            "- "
            f"`{sr['imu_name']}`: 配置 `ODR {float(sr['configured_odr_hz']):.1f} Hz / ±{float(sr['accel_range_g']):.0f} g / ±{float(sr['gyro_range_dps']):.0f} dps`，"
            f"实测包间隔 `{float(sr['packet_period_mean_us']):.3f} us`，"
            f"实测刷新率 `{float(sr['packet_rate_hz']):.3f} Hz`，"
            f"采样点数 `{int(sr['sample_count'])}`，"
            f"平均每 poll 包数 `{float(sr.get('avg_packets_per_poll', 0.0)):.3f}` 包"
            f"{f'，已剔除异常包 `{filtered_bad_packet_count}` 个' if filtered_bad_packet_count > 0 else ''}。"
        )

    lines.extend(
        [
        "",
        "## 噪声指标汇总",
        "",
        "| 指标 | 42688_A | 42688_B | 45686_A | 45686_B | 42688 纸面 | 45686 纸面 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )

    for axis in ("x", "y", "z"):
        row = [f"Gyro {axis.upper()} ARW (°/√hr)"]
        for imu_id in (1, 2, 3, 4):
            sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id))
            row.append(_format_float(float(sr["gyro"][axis]["arw_deg_sqrt_hr"])) if sr else "-")
        row.append(_format_float(float((per_imu_report.get(1) or per_imu_report.get("1") or {}).get("paper_gyro_arw_deg_sqrt_hr", 0.0))))
        row.append(_format_float(float((per_imu_report.get(3) or per_imu_report.get("3") or {}).get("paper_gyro_arw_deg_sqrt_hr", 0.0))))
        lines.append("| " + " | ".join(row) + " |")

    for axis in ("x", "y", "z"):
        row = [f"Acc {axis.upper()} Noise (µg/√Hz)"]
        for imu_id in (1, 2, 3, 4):
            sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id))
            row.append(_format_float(float(sr["accel"][axis]["noise_density_ug_sqhz"])) if sr else "-")
        row.append(_format_float(float((per_imu_report.get(1) or per_imu_report.get("1") or {}).get("paper_accel_noise_ug_sqhz", 0.0))))
        row.append(_format_float(float((per_imu_report.get(3) or per_imu_report.get("3") or {}).get("paper_accel_noise_ug_sqhz", 0.0))))
        lines.append("| " + " | ".join(row) + " |")

    lines.extend(_generate_arw_charts(per_imu_report, destination))
    lines.extend(
        [
            "## 说明",
            "",
            "- Gyro 指标显示为 `°/√hr`，按 Allan deviation 曲线上斜率最接近 `-1/2` 的区段提取。",
            "- Acc 指标显示为 `µg/√Hz`，由 Allan 白噪声区段估计值换算得到，更贴近数据手册口径。",
            "- 表中的采样点数、包间隔和刷新率都来自全量逐包数据，不受 Allan/ARW 降采样设置影响。",
            "",
            "## 指标计算过程",
            "",
            f"- 以下以 `{example_imu_name}` 的 `X` 轴为例说明；`Y/Z` 轴和其他 IMU 的处理流程完全一致，只是替换对应 CSV 列名。",
            f"- 数据来源来自该 IMU 独立 CSV 中的原始列：时间轴使用 `{example_imu_name}_packet_timestamp_continuous`，Gyro X 使用 `{example_imu_name}_gyro_x_deg_s`，Acc X 使用 `{example_imu_name}_accel_x_mps2`。",
            "- 采样周期不再使用 poll 周期，而是直接对 `*_packet_timestamp_continuous` 做正向差分统计，取均值作为该 IMU 的实际包间隔。",
            (
                f"- 以本例为例，实际平均包间隔约为 `{example_dt_us:.3f} us`，即 `dt = {example_dt_s:.9f} s`。"
                if example_dt_s > 0.0
                else "- 实际平均包间隔由 `*_packet_timestamp_continuous` 差分均值给出。"
            ),
            f"- 若 Allan/ARW 阶段启用了统一降采样 `x{common_decimation_factor}`，则只对噪声计算序列按固定步长抽取；包间隔、刷新率、点数统计仍基于全量 CSV。",
            "",
            "### 1. 从 CSV 取原始单轴序列",
            "",
            "```text",
            "timestamp_us[n] = *_packet_timestamp_continuous",
            "gyro_x[n]      = *_gyro_x_deg_s",
            "acc_x[n]       = *_accel_x_mps2",
            "```",
            "",
            "- 其中 `timestamp_us[n]` 是解析后连续展开的包时间戳，单位 `us`，用于恢复该 IMU 自身的真实采样节拍。",
            "- Gyro 序列在 Allan 计算前会按脚本实现转换到 `rad/s` 再参与统计，但提取 ARW 时又按 `deg/s` 口径输出，最终单位为 `°/√hr`。",
            "",
            "### 2. 去均值",
            "",
            "```text",
            "x_centered[n] = x[n] - mean(x)",
            "```",
            "",
            "- Allan deviation 关注随机项，因此先去掉直流均值，避免静态偏置把噪声结果抬高。",
            "",
            "### 3. 构造 Allan deviation 曲线",
            "",
            "对每个 cluster size `m`：",
            "",
            "```text",
            "tau = m * dt",
            "avg_k = mean(x_centered[k*m : (k+1)*m])",
            "sigma(tau) = sqrt(0.5 * mean((avg_{k+1} - avg_k)^2))",
            "```",
            "",
            "- 脚本会自动生成一组递增的 `m`，在每个 `tau` 上得到一个 Allan 点 `(tau, sigma)`。",
            "- 这里的实现对应脚本中的 `_allan_deviation_from_rate(...)`。",
            "",
            "### 4. 选择白噪声区段",
            "",
            "对于相邻两个 Allan 点 `(tau0, sigma0)`、`(tau1, sigma1)`，计算局部斜率：",
            "",
            "```text",
            "slope = log(sigma1 / sigma0) / log(tau1 / tau0)",
            "```",
            "",
            "- 理想白噪声区在 Allan 图上斜率应接近 `-1/2`。",
            "- 脚本会遍历所有相邻点，选取 `abs(slope + 0.5)` 最小的那一段作为指标提取点。",
            "",
            "### 5. Gyro X 的 ARW 计算",
            "",
            "在选中的白噪声点 `(tau0, sigma0_deg_s)` 上，脚本按下面公式计算：",
            "",
            "```text",
            "ARW_deg_sqrt_hr = sigma0_deg_s * sqrt(tau0) * 60",
            "```",
            "",
            "- 其中 `sigma0_deg_s` 的单位是 `deg/s`，乘 `sqrt(s)` 后得到 `deg/sqrt(s)`，再乘 `60` 换算为 `deg/sqrt(hr)`。",
            "- 这正对应脚本 `_estimate_arw_metrics(...)` 中的 `adev0_deg_s * sqrt(tau0_s) * 60.0`。",
            "",
            "### 6. Acc X 的噪声密度计算",
            "",
            "脚本先在白噪声点上取随机游走量：",
            "",
            "```text",
            "random_walk = sigma0_mps2 * sqrt(tau0) * 60",
            "noise_density_mps2_sqrtHz = random_walk / 60",
            "noise_density_ug_sqrtHz = noise_density_mps2_sqrtHz / 9.80665 * 1e6",
            "```",
            "",
            "- 合并后可以理解为先得到 `m/s^2/sqrt(Hz)`，再按 `1 g = 9.80665 m/s^2` 转成 `µg/√Hz`。",
            "- 这对应脚本 `_estimate_allan_white_noise_metrics(...)` 中的 `noise_density_ug_sqhz`。",
            "",
            "### 7. 为什么同一套流程可用于所有轴",
            "",
            "- 各轴唯一变化的只是输入列名，例如 `gyro_y_deg_s`、`gyro_z_deg_s`、`accel_y_mps2`、`accel_z_mps2`。",
            "- 去均值、Allan deviation、白噪声区筛选和单位换算完全一致，因此说明一个轴即可覆盖全部轴的计算过程。",
        ]
    )
    return lines


def _write_markdown_report(destination: Path, source: Path, summary: dict[str, object]) -> None:
    if summary.get("test") == "arw":
        destination.write_text("\n".join(_build_arw_markdown_lines(source, summary, destination)) + "\n", encoding="utf-8")
        return

    lines = ["# 数据分析报告", "", f"- 源文件: `{source}`", "", "## 摘要", ""]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_real_imu_bin_file(
    test_key: str,
    source: Path,
    output_dir: Path | None = None,
    progress_callback: Callable[[str], None] | None = None,
) -> tuple[Path, Path, Path, dict[str, object]]:
    source = source.resolve()
    if not source.exists():
        raise FileNotFoundError(source)

    if output_dir is None:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = ANALYSIS_OUTPUT_DIRS.get(test_key, TOOL_PROJECT_ROOT / "data") / f"{source.stem}_{timestamp}"
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_dir / f"{source.stem}_{test_key}.csv"
    md_path = output_dir / f"{source.stem}_{test_key}_analysis.md"
    legacy_packet_csv_path = output_dir / f"{source.stem}_{test_key}_packets.csv"
    legacy_summary_csv_path = output_dir / f"{source.stem}_{test_key}_summary.csv"

    legacy_packet_csv_path.unlink(missing_ok=True)
    legacy_summary_csv_path.unlink(missing_ok=True)

    progress = ParseProgressReporter(
        total_bytes=source.stat().st_size,
        callback=progress_callback,
        stage="BIN解析",
    )
    progress.log(f"开始解析 BIN: `{source}`")

    per_imu_writer = StreamingPerImuCsvWriter(source, output_dir)
    poll_writer = StreamingPollComparisonWriter(source, output_dir)
    packet_state = PacketRowBuilderState(wrap_count_by_imu={}, last_timestamp_by_imu={})
    raw_capture_imu_ids: set[int] = set()
    primary_packet_csv_path = csv_path
    legacy_frames: list[RealImuFrame] = []
    legacy_raw_captures: list[RawImuCapture] = []

    try:
        for record_type, record in iter_real_imu_bin_file_records(source, progress):
            if record_type == "frame":
                if test_key != "arw":
                    legacy_frames.append(record)  # type: ignore[arg-type]
                continue

            capture = record  # type: ignore[assignment]
            raw_capture_imu_ids.add(capture.imu_id)
            if test_key != "arw":
                legacy_raw_captures.append(capture)
            for row in _iter_capture_packet_rows(capture, packet_state):
                progress.packet_count += 1
                per_imu_writer.write_row(row)
                poll_writer.write_row(row)
                progress.maybe_report()
        poll_writer.close()
        per_imu_writer.close()
    finally:
        try:
            poll_writer.close()
        except Exception:
            pass
        try:
            per_imu_writer.close()
        except Exception:
            pass

    csv_path = poll_writer.destination
    per_imu_csv_paths = dict(sorted(per_imu_writer.paths.items()))
    primary_imu_id = min(per_imu_csv_paths) if per_imu_csv_paths else 0
    if primary_imu_id in per_imu_csv_paths:
        primary_packet_csv_path = per_imu_csv_paths[primary_imu_id]

    summary: dict[str, object]
    if test_key == "arw":
        progress.log("逐包 CSV 已写出，开始基于每个 IMU 的 CSV 统计实际采样率和噪声指标")
        per_imu_report = _analyze_arw_per_imu_from_csvs(per_imu_csv_paths, progress_callback=progress_callback)
        poll_stats = _collect_poll_comparison_stats(csv_path)
        avg_packets_per_poll = poll_stats.get("avg_packets_per_poll", {})
        if isinstance(avg_packets_per_poll, dict):
            for imu_id, average_value in avg_packets_per_poll.items():
                report_item = per_imu_report.get(imu_id)
                if report_item is not None:
                    report_item["avg_packets_per_poll"] = float(average_value)
        filtered_bad_packets = dict(sorted(packet_state.skipped_packets_by_imu.items()))
        for imu_id, skipped_count in filtered_bad_packets.items():
            report_item = per_imu_report.get(imu_id)
            if report_item is not None:
                report_item["filtered_bad_packet_count"] = skipped_count
        primary_report = per_imu_report.get(primary_imu_id, {}) if primary_imu_id else {}
        summary = {
            "test": test_key,
            "source": str(source),
            "comparison_csv": csv_path.name,
            "per_imu_csvs": ", ".join(path.name for _, path in sorted(per_imu_csv_paths.items())),
            "per_imu_report": per_imu_report,
            "source_size_text": _format_file_size(source.stat().st_size),
            "filtered_bad_packets": {
                _imu_name_from_id(imu_id): skipped_count for imu_id, skipped_count in filtered_bad_packets.items()
            },
            "poll_row_count": int(poll_stats.get("poll_row_count", 0)),
            "poll_total_duration_s": float(poll_stats.get("poll_total_duration_s", 0.0)),
            "primary_imu_name": _imu_name_from_id(primary_imu_id) if primary_imu_id else "unknown",
            "frames": str(primary_report.get("sample_count", 0)),
            "duration_s": f"{float(primary_report.get('duration_s', 0.0)):.3f}",
        }
    else:
        primary_captures = [capture for capture in legacy_raw_captures if capture.imu_id == primary_imu_id]
        if not primary_captures:
            primary_captures = legacy_raw_captures
        frames = legacy_frames if legacy_frames else _build_synthetic_frames_from_raw_captures(primary_captures)
        summary = analyze_real_imu_frames(test_key, frames, primary_captures)
        summary["primary_imu_name"] = _imu_name_from_id(primary_imu_id) if primary_imu_id else "unknown"
        summary["comparison_csv"] = csv_path.name
        summary["per_imu_csvs"] = ", ".join(path.name for _, path in sorted(per_imu_csv_paths.items()))
        summary["per_imu_report"] = {}

    progress.log("开始生成 Markdown 报告")
    _write_markdown_report(md_path, source, summary)
    progress.log(f"解析完成，输出目录: `{output_dir}`")
    return csv_path, primary_packet_csv_path, md_path, summary


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
        expected_drive_file=None,
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


def command_probe_imu(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, f"imu_probe{args.index}", timeout=args.timeout)
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_probe_imu_all(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, "imu_probe_all", timeout=args.timeout)
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_noise_prepare(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, "noise_test_prepare", timeout=args.timeout)
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_noise_start(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(
        args.port,
        f"noise_test_start {args.minutes} {args.seconds}",
        timeout=args.timeout,
    )
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_noise_status(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, "noise_test_status", timeout=args.timeout)
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_noise_stop(args: argparse.Namespace) -> int:
    port, response = send_cdc_command(args.port, "noise_test_stop", timeout=args.timeout)
    print(f"port: {port.device}")
    if response:
        print(response)
    return 0


def command_decode_real_bin(args: argparse.Namespace) -> int:
    source = Path(args.input).resolve()
    if not source.exists():
        raise FileNotFoundError(source)

    if args.output:
        csv_destination = Path(args.output).resolve()
    else:
        csv_destination = source.with_suffix(".csv")

    packet_count = decode_real_imu_raw_packets_to_csv(source, csv_destination)
    csv_destination.with_name(f"{csv_destination.stem}_packets.csv").unlink(missing_ok=True)
    print(f"decoded csv: {csv_destination}")
    print(f"packets: {packet_count}")
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

    parser_probe_imu = subparsers.add_parser("probe-imu", help="probe one imu slot")
    parser_probe_imu.add_argument("index", type=int, choices=[1, 2, 3, 4])
    parser_probe_imu.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_probe_imu.add_argument("--timeout", type=float, default=2.0)
    parser_probe_imu.set_defaults(func=command_probe_imu)

    parser_probe_all = subparsers.add_parser("probe-imu-all", help="probe all imu slots")
    parser_probe_all.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_probe_all.add_argument("--timeout", type=float, default=2.0)
    parser_probe_all.set_defaults(func=command_probe_imu_all)

    parser_noise_prepare = subparsers.add_parser("noise-prepare", help="probe imu count before 10s noise recording")
    parser_noise_prepare.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_noise_prepare.add_argument("--timeout", type=float, default=2.0)
    parser_noise_prepare.set_defaults(func=command_noise_prepare)

    parser_noise_start = subparsers.add_parser("noise-start", help="start imu noise recording with custom duration")
    parser_noise_start.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_noise_start.add_argument("--minutes", type=int, default=0, help="recording minutes, default 0")
    parser_noise_start.add_argument("--seconds", type=int, default=10, help="recording seconds, default 10")
    parser_noise_start.add_argument("--timeout", type=float, default=2.0)
    parser_noise_start.set_defaults(func=command_noise_start)

    parser_noise_status = subparsers.add_parser("noise-status", help="get 10s imu noise recording progress")
    parser_noise_status.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_noise_status.add_argument("--timeout", type=float, default=2.0)
    parser_noise_status.set_defaults(func=command_noise_status)

    parser_noise_stop = subparsers.add_parser("noise-stop", help="stop 10s imu noise recording")
    parser_noise_stop.add_argument("--port", help="CDC serial port, e.g. COM78")
    parser_noise_stop.add_argument("--timeout", type=float, default=2.0)
    parser_noise_stop.set_defaults(func=command_noise_stop)

    parser_decode_real = subparsers.add_parser("decode-real-bin", help="decode IMUNOISE.BIN into csv")
    parser_decode_real.add_argument("input", help="path to IMUNOISE.BIN")
    parser_decode_real.add_argument("--output", help="output csv path")
    parser_decode_real.set_defaults(func=command_decode_real_bin)

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
