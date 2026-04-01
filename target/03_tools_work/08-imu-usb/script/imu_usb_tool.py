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
from dataclasses import dataclass
from math import sqrt
from pathlib import Path

from serial_client import DeviceClient, PortInfo


DEFAULT_VID = 0x0FFE
DEFAULT_PID_CDC = 0x0001
DEFAULT_PID_COMPOSITE = 0x0003
DEFAULT_LOG_FILE = "IMU_LOG.CSV"
DEFAULT_BAUDRATE = 115200
DEFAULT_WAIT_SECONDS = 25.0
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


def _imu_uses_little_endian_packets(imu_id: int) -> bool:
    return imu_id in (3, 4)


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
    wrap_count_by_imu: dict[int, int] = {}
    last_timestamp_by_imu: dict[int, int] = {}
    global_packet_index = 0

    for capture in captures:
        if capture.packet_size <= 0:
            continue

        packet_count = capture.fifo_byte_count // capture.packet_size
        for packet_index in range(packet_count):
            start = packet_index * capture.packet_size
            end = start + capture.packet_size
            packet = capture.payload[start:end]
            if len(packet) != capture.packet_size:
                continue

            global_packet_index += 1
            packet_timestamp = _packet_timestamp_u16(packet, capture.imu_id)
            wrap_count = wrap_count_by_imu.get(capture.imu_id, 0)
            last_timestamp = last_timestamp_by_imu.get(capture.imu_id)
            if last_timestamp is not None and packet_timestamp < last_timestamp:
                wrap_count += 1
            wrap_count_by_imu[capture.imu_id] = wrap_count
            last_timestamp_by_imu[capture.imu_id] = packet_timestamp
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

            rows.append(
                {
                    "packet_index": global_packet_index,
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
            )
    return rows


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


def decode_real_imu_raw_packets_to_csv(source: Path, destination: Path) -> int:
    payload = source.read_bytes()
    captures = iter_real_imu_raw_captures(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows = _build_raw_packet_rows(captures)

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
        for row in rows:
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

    return len(rows)


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

    return {
        "rms": _rms(centered_values),
        "random_walk": best_random_walk,
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
        ("gyro", "x", "Gyro X", "ARW", "deg/sqrt(hr)", "arw_deg_sqrt_hr"),
        ("gyro", "y", "Gyro Y", "ARW", "deg/sqrt(hr)", "arw_deg_sqrt_hr"),
        ("gyro", "z", "Gyro Z", "ARW", "deg/sqrt(hr)", "arw_deg_sqrt_hr"),
        ("accel", "x", "Acc X", "RW", "m/s/sqrt(hr)", "random_walk"),
        ("accel", "y", "Acc Y", "RW", "m/s/sqrt(hr)", "random_walk"),
        ("accel", "z", "Acc Z", "RW", "m/s/sqrt(hr)", "random_walk"),
    ]

    def _sr(imu_id: int) -> dict:
        return per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}  # type: ignore

    rankings: list[dict[str, object]] = []
    for metric_type, axis, axis_label, metric_label, unit, value_key in metric_specs:
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

        rankings.append(
            {
                "metric_type": metric_type,
                "axis": axis,
                "axis_label": axis_label,
                "metric_label": metric_label,
                "title": f"{axis_label} {metric_label}",
                "unit": unit,
                "ranked_items": ranked_items,
                "family_best": family_best,
                "family_winner": family_winner,
            }
        )
    return rankings


def _generate_arw_charts(per_imu_report: dict, destination: Path) -> list[str]:
    """Generate ARW/RW markdown sections and charts for the report."""
    rankings = _build_arw_metric_rankings(per_imu_report)
    if not rankings:
        return []

    stem = destination.stem
    out_dir = destination.parent

    def _imu_color_from_family(family: str) -> str:
        return "#D64B45" if family == "42688" else "#2E9F5B"

    family_win_count = {"42688": 0, "45686": 0, "tie": 0}
    for ranking in rankings:
        family_win_count[str(ranking["family_winner"])] += 1

    markdown_lines: list[str] = [
        "",
        "## 噪声指标可视化",
        "",
        "> 红色代表 ICM42688，绿色代表 ICM45686。",
        "> 所有排序都按“数值越小越好”处理，这样可以直接看出每个轴到底是红色更强还是绿色更强。",
        "",
        "### 2.1 红绿胜负速览",
        "",
        f"- ICM42688 胜出轴数: `{family_win_count['42688']}` / `{len(rankings)}`",
        f"- ICM45686 胜出轴数: `{family_win_count['45686']}` / `{len(rankings)}`",
    ]
    if family_win_count["tie"] > 0:
        markdown_lines.append(f"- 平局轴数: `{family_win_count['tie']}`")
    markdown_lines.extend(
        [
            "",
            "| 指标 | 42688 最优 | 45686 最优 | 谁更好 | 差值 |",
            "| --- | ---: | ---: | --- | ---: |",
        ]
    )
    for ranking in rankings:
        red_item = ranking["family_best"]["42688"]
        green_item = ranking["family_best"]["45686"]
        red_value = float(red_item["value"]) if red_item else 0.0
        green_value = float(green_item["value"]) if green_item else 0.0
        red_label = f" ({red_item['imu_name']})" if red_item else ""
        green_label = f" ({green_item['imu_name']})" if green_item else ""
        winner = ranking["family_winner"]
        if winner == "42688":
            winner_text = "红色更好"
        elif winner == "45686":
            winner_text = "绿色更好"
        else:
            winner_text = "平局"
        diff_text = _format_float(abs(red_value - green_value)) if red_item and green_item else "-"
        markdown_lines.append(
            f"| {ranking['title']} | "
            f"`{_format_float(red_value)}`{red_label} | "
            f"`{_format_float(green_value)}`{green_label} | "
            f"{winner_text} | {diff_text} |"
        )

    markdown_lines.extend(
        [
            "",
            "### 2.2 每轴完整排序",
            "",
            "| 指标 | 第1名 | 第2名 | 第3名 | 第4名 |",
            "| --- | --- | --- | --- | --- |",
        ]
    )
    for ranking in rankings:
        cells = []
        for index, item in enumerate(ranking["ranked_items"], start=1):
            family_text = "红" if item["family"] == "42688" else "绿"
            cells.append(f"`#{index}` {item['imu_name']} {family_text} `{_format_float(float(item['value']))}`")
        while len(cells) < 4:
            cells.append("-")
        markdown_lines.append(f"| {ranking['title']} | " + " | ".join(cells[:4]) + " |")
    markdown_lines.append("")

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

    imu_ids = sorted(int(k) for k in per_imu_report)

    def _sr(imu_id: int) -> dict:
        return per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}  # type: ignore

    def _imu_label(imu_id: int) -> str:
        return str(_sr(imu_id).get("imu_name", f"IMU{imu_id}"))

    def _imu_color(imu_id: int) -> str:
        return _imu_color_from_family("42688" if imu_id in (1, 2) else "45686")

    def _get_vals(imu_id: int) -> tuple[list[float], list[float]]:
        sr = _sr(imu_id)
        gyro_vals = [float(sr["gyro"][axis]["arw_deg_sqrt_hr"]) if sr else 0.0 for axis in ("x", "y", "z")]
        accel_vals = [float(sr["accel"][axis]["random_walk"]) if sr else 0.0 for axis in ("x", "y", "z")]
        return gyro_vals, accel_vals

    all_gyro = {imu_id: _get_vals(imu_id)[0] for imu_id in imu_ids}
    all_accel = {imu_id: _get_vals(imu_id)[1] for imu_id in imu_ids}

    def _axis_min(values: dict[int, list[float]], index: int) -> float:
        valid = [values[imu_id][index] for imu_id in imu_ids if values[imu_id][index] > 0.0]
        return min(valid) if valid else 1.0

    gyro_ref = [_axis_min(all_gyro, index) for index in range(3)]
    accel_ref = [_axis_min(all_accel, index) for index in range(3)]

    def _radar_scores(imu_id: int) -> list[float]:
        gyro_vals, accel_vals = _get_vals(imu_id)
        return [gyro_ref[i] / gyro_vals[i] if gyro_vals[i] > 0.0 else 0.0 for i in range(3)] + [
            accel_ref[i] / accel_vals[i] if accel_vals[i] > 0.0 else 0.0 for i in range(3)
        ]

    radar_labels = ["Gyro X", "Gyro Y", "Gyro Z", "Acc X", "Acc Y", "Acc Z"]
    angles = np.linspace(0, 2 * np.pi, len(radar_labels), endpoint=False).tolist()
    angles_closed = angles + angles[:1]

    fig_radar, ax_radar = plt.subplots(figsize=(7.4, 7.2), subplot_kw=dict(polar=True))
    ax_radar.set_theta_offset(np.pi / 2)
    ax_radar.set_theta_direction(-1)
    ax_radar.set_xticks(angles)
    ax_radar.set_xticklabels(radar_labels, fontsize=10)
    ax_radar.set_ylim(0, 1.35)
    ax_radar.set_yticks([0.5, 0.75, 1.0, 1.25])
    ax_radar.set_yticklabels(["0.50", "0.75", "1.00", "1.25"], fontsize=8, color="gray")
    ax_radar.axhline(1.0, color="#95A5A6", linewidth=0.9, linestyle="--", alpha=0.7)
    for imu_id in imu_ids:
        scores = _radar_scores(imu_id)
        closed_scores = scores + [scores[0]]
        ax_radar.plot(angles_closed, closed_scores, color=_imu_color(imu_id), linewidth=2.2, label=_imu_label(imu_id))
        ax_radar.fill(angles_closed, closed_scores, color=_imu_color(imu_id), alpha=0.10)
    ax_radar.legend(loc="upper right", bbox_to_anchor=(1.36, 1.18), fontsize=9)
    ax_radar.set_title("六轴噪声雷达图\n归一化后越大越好", fontsize=12, pad=22)
    fig_radar.patch.set_facecolor("white")
    radar_path = out_dir / f"{stem}_radar.png"
    fig_radar.savefig(radar_path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig_radar)

    fig_rank, axes_grid = plt.subplots(3, 2, figsize=(14.8, 10.8))
    fig_rank.suptitle(
        "每个轴从优到劣排序\n红色 = ICM42688  绿色 = ICM45686  灰虚线 = 纸面值",
        fontsize=13,
    )
    for idx, ranking in enumerate(rankings):
        row, col = divmod(idx, 2)
        sub_ax = axes_grid[row][col]
        ranked_items = list(ranking["ranked_items"])
        labels = [str(item["imu_name"]) for item in ranked_items]
        values = [float(item["value"]) for item in ranked_items]
        colors = [_imu_color_from_family(str(item["family"])) for item in ranked_items]
        y_pos = list(range(len(ranked_items)))
        bars = sub_ax.barh(y_pos, values, color=colors, edgecolor="white", linewidth=0.8, zorder=3)
        sub_ax.set_yticks(y_pos)
        sub_ax.set_yticklabels(labels, fontsize=9)
        sub_ax.invert_yaxis()
        sub_ax.grid(axis="x", linestyle=":", linewidth=0.7, alpha=0.6)
        sub_ax.set_axisbelow(True)
        for order, (bar, value) in enumerate(zip(bars, values), start=1):
            sub_ax.text(
                value * 1.01 if value > 0.0 else 0.001,
                bar.get_y() + bar.get_height() / 2.0,
                f"#{order}  {value:.6f}",
                va="center",
                fontsize=8,
            )
        paper_values = []
        for item in ranked_items:
            sr = _sr(int(item["imu_id"]))
            if ranking["metric_type"] == "gyro":
                paper_values.append(float(sr.get("paper_gyro_arw_deg_sqrt_hr", 0.0)))
            else:
                paper_values.append(float(sr.get("paper_accel_rw_mps_sqrthr", 0.0)))
        paper_ref = next((value for value in paper_values if value > 0.0), 0.0)
        if paper_ref > 0.0:
            sub_ax.axvline(paper_ref, color="#7F8C8D", linestyle="--", linewidth=1.2, zorder=4)
        winner = str(ranking["family_winner"])
        winner_text = "红色领先" if winner == "42688" else "绿色领先" if winner == "45686" else "平局"
        sub_ax.set_title(f"{ranking['title']} ({ranking['unit']})\n{winner_text}", fontsize=10)
    fig_rank.tight_layout(rect=(0, 0, 1, 0.95))
    bars_path = out_dir / f"{stem}_ranked_bars.png"
    fig_rank.savefig(bars_path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig_rank)

    fig_duel, ax_duel = plt.subplots(figsize=(12.5, 5.8))
    y_pos = np.arange(len(rankings))
    for idx, ranking in enumerate(rankings):
        red_item = ranking["family_best"]["42688"]
        green_item = ranking["family_best"]["45686"]
        if not red_item or not green_item:
            continue
        red_value = float(red_item["value"])
        green_value = float(green_item["value"])
        ax_duel.plot([red_value, green_value], [idx, idx], color="#BDC3C7", linewidth=2.0, zorder=1)
        ax_duel.scatter(red_value, idx, s=90, color=_imu_color_from_family("42688"), zorder=3)
        ax_duel.scatter(green_value, idx, s=90, color=_imu_color_from_family("45686"), zorder=3)
        winner_text = "红胜" if ranking["family_winner"] == "42688" else "绿胜" if ranking["family_winner"] == "45686" else "平"
        anchor_x = max(red_value, green_value) * 1.03 if max(red_value, green_value) > 0.0 else 0.01
        ax_duel.text(anchor_x, idx, winner_text, va="center", fontsize=9)
    ax_duel.set_yticks(y_pos)
    ax_duel.set_yticklabels([str(ranking["title"]) for ranking in rankings], fontsize=9)
    ax_duel.invert_yaxis()
    ax_duel.grid(axis="x", linestyle=":", linewidth=0.7, alpha=0.6)
    ax_duel.set_axisbelow(True)
    ax_duel.set_xlabel("数值越靠左越好", fontsize=10)
    ax_duel.set_title("红绿对决图\n每个轴只比较各自型号里的最优一颗 IMU", fontsize=12)
    fig_duel.tight_layout()
    duel_path = out_dir / f"{stem}_family_duel.png"
    fig_duel.savefig(duel_path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig_duel)

    markdown_lines.extend(
        [
            f"![六轴噪声雷达图](./{radar_path.name})",
            "",
            f"![每轴从优到劣排序图](./{bars_path.name})",
            "",
            f"![红绿对决图](./{duel_path.name})",
            "",
        ]
    )
    return markdown_lines


def _write_markdown_report(destination: Path, source: Path, summary: dict[str, str]) -> None:
    if summary.get("test") == "arw":
        primary_csv_name = summary.get("comparison_csv", f"{source.stem}_{summary.get('test', 'arw')}.csv")
        per_imu_report = summary.get("per_imu_report", {})
        detected_imu_ids = sorted(int(imu_id) for imu_id in per_imu_report)
        missing_imu_names = [_imu_name_from_id(imu_id) for imu_id in sorted(IMU_ID_TO_NAME) if imu_id not in detected_imu_ids]
        primary_imu_id = next((imu_id for imu_id in detected_imu_ids if _imu_name_from_id(imu_id) == summary.get("primary_imu_name", "")), 0)
        primary_imu_report = per_imu_report.get(primary_imu_id, {}) if primary_imu_id else {}
        _all_imu_ids = sorted(IMU_ID_TO_NAME)
        _col_hdr: list[str] = []
        for _sid in _all_imu_ids:
            _col_hdr.extend([_imu_name_from_id(_sid), "Δ%"])
        _tbl_header = "| 轴向 | " + " | ".join(_col_hdr) + " |"
        _tbl_sep = "| --- | " + " | ".join(["---:"] * len(_col_hdr)) + " |"
        _tbl_rows: list[str] = [_tbl_header, _tbl_sep]
        for _axis in ("x", "y", "z"):
            _row = [f"Gyro {_axis.upper()}"]
            for _sid in _all_imu_ids:
                _sr = per_imu_report.get(_sid) or per_imu_report.get(str(_sid))
                if not _sr:
                    _row.extend(["-", "-"])
                    continue
                _arw = float(_sr["gyro"][_axis]["arw_deg_sqrt_hr"])
                _paper = float(_sr.get("paper_gyro_arw_deg_sqrt_hr", 0.0))
                _delta = f"{(_arw - _paper) / _paper * 100.0:+.1f}" if _paper > 0.0 else "-"
                _row.extend([_format_float(_arw), _delta])
            _tbl_rows.append("| " + " | ".join(_row) + " |")
        for _axis in ("x", "y", "z"):
            _row = [f"Acc {_axis.upper()}"]
            for _sid in _all_imu_ids:
                _sr = per_imu_report.get(_sid) or per_imu_report.get(str(_sid))
                if not _sr:
                    _row.extend(["-", "-"])
                    continue
                _rw = float(_sr["accel"][_axis]["random_walk"])
                _paper_arw = float(_sr.get("paper_accel_rw_mps_sqrthr", 0.0))
                _delta = f"{(_rw - _paper_arw) / _paper_arw * 100.0:+.1f}" if _paper_arw > 0.0 else "-"
                _row.extend([_format_float(_rw), _delta])
            _tbl_rows.append("| " + " | ".join(_row) + " |")

        lines = [
            "# 数据分析报告",
            "",
            "## 随机游走指标汇总",
            "",
            "> Gyro ARW: deg/sqrt(hr)  |  Acc 随机游走: m/s/sqrt(hr)  |  Δ% = 实测相对纸面变化",
            "",
            *_tbl_rows,
            "",
            "## 1. 测试概况",
            "",
            f"- 源文件: `{source}`",
            f"- 对比 CSV: `{primary_csv_name}`",
            f"- 独立 IMU CSV: `{summary.get('per_imu_csvs', '')}`",
            f"- 本页指标默认针对: `{summary.get('primary_imu_name', 'unknown')}`",
            "- 测试项目: `测试项目 3：角度随机游走 ARW / 噪声`",
            f"- 样本点数: `{summary['frames']}`",
            f"- 记录时长: `{summary.get('duration_s', '0')}` s",
            (
                f"- 主 IMU FIFO 包平均周期: `{primary_imu_report['packet_period_s']:.6f}` s (`{primary_imu_report['packet_period_mean_us']:.3f} us`)"
                if primary_imu_report
                else "- 主 IMU FIFO 包平均周期: `无`"
            ),
            (
                f"- 主 IMU FIFO 包平均刷新率: `{primary_imu_report['packet_rate_hz']:.3f}` Hz"
                if primary_imu_report
                else "- 主 IMU FIFO 包平均刷新率: `无`"
            ),
            f"- 检测到数据的 IMU: `{', '.join(_imu_name_from_id(imu_id) for imu_id in detected_imu_ids) or '无'}`",
            f"- 未检测到数据的 IMU: `{', '.join(missing_imu_names) or '无'}`",
        ]
        lines.extend(["", "## 2. 分 IMU 噪声统计", ""])
        for imu_id in sorted(IMU_ID_TO_NAME):
            imu_name = _imu_name_from_id(imu_id)
            imu_report = per_imu_report.get(str(imu_id)) or per_imu_report.get(imu_id)
            lines.append(f"### {imu_name}")
            lines.append("")
            if not imu_report:
                lines.append("- 本次 BIN 中未检测到该 IMU 的有效原始包。")
                lines.append("")
                continue

            gyro_metrics = imu_report["gyro"]
            accel_metrics = imu_report["accel"]
            lines.extend(
                [
                    f"- 样本点数: `{imu_report['sample_count']}`",
                    f"- FIFO 包平均周期: `{imu_report['packet_period_s']:.6f}` s (`{imu_report['packet_period_mean_us']:.3f} us`)",
                    f"- FIFO 包平均刷新率: `{imu_report['packet_rate_hz']:.3f}` Hz",
                    f"- FIFO 包周期标准差: `{imu_report['packet_period_std_us']:.3f} us`",
                    f"- 记录时长: `{imu_report['duration_s']:.3f}` s",
                    f"- 平均温度: `{imu_report['temp_mean_c']:.3f} °C`",
                    f"- 温度范围: `{imu_report['temp_min_c']:.3f} ~ {imu_report['temp_max_c']:.3f} °C`",
                    f"- 温度变化范围: `{imu_report['temp_range_c']:.3f} °C`",
                    f"- 非零包时间戳点数: `{imu_report['packet_timestamp_nonzero_count']}`",
                    f"- 配置 ODR: `{imu_report['configured_odr_hz']:.1f} Hz`",
                    f"- Acc 量程/分辨率: `±{imu_report['accel_range_g']:.0f} g`, `{imu_report['accel_resolution_mg_lsb']:.6f} mg/LSB`",
                    f"- Gyro 量程/分辨率: `±{imu_report['gyro_range_dps']:.0f} dps`, `{imu_report['gyro_resolution_mdps_lsb']:.6f} mdps/LSB`",
                    "",
                    "| 类型 | 轴向 | RMS | 随机游走指标 | Allan 最小值 | 特征 tau (s) |",
                    "| --- | --- | ---: | ---: | ---: | ---: |",
                ]
            )
            for axis in ("x", "y", "z"):
                axis_gyro = gyro_metrics[axis]
                lines.append(
                    "| Gyro | "
                    f"{axis.upper()} | "
                    f"{axis_gyro['rms_deg_s']:.6f} deg/s | "
                    f"{axis_gyro['arw_deg_sqrt_hr']:.6f} deg/sqrt(hr) | "
                    f"{axis_gyro['allan_min_deg_s']:.6f} deg/s | "
                    f"{axis_gyro['allan_min_tau_s']:.6f} |"
                )
            for axis in ("x", "y", "z"):
                axis_acc = accel_metrics[axis]
                lines.append(
                    "| Acc | "
                    f"{axis.upper()} | "
                    f"{axis_acc['rms']:.6f} m/s^2 | "
                    f"{axis_acc['random_walk']:.6f} m/s/sqrt(hr) | "
                    f"{axis_acc['allan_min_value']:.6f} m/s^2 | "
                    f"{axis_acc['allan_min_tau_s']:.6f} |"
                )
            lines.extend(
                [
                    "",
                    "- `Gyro` 行延续原有 ARW 算法。",
                    "- `Acc` 行采用 Allan deviation 白噪声区段提取的加速度随机游走指标，单位为 `m/s/sqrt(hr)`。",
                    "- `FIFO 包平均周期/刷新率` 直接基于逐包 `packet_timestamp_continuous` 差分统计。",
                    "",
                    "#### 纸面对比",
                    "",
                    f"- 实测 Gyro ARW 三轴均值: `{imu_report['measured_gyro_arw_mean_deg_sqrt_hr']:.6f}` deg/sqrt(hr)",
                    (
                        f"- 文档纸面 Gyro Noise Density: `{imu_report['paper_gyro_noise_density_deg_s_sqhz']:.6f}` deg/s/sqrt(Hz)"
                        f"  →  换算 Gyro ARW: `{imu_report['paper_gyro_arw_deg_sqrt_hr']:.6f}` deg/sqrt(hr)"
                    ) if float(imu_report["paper_gyro_noise_density_deg_s_sqhz"]) > 0.0 else "- 文档纸面 Gyro Noise Density: `未给出`",
                    f"- Gyro ARW 相对纸面变化: `{imu_report['gyro_arw_change_pct']:+.2f} %`" if float(imu_report["paper_gyro_arw_deg_sqrt_hr"]) > 0.0 else "- Gyro ARW 相对纸面变化: `无法计算`",
                    "",
                ]
            )
            if float(imu_report.get("paper_accel_noise_ug_sqhz", 0.0)) > 0.0:
                lines.extend(
                    [
                        f"- 文档纸面 Accel Noise Density: `{imu_report['paper_accel_noise_ug_sqhz']:.1f}` µg/sqrt(Hz)"
                        f"  →  换算 Accel RW: `{imu_report['paper_accel_rw_mps_sqrthr']:.6f}` m/s/sqrt(hr)",
                    ] + [
                        f"- Acc {_ax.upper()} 随机游走相对纸面变化: `{imu_report['accel_rw_axis_change_pct'][_ax]:+.2f} %`"
                        for _ax in ("x", "y", "z")
                    ] + [""]
                )
            else:
                lines.extend(["- Acc 随机游走纸面对比: `01-测试过程和结果.md` 当前未给出对应纸面值，只展示实测结果。", ""])

        try:
            lines.extend(_generate_arw_charts(per_imu_report, destination))
        except Exception as _chart_err:  # noqa: BLE001
            lines.extend(["", f"> ⚠️ 图表生成失败: {_chart_err}", ""])

        lines.extend(
            [
                "## 3. 核心指标",
                "",
                "| 轴向 | Gyro RMS (deg/s) | ARW (deg/sqrt(hr)) | Allan 最小值 (deg/s) | 特征 tau (s) |",
                "| --- | ---: | ---: | ---: | ---: |",
            ]
        )
        for axis in ("x", "y", "z"):
            lines.append(
                "| "
                f"{axis.upper()} | "
                f"{summary.get(f'gyro_rms_{axis}_deg_s', '0')} | "
                f"{summary.get(f'gyro_arw_{axis}_deg_sqrt_hr', '0')} | "
                f"{summary.get(f'allan_min_{axis}_deg_s', '0')} | "
                f"{summary.get(f'allan_min_tau_{axis}_s', '0')} |"
            )

        lines.extend(
            [
                "",
                "说明:",
                    f"- `Gyro RMS` 使用去均值后的陀螺序列计算，直接反映时域噪声大小。",
                    f"- `ARW` 取 Allan deviation 曲线中斜率最接近 `-1/2` 的区段估算。",
                    f"- `Allan 最小值` 用来观察噪声曲线的最低点以及对应时间尺度。",
                "",
                "## 4. 计算过程",
                "",
                "1. 去均值",
                "",
                "- 对每个轴的陀螺序列先去均值: `w_i' = w_i - mean(w)`",
                "- 这样可以把固定零偏从噪声统计里剥离掉，让指标更接近纯噪声。",
                "",
                "2. Gyro RMS",
                "",
                "- 公式: `RMS = sqrt((1 / N) * sum((w_i')^2))`",
                "- 报告中的 `Gyro RMS` 单位为 `deg/s`，来源于去均值后的时域序列。",
                "",
                "3. Allan variance / Allan deviation",
                "",
                "- 先按聚合时间 `tau = m * Ts` 把序列分段求均值，得到 `y_k`。",
                "- Allan variance: `AVAR(tau) = (1 / 2) * mean((y_(k+1) - y_k)^2)`",
                "- Allan deviation: `ADEV(tau) = sqrt(AVAR(tau))`",
                "- 报告中的 `Allan 最小值` 即各个 `tau` 下 `ADEV(tau)` 的最小点。",
                "",
                "4. ARW 提取",
                "",
                "- 在 Allan deviation 曲线上计算相邻对数坐标点的斜率。",
                "- 选取斜率最接近 `-1/2` 的区段，视为白噪声主导区域。",
                "- 在该区段按 `ARW = ADEV(tau) * sqrt(tau) * 60` 估算，单位为 `deg/sqrt(hr)`。",
            ]
        )

        consistency_sample_count = int(summary.get("same_model_consistency_sample_count", "0"))
        lines.extend(["", "## 5. 同型号一致性", ""])
        if consistency_sample_count >= 2:
            lines.extend(
                [
                    f"- 参与一致性统计的样本数: `{consistency_sample_count}`",
                    f"- 样本间三轴平均 RMS 均值: `{summary.get('same_model_rms_mean_deg_s', '0')}` deg/s",
                    f"- 样本间三轴平均 RMS 标准差: `{summary.get('same_model_rms_std_deg_s', '0')}` deg/s",
                    f"- 样本间三轴平均 RMS 变异系数: `{summary.get('same_model_rms_cv_pct', '0')}` %",
                    "- 计算方式: 先按 `imu_id` 分开提取各自三轴陀螺序列，分别算去均值 `RMS`，再对每颗 IMU 的三轴 RMS 取平均后做离散度统计。",
                ]
            )
        else:
            lines.extend(
                [
                    "- 当前文件可用于单次噪声评估，但不足以稳定给出同型号样本一致性。",
                    "- 若 BIN 中同时包含多颗 IMU 的原始包，或后续支持多文件联合分析，可继续扩展该项。",
                ]
            )

        lines.extend(
            [
                "",
                "## 6. CSV 字段说明",
                "",
                "本次会输出两类 CSV:",
                "",
                f"- 对比 CSV `{primary_csv_name}`: 以 `poll_count` 为横坐标，每行代表一次 poll，各 IMU 的值为该轮次内多个 FIFO 包换算后的平均值。",
                f"- 独立 IMU CSV: 每个文件只保留单颗 IMU 的逐包数据，字段名使用具体 IMU 名字前缀，且不再包含 `poll_count` 列。",
                "",
                "### 6.1 独立 IMU CSV 字段",
                "",
                "| 列名 | 含义 | 获取或计算方式 |",
                "| --- | --- | --- |",
                "| `packet_index` | 全文件内的全局包序号 | 按解析顺序从 1 递增 |",
                "| `42688A_poll_timestamp_us` | `42688A` 这颗 IMU 对应 poll 轮次的 MCU 微秒时间戳 | 固件用 `TIM2 1MHz` 自由运行计数器在写 BIN 时记录 |",
                "| `42688A_packet_index_in_capture` | 当前包在本次 FIFO 记录块中的位置 | 同一轮 poll 的 FIFO 数据内从 1 递增 |",
                "| `42688A_packet_size` | 单个 FIFO 包字节数 | 固件从 IMU 原始读数结构里直接记录 |",
                "| `42688A_packet_timestamp_u16` | FIFO 包尾部自带的 16 位时间戳 | 从包内 `0x0E~0x0F` 字节提取，若包长不足则记 0 |",
                "| `42688A_packet_timestamp_continuous` | 展开的连续 FIFO 时间戳 | 在 PC 端基于 `42688A_packet_timestamp_u16` 做 16 位回绕展开 |",
                "| `42688A_fifo_header` | FIFO 包头字节 | 原始包第 0 字节 |",
                "| `42688A_raw_accel_x/y/z` | 加速度三轴原始 LSB | 分别从包内 `0x01~0x06` 按 big-endian 有符号 16 位解析 |",
                "| `42688A_raw_gyro_x/y/z` | 角速度三轴原始 LSB | 分别从包内 `0x07~0x0C` 按 big-endian 有符号 16 位解析 |",
                "| `42688A_temp_raw` | 温度原始值 | 包内 `0x0D` 的有符号 8 位值 |",
                "| `42688A_accel_x/y/z_mps2` | 加速度物理值，单位 `m/s^2` | `42688A_raw_accel * (16 / 2^15) * 9.80665` |",
                "| `42688A_gyro_x/y/z_deg_s` | 角速度物理值，单位 `deg/s` | `42688A_raw_gyro * (2000 / 2^15)` |",
                "| `42688A_temp_c` | 温度物理值，单位 `°C` | `42688A_temp_raw / 2.07 + 25.0` |",
                "",
                "### 6.2 对比 CSV 字段",
                "",
                "| 列名 | 含义 | 获取或计算方式 |",
                "| --- | --- | --- |",
                "| `poll_count` | 全局 poll 轮次序号，也是多颗 IMU 统一对齐的横坐标 | 固件在每轮 `poll` 开始时自增一次，同一轮内所有 IMU 共用该值 |",
                "| `42688A_accel_x/y/z_mps2` | `42688A` 在该 poll 轮次下的加速度平均值 | 对该轮次内 `42688A` 的多个 FIFO 包先换算成物理单位，再按轴求平均 |",
                "| `42688A_gyro_x/y/z_deg_s` | `42688A` 在该 poll 轮次下的角速度平均值 | 对该轮次内 `42688A` 的多个 FIFO 包先换算成 `deg/s`，再按轴求平均 |",
                "| `42688A_temp_c` | `42688A` 在该 poll 轮次下的温度平均值 | 对该轮次内 `42688A` 的多个 FIFO 包先换算成 `°C` 后求平均 |",
                "",
                "说明:",
                "- 其它 IMU 如 `42688B`、`45686A`、`45686B` 的字段命名方式与 `42688A` 完全相同，只是前缀替换为对应 IMU 名字。",
                "- 报告中的噪声、ARW 和 Allan 指标，默认基于“本页指标默认针对”的那颗 IMU 的独立 CSV 数据计算。",
            ]
        )

        destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return

    lines = [
        "# 数据分析报告",
        "",
        f"- 源文件: `{source}`",
        "",
        "## 摘要",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_real_imu_bin_file(test_key: str, source: Path, output_dir: Path | None = None) -> tuple[Path, Path, Path, dict[str, str]]:
    source = source.resolve()
    if not source.exists():
        raise FileNotFoundError(source)

    if output_dir is None:
        output_dir = source.parent
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_dir / f"{source.stem}_{test_key}.csv"
    md_path = output_dir / f"{source.stem}_{test_key}_analysis.md"
    legacy_packet_csv_path = output_dir / f"{source.stem}_{test_key}_packets.csv"
    legacy_summary_csv_path = output_dir / f"{source.stem}_{test_key}_summary.csv"

    legacy_packet_csv_path.unlink(missing_ok=True)
    legacy_summary_csv_path.unlink(missing_ok=True)
    payload = source.read_bytes()
    parsed = parse_real_imu_bin(payload)
    raw_rows = _build_raw_packet_rows(parsed.raw_captures)
    _write_per_imu_packet_csvs(raw_rows, source, output_dir)
    csv_path = _write_poll_comparison_csv(raw_rows, source, output_dir)

    raw_capture_by_imu: dict[int, list[RawImuCapture]] = {}
    for capture in parsed.raw_captures:
        raw_capture_by_imu.setdefault(capture.imu_id, []).append(capture)
    primary_imu_id = min(raw_capture_by_imu) if raw_capture_by_imu else 0
    primary_captures = raw_capture_by_imu.get(primary_imu_id, parsed.raw_captures)
    frames = parsed.frames if parsed.frames else _build_synthetic_frames_from_raw_captures(primary_captures)
    summary = analyze_real_imu_frames(test_key, frames, primary_captures)
    summary["primary_imu_name"] = _imu_name_from_id(primary_imu_id) if primary_imu_id else "unknown"
    summary["comparison_csv"] = csv_path.name
    summary["per_imu_csvs"] = ", ".join(
        f"{source.stem}_{_imu_name_from_id(imu_id)}.csv" for imu_id in sorted(raw_capture_by_imu)
    )
    summary["per_imu_report"] = _analyze_arw_per_imu(parsed.raw_captures) if test_key == "arw" else {}
    _write_markdown_report(md_path, source, summary)
    return csv_path, csv_path, md_path, summary


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
