from __future__ import annotations

import argparse
import ctypes
import csv
import math
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
REAL_IMU_RAW_RECORD_HEADER = struct.Struct("<HHBBHIQ")
REAL_IMU_RAW_MAGIC_HEAD = 0x55AB
ICM42688_ACCEL_SCALER_MPS2 = 16.0 / float(1 << 15) * 9.80665
ICM42688_GYRO_SCALER_RAD_S = 2000.0 / float(1 << 15) * math.pi / 180.0
ICM42688_TEMP_SCALER_LSB_PER_C = 2.07
ICM42688_TEMP_OFFSET_C = 25.0
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
    imu_index: int
    seq: int
    timestamp_us: int
    packet_size: int
    fifo_byte_count: int
    payload: bytes


@dataclass(frozen=True)
class ParsedRealImuBin:
    frames: list[RealImuFrame]
    raw_captures: list[RawImuCapture]


def _raw_accel_to_mps2(raw_value: int) -> float:
    return float(raw_value) * ICM42688_ACCEL_SCALER_MPS2


def _raw_gyro_to_rad_s(raw_value: int) -> float:
    return float(raw_value) * ICM42688_GYRO_SCALER_RAD_S


def _raw_temp_to_celsius(raw_value: int) -> float:
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
    status: str


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
    for line in response.splitlines():
        stripped = line.strip()
        if not stripped.startswith("STATUS "):
            continue

        parts: dict[str, str] = {}
        for token in stripped[7:].split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            parts[key] = value

        return DeviceStatus(
            mode=parts.get("mode", "unknown"),
            sd=parts.get("sd", "unknown"),
            test=parts.get("test", "unknown"),
        )

    raise ValueError("STATUS line not found")


def parse_imu_probe_response(response: str) -> list[ImuProbeResult]:
    results: list[ImuProbeResult] = []

    for line in response.splitlines():
        stripped = line.strip()
        if not stripped.startswith("IMU_PROBE "):
            continue

        parts: dict[str, str] = {}
        for token in stripped[10:].split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            parts[key] = value

        imu_text = parts.get("imu")
        if imu_text is None:
            continue

        results.append(
            ImuProbeResult(
                imu=int(imu_text),
                name=parts.get("name", "unknown"),
                status=parts.get("status", "unknown"),
            )
        )

    if not results:
        raise ValueError("IMU_PROBE line not found")

    return results


def parse_noise_prepare_response(response: str) -> NoisePrepareResult:
    status_text = "unknown"

    for line in response.splitlines():
        stripped = line.strip()
        if stripped.startswith("RESULT cmd=noise_test_prepare "):
            for token in stripped.split():
                if token.startswith("status="):
                    status_text = token.split("=", 1)[1]

    for line in response.splitlines():
        stripped = line.strip()
        if not stripped.startswith("NOISE_TEST "):
            continue

        parts: dict[str, str] = {}
        for token in stripped[11:].split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            parts[key] = value

        return NoisePrepareResult(
            detected=int(parts.get("detected", "0")),
            duration_s=int(parts.get("duration_s", "0")),
            recording=int(parts.get("recording", "0")),
            status=status_text,
        )

    raise ValueError("NOISE_TEST line not found")


def parse_noise_status_response(response: str) -> NoiseStatusResult:
    status_text = "unknown"

    for line in response.splitlines():
        stripped = line.strip()
        if stripped.startswith("RESULT cmd=noise_test_status "):
            for token in stripped.split():
                if token.startswith("status="):
                    status_text = token.split("=", 1)[1]

    for line in response.splitlines():
        stripped = line.strip()
        if not stripped.startswith("NOISE_TEST_STATUS "):
            continue

        parts: dict[str, str] = {}
        for token in stripped[18:].split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            parts[key] = value

        return NoiseStatusResult(
            recording=int(parts.get("recording", "0")),
            frames=int(parts.get("frames", "0")),
            duration_s=int(parts.get("duration_s", "0")),
            file=parts.get("file", ""),
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
                total_size,
                imu_index,
                packet_size,
                fifo_byte_count,
                seq,
                timestamp_us,
            ) = REAL_IMU_RAW_RECORD_HEADER.unpack_from(payload, offset)

            if total_size < REAL_IMU_RAW_RECORD_HEADER.size + 2:
                raise ValueError(f"invalid raw capture size {total_size} at offset {offset}")
            if offset + total_size > len(payload):
                raise ValueError(f"truncated raw capture payload at offset {offset}")

            expected_fifo_bytes = total_size - REAL_IMU_RAW_RECORD_HEADER.size - 2
            if fifo_byte_count != expected_fifo_bytes:
                raise ValueError(
                    f"raw capture size mismatch at offset {offset}: {fifo_byte_count} != {expected_fifo_bytes}"
                )

            crc16 = struct.unpack_from("<H", payload, offset + total_size - 2)[0]
            calc_crc = _crc16_ccitt(payload[offset + 2 : offset + total_size - 2])
            if crc16 != calc_crc:
                raise ValueError(
                    f"invalid raw capture crc at offset {offset}: 0x{crc16:04X} != 0x{calc_crc:04X}"
                )

            raw_captures.append(
                RawImuCapture(
                    imu_index=imu_index,
                    seq=seq,
                    timestamp_us=timestamp_us,
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
    return parse_real_imu_bin(payload).frames


def iter_real_imu_raw_captures(payload: bytes) -> list[RawImuCapture]:
    return parse_real_imu_bin(payload).raw_captures


def decode_real_imu_bin_to_csv(source: Path, destination: Path) -> int:
    payload = source.read_bytes()
    parsed = parse_real_imu_bin(payload)
    frames = parsed.frames
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
                    _format_float(_raw_accel_to_mps2(frame.accel_x)),
                    _format_float(_raw_accel_to_mps2(frame.accel_y)),
                    _format_float(_raw_accel_to_mps2(frame.accel_z)),
                    _format_float(_raw_gyro_to_rad_s(frame.gyro_x)),
                    _format_float(_raw_gyro_to_rad_s(frame.gyro_y)),
                    _format_float(_raw_gyro_to_rad_s(frame.gyro_z)),
                    _format_float(_raw_temp_to_celsius(frame.temp_raw)),
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


def _packet_timestamp_u16(packet: bytes) -> int:
    if len(packet) < 16:
        return 0
    return (packet[14] << 8) | packet[15]


def _packet_hex(packet: bytes) -> str:
    return packet.hex(" ").upper()


def decode_real_imu_raw_packets_to_csv(source: Path, destination: Path) -> int:
    payload = source.read_bytes()
    captures = iter_real_imu_raw_captures(payload)
    destination.parent.mkdir(parents=True, exist_ok=True)

    wrap_count_by_imu: dict[int, int] = {}
    last_timestamp_by_imu: dict[int, int] = {}
    global_packet_index = 0

    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "packet_index",
                "capture_index",
                "frame_seq",
                "imu_index",
                "read_timestamp_us",
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
                "packet_hex",
            ]
        )

        for capture_index, capture in enumerate(captures, start=1):
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
                packet_timestamp = _packet_timestamp_u16(packet)
                last_timestamp = last_timestamp_by_imu.get(capture.imu_index)
                wrap_count = wrap_count_by_imu.get(capture.imu_index, 0)
                if last_timestamp is not None and packet_timestamp < last_timestamp:
                    wrap_count += 1
                    wrap_count_by_imu[capture.imu_index] = wrap_count
                else:
                    wrap_count_by_imu.setdefault(capture.imu_index, wrap_count)
                last_timestamp_by_imu[capture.imu_index] = packet_timestamp
                packet_timestamp_continuous = wrap_count_by_imu[capture.imu_index] * 65536 + packet_timestamp

                fifo_header = packet[0] if packet else 0
                accel_x = int.from_bytes(packet[1:3], byteorder="big", signed=True) if len(packet) >= 3 else 0
                accel_y = int.from_bytes(packet[3:5], byteorder="big", signed=True) if len(packet) >= 5 else 0
                accel_z = int.from_bytes(packet[5:7], byteorder="big", signed=True) if len(packet) >= 7 else 0
                gyro_x = int.from_bytes(packet[7:9], byteorder="big", signed=True) if len(packet) >= 9 else 0
                gyro_y = int.from_bytes(packet[9:11], byteorder="big", signed=True) if len(packet) >= 11 else 0
                gyro_z = int.from_bytes(packet[11:13], byteorder="big", signed=True) if len(packet) >= 13 else 0
                temp_raw = int.from_bytes(packet[13:14], byteorder="big", signed=True) if len(packet) >= 14 else 0

                writer.writerow(
                    [
                        global_packet_index,
                        capture_index,
                        capture.seq,
                        capture.imu_index,
                        capture.timestamp_us,
                        packet_index + 1,
                        capture.packet_size,
                        packet_timestamp,
                        packet_timestamp_continuous,
                        fifo_header,
                        accel_x,
                        accel_y,
                        accel_z,
                        gyro_x,
                        gyro_y,
                        gyro_z,
                        temp_raw,
                        _packet_hex(packet),
                    ]
                )

    return global_packet_index


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


def analyze_real_imu_frames(test_key: str, frames: list[RealImuFrame]) -> dict[str, str]:
    if not frames:
        raise ValueError("no frames found")

    acc_x = [_raw_accel_to_mps2(frame.accel_x) for frame in frames]
    acc_y = [_raw_accel_to_mps2(frame.accel_y) for frame in frames]
    acc_z = [_raw_accel_to_mps2(frame.accel_z) for frame in frames]
    gyro_x = [_raw_gyro_to_rad_s(frame.gyro_x) for frame in frames]
    gyro_y = [_raw_gyro_to_rad_s(frame.gyro_y) for frame in frames]
    gyro_z = [_raw_gyro_to_rad_s(frame.gyro_z) for frame in frames]
    temp_c = [_raw_temp_to_celsius(frame.temp_raw) for frame in frames]
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
        common.update(
            {
                "gyro_rms_x_rad_s": f"{_rms(gyro_x):.3f}",
                "gyro_rms_y_rad_s": f"{_rms(gyro_y):.3f}",
                "gyro_rms_z_rad_s": f"{_rms(gyro_z):.3f}",
                "acc_rms_x_mps2": f"{_rms(acc_x):.3f}",
                "acc_rms_y_mps2": f"{_rms(acc_y):.3f}",
                "acc_rms_z_mps2": f"{_rms(acc_z):.3f}",
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


def _write_markdown_report(destination: Path, source: Path, summary: dict[str, str]) -> None:
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
    packet_csv_path = output_dir / f"{source.stem}_{test_key}_packets.csv"
    md_path = output_dir / f"{source.stem}_{test_key}_analysis.md"

    decode_real_imu_bin_to_csv(source, csv_path)
    decode_real_imu_raw_packets_to_csv(source, packet_csv_path)
    frames = iter_real_imu_frames(source.read_bytes())
    summary = analyze_real_imu_frames(test_key, frames)
    _write_markdown_report(md_path, source, summary)
    return csv_path, packet_csv_path, md_path, summary


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
    port, response = send_cdc_command(args.port, "noise_test_start", timeout=args.timeout)
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
        summary_destination = Path(args.output).resolve()
    else:
        summary_destination = source.with_suffix(".csv")
    packet_destination = summary_destination.with_name(f"{summary_destination.stem}_packets.csv")

    frame_count = decode_real_imu_bin_to_csv(source, summary_destination)
    packet_count = decode_real_imu_raw_packets_to_csv(source, packet_destination)
    print(f"decoded csv: {summary_destination}")
    print(f"decoded packet csv: {packet_destination}")
    print(f"frames: {frame_count}")
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

    parser_noise_start = subparsers.add_parser("noise-start", help="start 10s imu noise recording")
    parser_noise_start.add_argument("--port", help="CDC serial port, e.g. COM78")
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
