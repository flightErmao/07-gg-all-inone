"""
067.BIN 第45686条数据异常 - 自动化验证脚本

本脚本按优先级从高到低依次验证：
1. 解码程序自身是否有 bug（CRC校验、帧边界对齐、字节序）
2. MCU写入SD卡是否出错（连续性检查、帧间距分析、截断检测）
3. 传感器数据是否本身异常（量程溢出、突变检测、温度异常）

用法:
    python verify_067_bin.py <path_to_067.BIN> [--target-record 45686]
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

# ---------- 常量（与 imu_usb_tool.py 保持一致）----------

REAL_IMU_RAW_RECORD_HEADER = struct.Struct("<HHHQIH")  # 20 bytes
REAL_IMU_RAW_MAGIC_HEAD = 0x55AB
REAL_IMU_FRAME_STRUCT = struct.Struct("<HBQHBHhhhhhhhH")  # 30 bytes
REAL_IMU_FRAME_MAGIC_HEAD = 0x55AA

ICM42688_ACCEL_SCALER_MPS2 = 16.0 / float(1 << 15) * 9.80665
ICM42688_GYRO_SCALER_RAD_S = 2000.0 / float(1 << 15) * math.pi / 180.0
ICM45686_ACCEL_SCALER_MPS2 = 16.0 / float(1 << 15) * 9.80665
ICM45686_GYRO_SCALER_RAD_S = 2000.0 / float(1 << 15) * math.pi / 180.0

IMU_ID_TO_NAME = {1: "42688_A", 2: "42688_B", 3: "45686_A", 4: "45686_B"}

# 16g 量程下 int16 最大值
ACCEL_RAW_MAX = 32767
ACCEL_RAW_MIN = -32768
GYRO_RAW_MAX = 32767
GYRO_RAW_MIN = -32768


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


def _imu_uses_little_endian(imu_id: int) -> bool:
    # Firmware configures FIFO sensor data as big-endian for both IMU families.
    return False


def _packet_s16(packet: bytes, start: int, imu_id: int) -> int:
    if len(packet) < start + 2:
        return 0
    byteorder = "little" if _imu_uses_little_endian(imu_id) else "big"
    return int.from_bytes(packet[start:start + 2], byteorder=byteorder, signed=True)


# ---------- 数据结构 ----------

@dataclass
class RecordInfo:
    """一条 BIN 文件中的记录"""
    index: int  # 从1开始的记录序号
    offset: int  # 文件偏移
    record_type: str  # "frame" or "raw_capture"
    size: int
    timestamp_us: int = 0
    poll_count: int = 0
    imu_id: int = 0
    packet_size: int = 0
    fifo_byte_count: int = 0
    packet_count: int = 0
    crc_ok: bool = True
    crc_expected: int = 0
    crc_actual: int = 0
    # 传感器数据摘要
    accel_values: list = field(default_factory=list)  # [(ax, ay, az), ...]
    gyro_values: list = field(default_factory=list)  # [(gx, gy, gz), ...]
    raw_bytes: bytes = b""


@dataclass
class VerifyResult:
    total_records: int = 0
    crc_errors: list = field(default_factory=list)
    timestamp_gaps: list = field(default_factory=list)
    timestamp_reversals: list = field(default_factory=list)
    accel_saturations: list = field(default_factory=list)
    gyro_saturations: list = field(default_factory=list)
    accel_spikes: list = field(default_factory=list)
    gyro_spikes: list = field(default_factory=list)
    truncated_records: list = field(default_factory=list)
    unknown_magic: list = field(default_factory=list)
    poll_count_gaps: list = field(default_factory=list)
    target_record: RecordInfo | None = None


# ---------- 核心扫描函数 ----------

def scan_bin_file(source: Path, target_record_index: int = 45686) -> VerifyResult:
    result = VerifyResult()
    file_size = source.stat().st_size
    print(f"文件: {source}")
    print(f"文件大小: {file_size:,} bytes ({file_size / 1024 / 1024:.2f} MB)")

    prev_records_by_imu: dict[int, RecordInfo] = {}
    prev_poll_count_by_imu: dict[int, int] = {}
    record_index = 0
    last_report_time = time.monotonic()

    with source.open("rb") as f:
        offset = 0
        while True:
            magic_bytes = f.read(2)
            if not magic_bytes:
                break
            if len(magic_bytes) != 2:
                result.truncated_records.append({"offset": offset, "detail": "文件末尾不足2字节"})
                break

            magic = struct.unpack("<H", magic_bytes)[0]

            # --- 0x55AA: summary frame ---
            if magic == REAL_IMU_FRAME_MAGIC_HEAD:
                frame_size = REAL_IMU_FRAME_STRUCT.size
                rest = f.read(frame_size - 2)
                if len(rest) != frame_size - 2:
                    result.truncated_records.append({"offset": offset, "detail": f"summary frame 截断, 需要{frame_size - 2}字节, 只有{len(rest)}字节"})
                    break
                record_bytes = magic_bytes + rest
                record_index += 1

                fields = REAL_IMU_FRAME_STRUCT.unpack(record_bytes)
                crc16_in_file = fields[-1]
                calc_crc = _crc16_ccitt(record_bytes[2:-2])

                rec = RecordInfo(
                    index=record_index,
                    offset=offset,
                    record_type="frame",
                    size=frame_size,
                    timestamp_us=fields[2],
                    crc_ok=(crc16_in_file == calc_crc),
                    crc_expected=calc_crc,
                    crc_actual=crc16_in_file,
                    raw_bytes=record_bytes,
                )

                if not rec.crc_ok:
                    result.crc_errors.append(rec)

                if record_index == target_record_index:
                    result.target_record = rec

                offset += frame_size
                continue

            # --- 0x55AB: raw capture ---
            if magic == REAL_IMU_RAW_MAGIC_HEAD:
                header_rest = f.read(REAL_IMU_RAW_RECORD_HEADER.size - 2)
                if len(header_rest) != REAL_IMU_RAW_RECORD_HEADER.size - 2:
                    result.truncated_records.append({"offset": offset, "detail": "raw capture header 截断"})
                    break

                header = magic_bytes + header_rest
                (_magic, fifo_byte_count, packet_size, timestamp_us, poll_count, imu_id) = \
                    REAL_IMU_RAW_RECORD_HEADER.unpack(header)

                if packet_size <= 0 or packet_size > 256:
                    result.truncated_records.append({
                        "offset": offset,
                        "detail": f"无效 packet_size={packet_size}"
                    })
                    break

                payload = f.read(fifo_byte_count)
                if len(payload) != fifo_byte_count:
                    result.truncated_records.append({
                        "offset": offset,
                        "detail": f"raw capture payload 截断, 需要{fifo_byte_count}字节, 只有{len(payload)}字节"
                    })
                    break

                crc_bytes = f.read(2)
                if len(crc_bytes) != 2:
                    result.truncated_records.append({"offset": offset, "detail": "raw capture CRC 截断"})
                    break

                crc16_in_file = struct.unpack("<H", crc_bytes)[0]
                calc_crc = _crc16_ccitt(header[2:] + payload)
                total_size = REAL_IMU_RAW_RECORD_HEADER.size + fifo_byte_count + 2

                record_index += 1
                pkt_count = fifo_byte_count // packet_size if packet_size > 0 else 0

                # 解析每个 packet 的加速度/陀螺仪原始值
                accel_vals = []
                gyro_vals = []
                for pi in range(pkt_count):
                    pkt_start = pi * packet_size
                    pkt = payload[pkt_start:pkt_start + packet_size]
                    ax = _packet_s16(pkt, 1, imu_id)
                    ay = _packet_s16(pkt, 3, imu_id)
                    az = _packet_s16(pkt, 5, imu_id)
                    gx = _packet_s16(pkt, 7, imu_id)
                    gy = _packet_s16(pkt, 9, imu_id)
                    gz = _packet_s16(pkt, 11, imu_id)
                    accel_vals.append((ax, ay, az))
                    gyro_vals.append((gx, gy, gz))

                rec = RecordInfo(
                    index=record_index,
                    offset=offset,
                    record_type="raw_capture",
                    size=total_size,
                    timestamp_us=timestamp_us,
                    poll_count=poll_count,
                    imu_id=imu_id,
                    packet_size=packet_size,
                    fifo_byte_count=fifo_byte_count,
                    packet_count=pkt_count,
                    crc_ok=(crc16_in_file == calc_crc),
                    crc_expected=calc_crc,
                    crc_actual=crc16_in_file,
                    accel_values=accel_vals,
                    gyro_values=gyro_vals,
                    raw_bytes=magic_bytes + header_rest + payload + crc_bytes,
                )

                # === 检查1: CRC ===
                if not rec.crc_ok:
                    result.crc_errors.append(rec)

                # === 检查2: 时间戳连续性 ===
                prev_rec = prev_records_by_imu.get(imu_id)
                if prev_rec is not None and prev_rec.timestamp_us > 0 and timestamp_us > 0:
                    dt_us = timestamp_us - prev_rec.timestamp_us
                    if dt_us < 0:
                        result.timestamp_reversals.append({
                            "record_index": record_index,
                            "imu_id": imu_id,
                            "offset": offset,
                            "prev_ts": prev_rec.timestamp_us,
                            "curr_ts": timestamp_us,
                            "delta_us": dt_us,
                        })
                    # 对于 1kHz/1.6kHz 采样, 正常 poll 间隔约 2ms, 超过 50ms 算异常间隔
                    elif dt_us > 50_000:
                        result.timestamp_gaps.append({
                            "record_index": record_index,
                            "imu_id": imu_id,
                            "offset": offset,
                            "prev_ts": prev_rec.timestamp_us,
                            "curr_ts": timestamp_us,
                            "delta_us": dt_us,
                            "delta_ms": dt_us / 1000.0,
                        })

                # === 检查3: poll_count 连续性 ===
                prev_poll = prev_poll_count_by_imu.get(imu_id)
                if prev_poll is not None:
                    expected_next = prev_poll + 1
                    if poll_count != expected_next and poll_count != 0:
                        result.poll_count_gaps.append({
                            "record_index": record_index,
                            "imu_id": imu_id,
                            "offset": offset,
                            "expected": expected_next,
                            "actual": poll_count,
                            "gap": poll_count - prev_poll,
                        })
                prev_poll_count_by_imu[imu_id] = poll_count

                # === 检查4: 量程饱和 ===
                for (ax, ay, az) in accel_vals:
                    if abs(ax) >= ACCEL_RAW_MAX - 10 or abs(ay) >= ACCEL_RAW_MAX - 10 or abs(az) >= ACCEL_RAW_MAX - 10:
                        result.accel_saturations.append({
                            "record_index": record_index,
                            "imu_id": imu_id,
                            "offset": offset,
                            "values": (ax, ay, az),
                        })
                        break

                for (gx, gy, gz) in gyro_vals:
                    if abs(gx) >= GYRO_RAW_MAX - 10 or abs(gy) >= GYRO_RAW_MAX - 10 or abs(gz) >= GYRO_RAW_MAX - 10:
                        result.gyro_saturations.append({
                            "record_index": record_index,
                            "imu_id": imu_id,
                            "offset": offset,
                            "values": (gx, gy, gz),
                        })
                        break

                # === 检查5: 突变检测（与前一条记录比较）===
                if prev_rec is not None and prev_rec.accel_values and accel_vals:
                    prev_a = prev_rec.accel_values[-1]
                    curr_a = accel_vals[0]
                    for axis in range(3):
                        delta = abs(curr_a[axis] - prev_a[axis])
                        # 突变阈值：超过量程的 10%（约 3276 LSB, 对应 ~1.6g）
                        if delta > 3276:
                            result.accel_spikes.append({
                                "record_index": record_index,
                                "imu_id": imu_id,
                                "offset": offset,
                                "axis": ["x", "y", "z"][axis],
                                "prev": prev_a[axis],
                                "curr": curr_a[axis],
                                "delta": delta,
                            })
                            break

                if prev_rec is not None and prev_rec.gyro_values and gyro_vals:
                    prev_g = prev_rec.gyro_values[-1]
                    curr_g = gyro_vals[0]
                    for axis in range(3):
                        delta = abs(curr_g[axis] - prev_g[axis])
                        if delta > 3276:
                            result.gyro_spikes.append({
                                "record_index": record_index,
                                "imu_id": imu_id,
                                "offset": offset,
                                "axis": ["x", "y", "z"][axis],
                                "prev": prev_g[axis],
                                "curr": curr_g[axis],
                                "delta": delta,
                            })
                            break

                prev_records_by_imu[imu_id] = rec

                if record_index == target_record_index:
                    result.target_record = rec

                offset += total_size

                # 进度报告
                now = time.monotonic()
                if now - last_report_time > 2.0:
                    pct = offset * 100.0 / file_size if file_size > 0 else 0
                    print(f"  扫描进度: {pct:.1f}% ({record_index} 条记录, offset={offset:,})")
                    last_report_time = now
                continue

            # --- 未知 magic ---
            result.unknown_magic.append({"offset": offset, "magic": f"0x{magic:04X}"})
            # 尝试逐字节前进寻找下一个有效 magic
            f.seek(offset + 1)
            offset += 1

    result.total_records = record_index
    return result


# ---------- 报告输出 ----------

def print_report(result: VerifyResult, target_index: int) -> None:
    print("\n" + "=" * 70)
    print(f"扫描完成: 共 {result.total_records} 条记录")
    print("=" * 70)

    # 目标记录详情
    print(f"\n--- 目标记录 #{target_index} 详情 ---")
    if result.target_record is None:
        print(f"  [警告] 未找到第 {target_index} 条记录！文件可能不足 {target_index} 条。")
    else:
        rec = result.target_record
        print(f"  类型: {rec.record_type}")
        print(f"  文件偏移: {rec.offset} (0x{rec.offset:X})")
        print(f"  记录大小: {rec.size} bytes")
        print(f"  时间戳: {rec.timestamp_us} us")
        print(f"  CRC 校验: {'通过' if rec.crc_ok else f'失败 (文件中=0x{rec.crc_actual:04X}, 计算=0x{rec.crc_expected:04X})'}")
        if rec.record_type == "raw_capture":
            print(f"  IMU ID: {rec.imu_id} ({IMU_ID_TO_NAME.get(rec.imu_id, '未知')})")
            print(f"  poll_count: {rec.poll_count}")
            print(f"  packet_size: {rec.packet_size}")
            print(f"  fifo_byte_count: {rec.fifo_byte_count}")
            print(f"  packet_count: {rec.packet_count}")
            if rec.accel_values:
                print(f"  加速度原始值(首包): ax={rec.accel_values[0][0]}, ay={rec.accel_values[0][1]}, az={rec.accel_values[0][2]}")
            if rec.gyro_values:
                print(f"  陀螺仪原始值(首包): gx={rec.gyro_values[0][0]}, gy={rec.gyro_values[0][1]}, gz={rec.gyro_values[0][2]}")
        print(f"  原始字节(前64字节): {rec.raw_bytes[:64].hex(' ').upper()}")

    # 1. CRC 错误
    print(f"\n--- 验证1: CRC 校验错误 ---")
    if not result.crc_errors:
        print("  [通过] 所有记录 CRC 均正确")
    else:
        print(f"  [异常] 发现 {len(result.crc_errors)} 条 CRC 错误:")
        for rec in result.crc_errors[:20]:
            print(f"    记录#{rec.index} offset={rec.offset} 类型={rec.record_type} "
                  f"文件CRC=0x{rec.crc_actual:04X} 计算CRC=0x{rec.crc_expected:04X} "
                  f"imu_id={rec.imu_id}")
        if len(result.crc_errors) > 20:
            print(f"    ... 还有 {len(result.crc_errors) - 20} 条")

    # 2. 未知 magic
    print(f"\n--- 验证2: 未知帧头 ---")
    if not result.unknown_magic:
        print("  [通过] 所有帧头均正确 (0x55AA 或 0x55AB)")
    else:
        print(f"  [异常] 发现 {len(result.unknown_magic)} 处未知帧头:")
        for item in result.unknown_magic[:10]:
            print(f"    offset={item['offset']} magic={item['magic']}")

    # 3. 截断记录
    print(f"\n--- 验证3: 截断/不完整记录 ---")
    if not result.truncated_records:
        print("  [通过] 无截断记录")
    else:
        print(f"  [异常] 发现 {len(result.truncated_records)} 处截断:")
        for item in result.truncated_records:
            print(f"    offset={item['offset']} 详情: {item['detail']}")

    # 4. 时间戳逆转
    print(f"\n--- 验证4: 时间戳逆转 ---")
    if not result.timestamp_reversals:
        print("  [通过] 时间戳单调递增")
    else:
        print(f"  [异常] 发现 {len(result.timestamp_reversals)} 处时间戳逆转:")
        for item in result.timestamp_reversals[:10]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} "
                  f"prev={item['prev_ts']} curr={item['curr_ts']} delta={item['delta_us']}us")

    # 5. 时间戳异常间隔
    print(f"\n--- 验证5: 时间戳异常间隔 (>50ms) ---")
    if not result.timestamp_gaps:
        print("  [通过] 无异常间隔")
    else:
        print(f"  [异常] 发现 {len(result.timestamp_gaps)} 处异常间隔:")
        for item in result.timestamp_gaps[:20]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} "
                  f"间隔={item['delta_ms']:.1f}ms offset={item['offset']}")

    # 6. poll_count 不连续
    print(f"\n--- 验证6: poll_count 连续性 ---")
    if not result.poll_count_gaps:
        print("  [通过] poll_count 连续")
    else:
        print(f"  [异常] 发现 {len(result.poll_count_gaps)} 处 poll_count 跳跃:")
        for item in result.poll_count_gaps[:20]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} "
                  f"期望={item['expected']} 实际={item['actual']} 跳跃={item['gap']}")

    # 7. 加速度饱和
    print(f"\n--- 验证7: 加速度量程饱和 ---")
    if not result.accel_saturations:
        print("  [通过] 无加速度饱和")
    else:
        print(f"  [异常] 发现 {len(result.accel_saturations)} 处加速度饱和:")
        for item in result.accel_saturations[:10]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} 值={item['values']}")

    # 8. 陀螺仪饱和
    print(f"\n--- 验证8: 陀螺仪量程饱和 ---")
    if not result.gyro_saturations:
        print("  [通过] 无陀螺仪饱和")
    else:
        print(f"  [异常] 发现 {len(result.gyro_saturations)} 处陀螺仪饱和:")
        for item in result.gyro_saturations[:10]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} 值={item['values']}")

    # 9. 加速度突变
    print(f"\n--- 验证9: 加速度突变 (>10%量程) ---")
    if not result.accel_spikes:
        print("  [通过] 无加速度突变")
    else:
        print(f"  [异常] 发现 {len(result.accel_spikes)} 处加速度突变:")
        for item in result.accel_spikes[:20]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} "
                  f"轴={item['axis']} 前={item['prev']} 后={item['curr']} 变化={item['delta']}")

    # 10. 陀螺仪突变
    print(f"\n--- 验证10: 陀螺仪突变 (>10%量程) ---")
    if not result.gyro_spikes:
        print("  [通过] 无陀螺仪突变")
    else:
        print(f"  [异常] 发现 {len(result.gyro_spikes)} 处陀螺仪突变:")
        for item in result.gyro_spikes[:20]:
            print(f"    记录#{item['record_index']} IMU{item['imu_id']} "
                  f"轴={item['axis']} 前={item['prev']} 后={item['curr']} 变化={item['delta']}")

    # 综合结论
    print("\n" + "=" * 70)
    print("综合结论:")
    issues = []
    if result.crc_errors:
        issues.append(f"CRC错误({len(result.crc_errors)}处)")
    if result.unknown_magic:
        issues.append(f"未知帧头({len(result.unknown_magic)}处)")
    if result.truncated_records:
        issues.append(f"截断记录({len(result.truncated_records)}处)")
    if result.timestamp_reversals:
        issues.append(f"时间逆转({len(result.timestamp_reversals)}处)")
    if result.poll_count_gaps:
        issues.append(f"poll_count跳跃({len(result.poll_count_gaps)}处)")
    if result.timestamp_gaps:
        issues.append(f"时间间隔异常({len(result.timestamp_gaps)}处)")
    if result.accel_saturations:
        issues.append(f"加速度饱和({len(result.accel_saturations)}处)")
    if result.gyro_saturations:
        issues.append(f"陀螺仪饱和({len(result.gyro_saturations)}处)")
    if result.accel_spikes:
        issues.append(f"加速度突变({len(result.accel_spikes)}处)")
    if result.gyro_spikes:
        issues.append(f"陀螺仪突变({len(result.gyro_spikes)}处)")

    if not issues:
        print("  所有验证项均通过，数据文件无明显异常。")
    else:
        print(f"  发现以下异常: {', '.join(issues)}")

    # 异常归因建议
    if result.crc_errors or result.unknown_magic or result.truncated_records:
        print("\n  >>> 优先排查方向: MCU写入SD卡错误或解码程序bug")
        if result.crc_errors:
            target_rec = result.target_record
            if target_rec and not target_rec.crc_ok:
                print(f"  >>> 目标记录#{target_index} CRC校验失败，大概率是SD卡写入时数据损坏")
            else:
                print("  >>> 目标记录CRC正确，但文件中其他记录有CRC错误，可能是SD卡写入不稳定")
    elif result.accel_saturations or result.gyro_saturations:
        print("\n  >>> 优先排查方向: 传感器量程溢出（真实冲击或传感器故障）")
    elif result.accel_spikes or result.gyro_spikes:
        print("\n  >>> 优先排查方向: 传感器数据突变（可能是外部冲击、接触不良或传感器故障）")
    elif result.timestamp_gaps or result.poll_count_gaps:
        print("\n  >>> 优先排查方向: MCU采集线程被抢占或SD卡写入阻塞导致数据丢失")

    print("=" * 70)


def main() -> int:
    parser = argparse.ArgumentParser(description="067.BIN 数据异常验证工具")
    parser.add_argument("input", help="BIN 文件路径")
    parser.add_argument("--target-record", type=int, default=45686, help="目标记录序号 (默认 45686)")
    args = parser.parse_args()

    source = Path(args.input).resolve()
    if not source.exists():
        print(f"文件不存在: {source}")
        return 1

    print(f"开始扫描，目标记录: #{args.target_record}")
    start = time.monotonic()
    result = scan_bin_file(source, args.target_record)
    elapsed = time.monotonic() - start
    print(f"扫描耗时: {elapsed:.1f}s")

    print_report(result, args.target_record)
    return 0


if __name__ == "__main__":
    sys.exit(main())
