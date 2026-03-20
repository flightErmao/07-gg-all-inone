#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁力计-SD距离影响评估工具

功能：
1. 打开两个串口：
   - shell串口：给MCU发送命令
   - data串口：接收MCU经uart1输出的磁力计原始数据
2. 提供开始测试、结束测试、导出评估结果按钮
3. 根据未校准原始磁力计数据，评估SD工作时对航向角的最大偏移
"""

from __future__ import annotations

import csv
import json
import math
import queue
import re
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from statistics import mean, pvariance
from typing import List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports


BASELINE_WINDOW_MS = 3000
MIN_BASELINE_SAMPLES = 5
MIN_ACTIVE_SAMPLES = 5
APP_TITLE = "磁力计-SD影响评估工具"


@dataclass
class MagSample:
    timestamp_ms: int
    x: float
    y: float
    z: float


@dataclass
class MagEvent:
    timestamp_ms: int
    name: str
    detail: str = ""


@dataclass
class CycleSummary:
    cycle_index: int
    start_timestamp_ms: int
    stop_timestamp_ms: int
    baseline_sample_count: int
    active_sample_count: int
    baseline_heading_deg: float
    baseline_field_magnitude_lsb: float
    baseline_field_magnitude_ut: float
    baseline_field_magnitude_mgs: float
    max_abs_heading_offset_deg: float
    mean_abs_heading_offset_deg: float
    p95_abs_heading_offset_deg: float
    max_field_delta_lsb: float
    mean_field_delta_lsb: float
    max_offset_timestamp_ms: int


@dataclass
class PhaseStats:
    phase_name: str
    start_timestamp_ms: int
    stop_timestamp_ms: int
    sample_count: int
    mean_x: float
    var_x: float
    mean_y: float
    var_y: float
    mean_z: float
    var_z: float
    mean_field_magnitude_lsb: float
    var_field_magnitude_lsb: float
    mean_heading_deg: float
    var_heading_deg: float
    mean_heading_offset_deg: float
    var_heading_offset_deg: float
    max_abs_heading_offset_deg: float
    p95_abs_heading_offset_deg: float


def normalize_heading_deg(angle_deg: float) -> float:
    angle_deg %= 360.0
    return angle_deg if angle_deg >= 0.0 else angle_deg + 360.0


def vector_heading_deg(x: float, y: float) -> float:
    return normalize_heading_deg(math.degrees(math.atan2(y, x)))


def heading_offset_deg(ref_x: float, ref_y: float, cur_x: float, cur_y: float) -> float:
    det = ref_x * cur_y - ref_y * cur_x
    dot = ref_x * cur_x + ref_y * cur_y
    return math.degrees(math.atan2(det, dot))


def percentile(values: List[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lower = math.floor(pos)
    upper = math.ceil(pos)
    if lower == upper:
        return ordered[lower]
    weight = pos - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def stable_mean(values: List[float]) -> float:
    return mean(values) if values else 0.0


def stable_variance(values: List[float]) -> float:
    return pvariance(values) if len(values) >= 2 else 0.0


def sanitize_name(text: str) -> str:
    text = text.strip()
    if not text:
        return "unknown_distance"
    return re.sub(r"[^0-9A-Za-z_\-\u4e00-\u9fff]+", "_", text)


def pair_test_cycles(events: List[MagEvent]) -> List[Tuple[MagEvent, MagEvent]]:
    pairs: List[Tuple[MagEvent, MagEvent]] = []
    current_start: Optional[MagEvent] = None

    for event in sorted(events, key=lambda item: item.timestamp_ms):
        if event.name == "TEST_START":
            current_start = event
        elif event.name == "TEST_STOP" and current_start is not None:
            if event.timestamp_ms > current_start.timestamp_ms:
                pairs.append((current_start, event))
                current_start = None

    return pairs


def is_local_button_event(event: MagEvent) -> bool:
    return "source=local_button" in event.detail


def dedupe_test_events(events: List[MagEvent], tolerance_ms: int = 1500) -> List[MagEvent]:
    deduped: List[MagEvent] = []

    for event in sorted(events, key=lambda item: item.timestamp_ms):
        if event.name not in {"TEST_START", "TEST_STOP"}:
            deduped.append(event)
            continue

        if deduped:
            prev = deduped[-1]
            if (
                prev.name == event.name
                and abs(prev.timestamp_ms - event.timestamp_ms) <= tolerance_ms
            ):
                prev_is_local = is_local_button_event(prev)
                curr_is_local = is_local_button_event(event)
                if prev_is_local and not curr_is_local:
                    deduped[-1] = event
                continue

        deduped.append(event)

    return deduped


def moving_average(values: List[float], window: int) -> List[float]:
    if not values:
        return []
    window = max(1, window)
    smoothed = []
    acc = 0.0
    for index, value in enumerate(values):
        acc += value
        if index >= window:
            acc -= values[index - window]
        current_window = min(index + 1, window)
        smoothed.append(acc / current_window)
    return smoothed


def find_sustained_run(values: List[float], threshold: float, run_len: int, start_index: int = 0, above: bool = True) -> Optional[int]:
    hit_count = 0
    candidate = None
    for index in range(start_index, len(values)):
        condition = values[index] >= threshold if above else values[index] <= threshold
        if condition:
            hit_count += 1
            if candidate is None:
                candidate = index
            if hit_count >= run_len:
                return candidate
        else:
            hit_count = 0
            candidate = None
    return None


def build_phase_stats(phase_name: str, rows: List[dict], ref_x: float, ref_y: float) -> PhaseStats:
    if not rows:
        raise ValueError(f"{phase_name} 没有样本，无法统计。")

    xs = [row["x"] for row in rows]
    ys = [row["y"] for row in rows]
    zs = [row["z"] for row in rows]
    mags = [row["field_magnitude_lsb"] for row in rows]
    headings = [row["heading_deg"] for row in rows]
    offsets = [heading_offset_deg(ref_x, ref_y, row["x"], row["y"]) for row in rows]
    abs_offsets = [abs(value) for value in offsets]

    return PhaseStats(
        phase_name=phase_name,
        start_timestamp_ms=rows[0]["timestamp_ms"],
        stop_timestamp_ms=rows[-1]["timestamp_ms"],
        sample_count=len(rows),
        mean_x=stable_mean(xs),
        var_x=stable_variance(xs),
        mean_y=stable_mean(ys),
        var_y=stable_variance(ys),
        mean_z=stable_mean(zs),
        var_z=stable_variance(zs),
        mean_field_magnitude_lsb=stable_mean(mags),
        var_field_magnitude_lsb=stable_variance(mags),
        mean_heading_deg=stable_mean(headings),
        var_heading_deg=stable_variance(headings),
        mean_heading_offset_deg=stable_mean(offsets),
        var_heading_offset_deg=stable_variance(offsets),
        max_abs_heading_offset_deg=max(abs_offsets),
        p95_abs_heading_offset_deg=percentile(abs_offsets, 0.95),
    )


def find_representative_heading_row(rows: List[dict], ref_x: float, ref_y: float) -> Tuple[dict, float]:
    if not rows:
        raise ValueError("阶段样本为空，无法提取代表航向。")

    best_row = rows[0]
    best_offset = heading_offset_deg(ref_x, ref_y, best_row["x"], best_row["y"])
    best_abs = abs(best_offset)

    for row in rows[1:]:
        offset = heading_offset_deg(ref_x, ref_y, row["x"], row["y"])
        if abs(offset) > best_abs:
            best_row = row
            best_offset = offset
            best_abs = abs(offset)

    return best_row, best_offset


def detect_three_phases(active_samples: List[MagSample]) -> Tuple[int, int, dict]:
    if len(active_samples) < 15:
        raise ValueError("测试段样本过少，无法自动拆分三阶段。")

    seed_count = max(5, min(20, len(active_samples) // 6))
    seed_samples = active_samples[:seed_count]
    ref_x = stable_mean([sample.x for sample in seed_samples])
    ref_y = stable_mean([sample.y for sample in seed_samples])
    ref_z = stable_mean([sample.z for sample in seed_samples])

    deltas = [
        math.sqrt((sample.x - ref_x) ** 2 + (sample.y - ref_y) ** 2 + (sample.z - ref_z) ** 2)
        for sample in active_samples
    ]
    smooth_window = max(3, min(9, len(active_samples) // 20 if len(active_samples) >= 20 else 3))
    smoothed = moving_average(deltas, smooth_window)

    quiet_segment = smoothed[:seed_count]
    quiet_mean = stable_mean(quiet_segment)
    quiet_std = math.sqrt(stable_variance(quiet_segment))
    quiet_p95 = percentile(quiet_segment, 0.95)

    high_threshold = max(quiet_mean + 6.0 * quiet_std, quiet_p95 * 2.5, 1.0)
    low_threshold = max(quiet_mean + 2.5 * quiet_std, quiet_p95 * 1.3, 0.5)
    sustain_len = max(3, min(8, seed_count // 2 + 1))

    phase2_start = find_sustained_run(smoothed, high_threshold, sustain_len, start_index=seed_count, above=True)

    if phase2_start is None:
        peak_index = max(range(len(smoothed)), key=lambda index: smoothed[index])
        if smoothed[peak_index] <= high_threshold:
            raise ValueError("未检测到明显的 SD 干扰阶段，无法自动拆分三阶段。")
        phase2_start = max(seed_count, peak_index - sustain_len // 2)

    phase3_start = find_sustained_run(smoothed, low_threshold, sustain_len, start_index=phase2_start + sustain_len, above=False)

    if phase3_start is None:
        tail_count = max(seed_count, len(smoothed) // 5)
        tail_start_limit = max(phase2_start + sustain_len, len(smoothed) - tail_count)
        best_index = None
        best_score = None
        for index in range(tail_start_limit, len(smoothed)):
            score = abs(smoothed[index] - quiet_mean)
            if best_score is None or score < best_score:
                best_score = score
                best_index = index
        if best_index is None or best_index <= phase2_start:
            raise ValueError("未检测到 SD 干扰消失后的恢复阶段，无法自动拆分三阶段。")
        phase3_start = best_index

    phase2_start = min(max(1, phase2_start), len(active_samples) - 2)
    phase3_start = min(max(phase2_start + 1, phase3_start), len(active_samples) - 1)

    if phase2_start < 3 or (phase3_start - phase2_start) < 3 or (len(active_samples) - phase3_start) < 3:
        raise ValueError("自动分段后的三阶段样本不足，请延长三个阶段的采样时间。")

    detection_info = {
        "seed_count": seed_count,
        "smooth_window": smooth_window,
        "quiet_mean_lsb": quiet_mean,
        "quiet_std_lsb": quiet_std,
        "quiet_p95_lsb": quiet_p95,
        "high_threshold_lsb": high_threshold,
        "low_threshold_lsb": low_threshold,
        "sustain_len": sustain_len,
    }

    return phase2_start, phase3_start, detection_info


def analyze_cycle(cycle_index: int, start_event: MagEvent, stop_event: MagEvent, samples: List[MagSample]) -> dict:
    all_before_start = [sample for sample in samples if sample.timestamp_ms < start_event.timestamp_ms]
    baseline_window_start = start_event.timestamp_ms - BASELINE_WINDOW_MS
    baseline_samples = [
        sample for sample in all_before_start if sample.timestamp_ms >= baseline_window_start
    ]
    if len(baseline_samples) < MIN_BASELINE_SAMPLES:
        baseline_samples = all_before_start

    active_samples = [
        sample for sample in samples
        if start_event.timestamp_ms <= sample.timestamp_ms <= stop_event.timestamp_ms
    ]

    if len(baseline_samples) < MIN_BASELINE_SAMPLES:
        raise ValueError(
            f"第 {cycle_index} 轮测试开始前的基线样本不足，至少需要 {MIN_BASELINE_SAMPLES} 个样本。"
        )
    if len(active_samples) < MIN_ACTIVE_SAMPLES:
        raise ValueError(
            f"第 {cycle_index} 轮测试进行中的样本不足，至少需要 {MIN_ACTIVE_SAMPLES} 个样本。"
        )

    ref_x = mean(sample.x for sample in baseline_samples)
    ref_y = mean(sample.y for sample in baseline_samples)
    ref_z = mean(sample.z for sample in baseline_samples)

    ref_xy_norm = math.hypot(ref_x, ref_y)
    if ref_xy_norm < 1e-6:
        raise ValueError(f"第 {cycle_index} 轮测试基线水平磁场过小，无法计算航向偏移。")

    baseline_heading = vector_heading_deg(ref_x, ref_y)
    baseline_field_mag = math.sqrt(ref_x * ref_x + ref_y * ref_y + ref_z * ref_z)

    series = []
    abs_offsets = []
    field_deltas = []

    for sample in active_samples:
        offset = heading_offset_deg(ref_x, ref_y, sample.x, sample.y)
        abs_offset = abs(offset)
        field_delta = math.sqrt(
            (sample.x - ref_x) ** 2 +
            (sample.y - ref_y) ** 2 +
            (sample.z - ref_z) ** 2
        )
        field_mag = math.sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z)
        series.append(
            {
                "timestamp_ms": sample.timestamp_ms,
                "relative_time_s": (sample.timestamp_ms - start_event.timestamp_ms) / 1000.0,
                "x": sample.x,
                "y": sample.y,
                "z": sample.z,
                "heading_deg": vector_heading_deg(sample.x, sample.y),
                "heading_offset_deg": offset,
                "abs_heading_offset_deg": abs_offset,
                "field_magnitude_lsb": field_mag,
                "field_delta_lsb": field_delta,
                "field_magnitude_delta_from_baseline_lsb": field_mag - baseline_field_mag,
            }
        )
        abs_offsets.append(abs_offset)
        field_deltas.append(field_delta)

    phase_detection = {"mode": "auto", "message": ""}
    phase_boundaries = {}

    try:
        phase2_start_idx, phase3_start_idx, detection_info = detect_three_phases(active_samples)
        phase1_rows = series[:phase2_start_idx]
        phase2_rows = series[phase2_start_idx:phase3_start_idx]
        phase3_rows = series[phase3_start_idx:]

        phase1_ref_x = stable_mean([row["x"] for row in phase1_rows])
        phase1_ref_y = stable_mean([row["y"] for row in phase1_rows])
        if math.hypot(phase1_ref_x, phase1_ref_y) < 1e-6:
            raise ValueError(f"第 {cycle_index} 轮阶段1水平磁场过小，无法计算三阶段偏移。")

        phase_stats = {
            "phase_1_idle": build_phase_stats("阶段1-SD未工作", phase1_rows, phase1_ref_x, phase1_ref_y),
            "phase_2_active": build_phase_stats("阶段2-SD工作中", phase2_rows, phase1_ref_x, phase1_ref_y),
            "phase_3_recover": build_phase_stats("阶段3-SD停止后", phase3_rows, phase1_ref_x, phase1_ref_y),
        }

        phase2_rep_row, phase2_rep_offset = find_representative_heading_row(phase2_rows, phase1_ref_x, phase1_ref_y)
        phase3_rep_row, phase3_rep_offset = find_representative_heading_row(phase3_rows, phase1_ref_x, phase1_ref_y)

        transition_metrics = {
            "before_to_active_mean_heading_offset_deg": phase_stats["phase_2_active"].mean_heading_offset_deg,
            "before_to_active_max_abs_heading_offset_deg": phase_stats["phase_2_active"].max_abs_heading_offset_deg,
            "active_to_recover_mean_heading_change_deg": phase_stats["phase_3_recover"].mean_heading_offset_deg - phase_stats["phase_2_active"].mean_heading_offset_deg,
            "before_to_recover_mean_heading_offset_deg": phase_stats["phase_3_recover"].mean_heading_offset_deg,
            "before_to_recover_max_abs_heading_offset_deg": phase_stats["phase_3_recover"].max_abs_heading_offset_deg,
        }

        phase_boundaries = {
            "phase_2_start_timestamp_ms": active_samples[phase2_start_idx].timestamp_ms,
            "phase_3_start_timestamp_ms": active_samples[phase3_start_idx].timestamp_ms,
            "phase_2_start_relative_s": series[phase2_start_idx]["relative_time_s"],
            "phase_3_start_relative_s": series[phase3_start_idx]["relative_time_s"],
        }
        schematic = {
            "mode": "auto",
            "before_heading_deg": phase_stats["phase_1_idle"].mean_heading_deg,
            "during_heading_deg": phase2_rep_row["heading_deg"],
            "after_heading_deg": phase3_rep_row["heading_deg"],
            "during_offset_deg": phase2_rep_offset,
            "after_offset_deg": phase3_rep_offset,
            "during_source": "阶段2最大绝对偏移对应航向",
            "after_source": "阶段3最大绝对偏移对应航向",
        }
    except ValueError as exc:
        phase_detection = {"mode": "fallback", "message": str(exc)}
        detection_info = {
            "seed_count": None,
            "smooth_window": None,
            "quiet_mean_lsb": None,
            "quiet_std_lsb": None,
            "quiet_p95_lsb": None,
            "high_threshold_lsb": None,
            "low_threshold_lsb": None,
            "sustain_len": None,
        }
        phase_stats = {
            "phase_all": build_phase_stats("整段测试-未检测到明显SD干扰", series, ref_x, ref_y),
        }
        transition_metrics = {
            "before_to_active_mean_heading_offset_deg": phase_stats["phase_all"].mean_heading_offset_deg,
            "before_to_active_max_abs_heading_offset_deg": phase_stats["phase_all"].max_abs_heading_offset_deg,
            "active_to_recover_mean_heading_change_deg": 0.0,
            "before_to_recover_mean_heading_offset_deg": phase_stats["phase_all"].mean_heading_offset_deg,
            "before_to_recover_max_abs_heading_offset_deg": phase_stats["phase_all"].max_abs_heading_offset_deg,
        }
        phase_all_rep_row, phase_all_rep_offset = find_representative_heading_row(series, ref_x, ref_y)
        phase_boundaries = {
            "phase_2_start_timestamp_ms": None,
            "phase_3_start_timestamp_ms": None,
            "phase_2_start_relative_s": None,
            "phase_3_start_relative_s": None,
        }
        schematic = {
            "mode": "fallback",
            "before_heading_deg": baseline_heading,
            "during_heading_deg": phase_all_rep_row["heading_deg"],
            "after_heading_deg": series[-1]["heading_deg"],
            "during_offset_deg": phase_all_rep_offset,
            "after_offset_deg": heading_offset_deg(ref_x, ref_y, series[-1]["x"], series[-1]["y"]),
            "during_source": "整段测试最大绝对偏移对应航向",
            "after_source": "测试结束前最后一个样本航向",
        }

    max_item = max(series, key=lambda item: item["abs_heading_offset_deg"])
    summary = CycleSummary(
        cycle_index=cycle_index,
        start_timestamp_ms=start_event.timestamp_ms,
        stop_timestamp_ms=stop_event.timestamp_ms,
        baseline_sample_count=len(baseline_samples),
        active_sample_count=len(active_samples),
        baseline_heading_deg=baseline_heading,
        baseline_field_magnitude_lsb=baseline_field_mag,
        baseline_field_magnitude_ut=0.0,
        baseline_field_magnitude_mgs=0.0,
        max_abs_heading_offset_deg=max_item["abs_heading_offset_deg"],
        mean_abs_heading_offset_deg=mean(abs_offsets),
        p95_abs_heading_offset_deg=percentile(abs_offsets, 0.95),
        max_field_delta_lsb=max(field_deltas),
        mean_field_delta_lsb=mean(field_deltas),
        max_offset_timestamp_ms=max_item["timestamp_ms"],
    )

    return {
        "summary": summary,
        "baseline": {
            "x": ref_x,
            "y": ref_y,
            "z": ref_z,
            "heading_deg": baseline_heading,
            "field_magnitude_lsb": baseline_field_mag,
        },
        "series": series,
        "phase_boundaries": phase_boundaries,
        "phase_detection": phase_detection,
        "detection_info": detection_info,
        "phase_stats": phase_stats,
        "transition_metrics": transition_metrics,
        "schematic": schematic,
    }


def ensure_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt


class MagSdEvalApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("980x760")

        self.shell_port_var = tk.StringVar()
        self.shell_baud_var = tk.StringVar(value="115200")
        self.data_port_var = tk.StringVar()
        self.data_baud_var = tk.StringVar(value="115200")
        self.distance_var = tk.StringVar(value="10mm")
        self.note_var = tk.StringVar()
        self.status_var = tk.StringVar(value="未连接")

        self.shell_serial: Optional[serial.Serial] = None
        self.data_serial: Optional[serial.Serial] = None
        self.reader_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.line_queue: "queue.Queue[str]" = queue.Queue()
        self.after_job: Optional[str] = None
        self.is_closing = False

        self.samples: List[MagSample] = []
        self.events: List[MagEvent] = []
        self.local_control_events: List[MagEvent] = []
        self.config_snapshot = {}
        self.last_export_dir: Optional[Path] = None

        self._build_ui()
        self.refresh_ports()
        self.after_job = self.root.after(100, self._drain_line_queue)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        config_frame = ttk.LabelFrame(main, text="连接配置", padding=12)
        config_frame.pack(fill=tk.X)

        ttk.Label(config_frame, text="Shell串口").grid(row=0, column=0, sticky=tk.W, padx=4, pady=4)
        self.shell_port_combo = ttk.Combobox(config_frame, textvariable=self.shell_port_var, width=18, state="readonly")
        self.shell_port_combo.grid(row=0, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="Shell波特率").grid(row=0, column=2, sticky=tk.W, padx=4, pady=4)
        ttk.Entry(config_frame, textvariable=self.shell_baud_var, width=12).grid(row=0, column=3, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="Data串口").grid(row=1, column=0, sticky=tk.W, padx=4, pady=4)
        self.data_port_combo = ttk.Combobox(config_frame, textvariable=self.data_port_var, width=18, state="readonly")
        self.data_port_combo.grid(row=1, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="Data波特率").grid(row=1, column=2, sticky=tk.W, padx=4, pady=4)
        ttk.Entry(config_frame, textvariable=self.data_baud_var, width=12).grid(row=1, column=3, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="距离记录").grid(row=2, column=0, sticky=tk.W, padx=4, pady=4)
        ttk.Entry(config_frame, textvariable=self.distance_var, width=20).grid(row=2, column=1, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="备注").grid(row=2, column=2, sticky=tk.W, padx=4, pady=4)
        ttk.Entry(config_frame, textvariable=self.note_var, width=32).grid(row=2, column=3, sticky=tk.W, padx=4, pady=4)

        ttk.Button(config_frame, text="刷新串口", command=self.refresh_ports).grid(row=0, column=4, padx=8, pady=4)
        ttk.Button(config_frame, text="连接", command=self.connect_serials).grid(row=1, column=4, padx=8, pady=4)
        ttk.Button(config_frame, text="断开", command=self.disconnect_serials).grid(row=2, column=4, padx=8, pady=4)

        control_frame = ttk.LabelFrame(main, text="测试控制", padding=12)
        control_frame.pack(fill=tk.X, pady=(12, 0))

        ttk.Button(control_frame, text="开始测试", command=self.start_test, width=16).pack(side=tk.LEFT, padx=6)
        ttk.Button(control_frame, text="结束测试", command=self.stop_test, width=16).pack(side=tk.LEFT, padx=6)
        ttk.Button(control_frame, text="导出评估结果", command=self.export_results, width=18).pack(side=tk.LEFT, padx=6)

        ttk.Label(control_frame, textvariable=self.status_var).pack(side=tk.RIGHT, padx=6)

        hint_frame = ttk.LabelFrame(main, text="说明", padding=12)
        hint_frame.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(
            hint_frame,
            justify=tk.LEFT,
            text=(
                "1. 连接后工具会自动发送 mag_sd_stream_start，让 MCU 在 uart1 输出 MAG_RAW 数据。\n"
                "2. 开始测试前建议保持设备静止 3 秒以上，作为基线段。\n"
                "3. 点击开始测试后，请依次经历：SD未工作 -> SD工作中 -> SD停止后恢复。\n"
                "4. 脚本会在 TEST_START ~ TEST_STOP 内自动识别三个阶段；若未识别到明显干扰，也会导出一般性报告。"
            ),
        ).pack(anchor=tk.W)

        log_frame = ttk.LabelFrame(main, text="运行日志", padding=12)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(12, 0))

        self.log_text = tk.Text(log_frame, wrap=tk.WORD, height=24)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.shell_port_combo["values"] = ports
        self.data_port_combo["values"] = ports

        if ports:
            if not self.shell_port_var.get():
                self.shell_port_var.set(ports[0])
            if not self.data_port_var.get():
                self.data_port_var.set(ports[min(1, len(ports) - 1)])

        self.log(f"已刷新串口列表，共 {len(ports)} 个串口。")

    def connect_serials(self) -> None:
        if self.shell_serial or self.data_serial:
            self.log("串口已经连接，如需重连请先断开。")
            return

        try:
            shell_port = self.shell_port_var.get().strip()
            data_port = self.data_port_var.get().strip()
            shell_baud = int(self.shell_baud_var.get().strip())
            data_baud = int(self.data_baud_var.get().strip())

            if not shell_port or not data_port:
                raise ValueError("请先选择 shell 串口和 data 串口。")

            self.shell_serial = serial.Serial(shell_port, shell_baud, timeout=0.5)
            self.data_serial = serial.Serial(data_port, data_baud, timeout=0.2)
            self.stop_event.clear()
            self.samples.clear()
            self.events.clear()
            self.local_control_events.clear()
            self.config_snapshot = {}

            self.reader_thread = threading.Thread(target=self._data_reader_loop, daemon=True)
            self.reader_thread.start()

            self.send_shell_command("mag_sd_stream_start")
            self.status_var.set("已连接，数据流已开启")
            self.log(
                f"连接成功。shell={shell_port}@{shell_baud}，data={data_port}@{data_baud}，距离={self.distance_var.get().strip() or '未填写'}"
            )
        except Exception as exc:
            self.disconnect_serials()
            messagebox.showerror("连接失败", str(exc))

    def disconnect_serials(self, stop_stream: bool = True, wait_reader: bool = True) -> None:
        shell_serial = self.shell_serial
        data_serial = self.data_serial
        reader_thread = self.reader_thread

        if stop_stream and shell_serial:
            try:
                payload = b"mag_sd_stream_stop\n"
                shell_serial.write(payload)
                shell_serial.flush()
            except Exception:
                pass

        self.stop_event.set()

        if data_serial:
            try:
                data_serial.cancel_read()
            except Exception:
                pass
            try:
                data_serial.close()
            except Exception:
                pass

        if shell_serial:
            try:
                shell_serial.close()
            except Exception:
                pass

        if wait_reader and reader_thread and reader_thread.is_alive():
            reader_thread.join(timeout=0.2)

        self.reader_thread = None
        self.data_serial = None
        self.shell_serial = None
        self.status_var.set("未连接")
        if not self.is_closing:
            self.log("串口已断开。")

    def send_shell_command(self, command: str) -> None:
        if not self.shell_serial:
            raise RuntimeError("shell 串口未连接。")

        payload = (command.strip() + "\n").encode("utf-8")
        self.shell_serial.write(payload)
        self.shell_serial.flush()
        self.log(f"发送命令: {command}")

    def start_test(self) -> None:
        try:
            self.send_shell_command("mag_sd_test_start")
            self._record_local_control_event("TEST_START")
            self.status_var.set("测试中")
        except Exception as exc:
            messagebox.showerror("开始测试失败", str(exc))

    def stop_test(self) -> None:
        try:
            self.send_shell_command("mag_sd_test_stop")
            self._record_local_control_event("TEST_STOP")
            self.status_var.set("测试已结束，可导出")
        except Exception as exc:
            messagebox.showerror("结束测试失败", str(exc))

    def export_results(self) -> None:
        if not self.samples:
            messagebox.showwarning("无数据", "还没有接收到磁力计数据，无法导出。")
            return

        try:
            self._ensure_config_snapshot()
            export_dir = self._prepare_export_dir()
            effective_events = dedupe_test_events(self.events + self.local_control_events)
            cycles = pair_test_cycles(effective_events)
            if not cycles:
                raise ValueError("没有找到完整的 TEST_START / TEST_STOP 事件对，无法评估。")

            cycle_results = []
            for index, (start_event, stop_event) in enumerate(cycles, start=1):
                cycle_results.append(analyze_cycle(index, start_event, stop_event, self.samples))

            self._apply_config_to_cycle_results(cycle_results)
            self._write_raw_files(export_dir)
            self._write_cycle_outputs(export_dir, cycle_results)
            self._write_report(export_dir, cycle_results)

            overall_max = max(
                result["summary"].max_abs_heading_offset_deg for result in cycle_results
            )
            self.last_export_dir = export_dir
            self.status_var.set("导出完成")
            self.log(
                f"导出完成：{export_dir}，共 {len(cycle_results)} 轮测试，最大航向偏移 {overall_max:.3f}°"
            )
            messagebox.showinfo(
                "导出完成",
                f"结果已导出到：\n{export_dir}\n\n最大航向偏移：{overall_max:.3f}°",
            )
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc))

    def _prepare_export_dir(self) -> Path:
        default_root = Path(__file__).resolve().parent.parent / "data" / "sd_eval_sessions"
        default_root.mkdir(parents=True, exist_ok=True)

        distance_name = sanitize_name(self.distance_var.get())
        time_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        suggested_dir = default_root / f"{time_name}_{distance_name}"

        chosen = filedialog.askdirectory(
            title="选择导出目录（取消则使用默认目录）",
            initialdir=str(default_root),
        )
        export_dir = (Path(chosen) / suggested_dir.name) if chosen else suggested_dir
        export_dir.mkdir(parents=True, exist_ok=True)
        return export_dir

    def _write_raw_files(self, export_dir: Path) -> None:
        sample_path = export_dir / "raw_samples.csv"
        with sample_path.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(["timestamp_ms", "x_raw", "y_raw", "z_raw"])
            for sample in self.samples:
                writer.writerow([sample.timestamp_ms, sample.x, sample.y, sample.z])

        event_path = export_dir / "events.csv"
        with event_path.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(["timestamp_ms", "event_name", "detail"])
            for event in self.events:
                writer.writerow([event.timestamp_ms, event.name, event.detail])

        metadata = {
            "distance": self.distance_var.get().strip(),
            "note": self.note_var.get().strip(),
            "shell_port": self.shell_port_var.get().strip(),
            "shell_baud": self.shell_baud_var.get().strip(),
            "data_port": self.data_port_var.get().strip(),
            "data_baud": self.data_baud_var.get().strip(),
            "exported_at": datetime.now().isoformat(timespec="seconds"),
            "mag_config_snapshot": self.config_snapshot,
        }
        with (export_dir / "session_metadata.json").open("w", encoding="utf-8") as fp:
            json.dump(metadata, fp, ensure_ascii=False, indent=2)

    def _apply_config_to_cycle_results(self, cycle_results: List[dict]) -> None:
        config_lsb = self._safe_float(self.config_snapshot.get("lsb_to_ut"))
        if config_lsb <= 0.0:
            return

        for result in cycle_results:
            summary: CycleSummary = result["summary"]
            summary.baseline_field_magnitude_ut = summary.baseline_field_magnitude_lsb * config_lsb
            summary.baseline_field_magnitude_mgs = summary.baseline_field_magnitude_ut * 10.0

    def _record_local_control_event(self, event_name: str) -> None:
        timestamp_ms = self._estimate_current_sample_timestamp()
        event = MagEvent(
            timestamp_ms=timestamp_ms,
            name=event_name,
            detail="source=local_button",
        )
        self.local_control_events.append(event)
        self.log(f"记录本地事件: {event.name} @ {event.timestamp_ms} ms")

    def _estimate_current_sample_timestamp(self) -> int:
        if self.samples:
            return self.samples[-1].timestamp_ms
        if self.events:
            return max(event.timestamp_ms for event in self.events)
        if self.local_control_events:
            return max(event.timestamp_ms for event in self.local_control_events)
        return 0

    def _ensure_config_snapshot(self) -> None:
        if self.config_snapshot:
            return
        if not self.shell_serial or not self.data_serial:
            return

        try:
            self.send_shell_command("mag_sd_stream_start")
        except Exception:
            return

        deadline = time.time() + 1.0
        while time.time() < deadline and not self.config_snapshot:
            time.sleep(0.05)
            self._drain_line_queue_once()

        if self.config_snapshot:
            self.log("已补收到磁力计配置快照。")
        else:
            self.log("本次采集中未收到 MCU 上报的磁力计配置快照。")

    def _write_cycle_outputs(self, export_dir: Path, cycle_results: List[dict]) -> None:
        plt = ensure_matplotlib()
        config_lsb = self._safe_float(self.config_snapshot.get("lsb_to_ut"))

        summary_csv = export_dir / "evaluation_summary.csv"
        with summary_csv.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(list(asdict(cycle_results[0]["summary"]).keys()))
            for result in cycle_results:
                writer.writerow(list(asdict(result["summary"]).values()))

        phase_summary_csv = export_dir / "phase_summary.csv"
        with phase_summary_csv.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(["cycle_index"] + list(asdict(next(iter(cycle_results[0]["phase_stats"].values()))).keys()))
            for result in cycle_results:
                for phase in result["phase_stats"].values():
                    writer.writerow([result["summary"].cycle_index] + list(asdict(phase).values()))

        for result in cycle_results:
            cycle_idx = result["summary"].cycle_index
            series_path = export_dir / f"cycle_{cycle_idx:02d}_series.csv"
            with series_path.open("w", newline="", encoding="utf-8-sig") as fp:
                writer = csv.writer(fp)
                writer.writerow(
                    [
                        "timestamp_ms",
                        "relative_time_s",
                        "x_raw",
                        "y_raw",
                        "z_raw",
                        "heading_deg",
                        "heading_offset_deg",
                        "abs_heading_offset_deg",
                        "field_magnitude_lsb",
                        "field_delta_lsb",
                        "field_magnitude_delta_from_baseline_lsb",
                    ]
                )
                for row in result["series"]:
                    writer.writerow(
                        [
                            row["timestamp_ms"],
                            row["relative_time_s"],
                            row["x"],
                            row["y"],
                            row["z"],
                            row["heading_deg"],
                            row["heading_offset_deg"],
                            row["abs_heading_offset_deg"],
                            row["field_magnitude_lsb"],
                            row["field_delta_lsb"],
                            row["field_magnitude_delta_from_baseline_lsb"],
                        ]
                    )

            fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True)
            times = [row["relative_time_s"] for row in result["series"]]
            phase2_time = result["phase_boundaries"]["phase_2_start_relative_s"]
            phase3_time = result["phase_boundaries"]["phase_3_start_relative_s"]

            def add_phase_markers(ax):
                if phase2_time is not None:
                    ax.axvline(phase2_time, color="#E67E22", linestyle="--", linewidth=1.2, label="自动识别: SD启动")
                if phase3_time is not None:
                    ax.axvline(phase3_time, color="#27AE60", linestyle="--", linewidth=1.2, label="自动识别: SD停止后恢复")

            axes[0].plot(times, [row["x"] for row in result["series"]], label="X_raw", linewidth=1.5)
            axes[0].plot(times, [row["y"] for row in result["series"]], label="Y_raw", linewidth=1.5)
            axes[0].plot(times, [row["z"] for row in result["series"]], label="Z_raw", linewidth=1.5)
            add_phase_markers(axes[0])
            axes[0].set_ylabel("Raw LSB")
            axes[0].set_title(f"第 {cycle_idx} 轮测试原始磁力计数据")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(loc="best")

            if config_lsb > 0.0:
                x_ut = [row["x"] * config_lsb for row in result["series"]]
                y_ut = [row["y"] * config_lsb for row in result["series"]]
                z_ut = [row["z"] * config_lsb for row in result["series"]]
                axes[1].plot(times, x_ut, label="X_uT", linewidth=1.5)
                axes[1].plot(times, y_ut, label="Y_uT", linewidth=1.5)
                axes[1].plot(times, z_ut, label="Z_uT", linewidth=1.5)
                add_phase_markers(axes[1])
                axes[1].set_ylabel("uT")
                axes[1].set_title("各轴磁场强度（按当前配置换算为 uT）")
                axes[1].text(
                    0.01,
                    0.95,
                    f"换算尺度: 1 LSB = {config_lsb:.6f} uT",
                    transform=axes[1].transAxes,
                    fontsize=10,
                    verticalalignment="top",
                    bbox=dict(boxstyle="round", facecolor="#F8F9F9", alpha=0.85, edgecolor="#BFC9CA"),
                )
                axes[1].grid(True, alpha=0.3)
                axes[1].legend(loc="best")
                field_magnitude_ut = [row["field_magnitude_lsb"] * config_lsb for row in result["series"]]
                axes[2].plot(times, field_magnitude_ut, color="#2471A3", linewidth=1.8, label="Field Magnitude (uT)")
                axes[2].set_ylabel("uT")
                axes[2].set_title("磁场总强度随时间变化")
            else:
                axes[1].text(
                    0.5,
                    0.5,
                    "未获取到有效 LSB 配置，无法换算为 uT",
                    transform=axes[1].transAxes,
                    ha="center",
                    va="center",
                )
                axes[1].set_title("各轴磁场强度（uT）")
                axes[1].set_ylabel("uT")
                field_magnitude_ut = [row["field_magnitude_lsb"] for row in result["series"]]
                axes[2].plot(times, field_magnitude_ut, color="#2471A3", linewidth=1.8, label="Field Magnitude")
                axes[2].set_ylabel("LSB")
                axes[2].set_title("磁场总强度随时间变化")

            add_phase_markers(axes[2])
            axes[2].grid(True, alpha=0.3)
            axes[2].legend(loc="best")

            axes[3].plot(
                times,
                [row["heading_offset_deg"] for row in result["series"]],
                color="#C0392B",
                linewidth=1.8,
                label="Heading Offset",
            )
            axes[3].axhline(0.0, color="black", linewidth=1.0, linestyle="--")
            add_phase_markers(axes[3])
            axes[3].set_xlabel("Time From TEST_START (s)")
            axes[3].set_ylabel("Offset (deg)")
            axes[3].set_title(
                f"第 {cycle_idx} 轮航向偏移，最大绝对偏移 = {result['summary'].max_abs_heading_offset_deg:.3f}°"
            )
            axes[3].grid(True, alpha=0.3)
            axes[3].legend(loc="best")

            fig.tight_layout()
            fig.savefig(export_dir / f"cycle_{cycle_idx:02d}_heading_offset.png", dpi=200, bbox_inches="tight")
            plt.close(fig)

            schematic = result["schematic"]
            fig, ax = plt.subplots(figsize=(8, 8))
            segment_len = 1.0

            def heading_to_vec(heading_deg: float):
                rad = math.radians(heading_deg)
                return math.sin(rad), math.cos(rad)

            def next_point(point, heading_deg: float):
                dx, dy = heading_to_vec(heading_deg)
                return point[0] + segment_len * dx, point[1] + segment_len * dy

            p0 = (0.0, 0.0)
            p1 = next_point(p0, schematic["before_heading_deg"])
            p2 = next_point(p1, schematic["during_heading_deg"])
            p3 = next_point(p2, schematic["after_heading_deg"])

            points = [p0, p1, p2, p3]
            colors = ["#2471A3", "#C0392B", "#27AE60"]
            labels = [
                ("SD启动前", schematic["before_heading_deg"], p0, p1),
                ("SD工作中", schematic["during_heading_deg"], p1, p2),
                ("SD关闭后", schematic["after_heading_deg"], p2, p3),
            ]

            for index, (label, heading_deg, start_pt, end_pt) in enumerate(labels):
                ax.annotate(
                    "",
                    xy=end_pt,
                    xytext=start_pt,
                    arrowprops=dict(arrowstyle="->", lw=2.5, color=colors[index]),
                )
                mid_x = (start_pt[0] + end_pt[0]) / 2.0
                mid_y = (start_pt[1] + end_pt[1]) / 2.0
                extra = ""
                if label == "SD工作中":
                    extra = f"\n偏移={schematic['during_offset_deg']:.3f}°"
                elif label == "SD关闭后":
                    extra = f"\n偏移={schematic['after_offset_deg']:.3f}°"
                ax.text(
                    mid_x,
                    mid_y,
                    f"{label}\n航向={heading_deg:.3f}°{extra}",
                    fontsize=10,
                    ha="center",
                    va="bottom",
                    bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor=colors[index]),
                )

            ax.plot([p[0] for p in points], [p[1] for p in points], color="#7F8C8D", linewidth=1.0, alpha=0.6)
            ax.scatter([p[0] for p in points], [p[1] for p in points], color="black", s=24)
            ax.text(p0[0], p0[1], "返航起点", fontsize=10, ha="right", va="top")
            ax.text(p3[0], p3[1], "返航末端", fontsize=10, ha="left", va="bottom")
            ax.set_title(f"第 {cycle_idx} 轮返航S型航线示意图")
            ax.set_xlabel("East (+)")
            ax.set_ylabel("North (+)")
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.text(
                0.02,
                0.02,
                f"SD工作中代表航向来源: {schematic['during_source']}\nSD关闭后代表航向来源: {schematic['after_source']}",
                transform=ax.transAxes,
                fontsize=9,
                ha="left",
                va="bottom",
                bbox=dict(boxstyle="round", facecolor="#FDFEFE", alpha=0.9, edgecolor="#D5D8DC"),
            )
            fig.tight_layout()
            fig.savefig(export_dir / f"cycle_{cycle_idx:02d}_return_s_curve.png", dpi=200, bbox_inches="tight")
            plt.close(fig)

    def _write_report(self, export_dir: Path, cycle_results: List[dict]) -> None:
        overall_max = max(result["summary"].max_abs_heading_offset_deg for result in cycle_results)
        config = self.config_snapshot
        lsb_consistent = self._is_lsb_consistent(config)
        report_lines = [
            "# 磁力计-SD影响评估报告",
            "",
            f"- 导出时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"- 记录距离: {self.distance_var.get().strip() or '未填写'}",
            f"- 备注: {self.note_var.get().strip() or '无'}",
            f"- 测试轮数: {len(cycle_results)}",
            f"- 全部测试中的最大航向偏移: {overall_max:.3f}°",
            "",
            "## 磁力计配置快照",
            "",
        ]

        if config:
            report_lines.extend(
                [
                    f"- 驱动: {config.get('driver', 'unknown')}",
                    f"- 配置是否可用: {config.get('available', 'unknown')}",
                    f"- Chip ID: {config.get('chip_id', 'unknown')}",
                    f"- CTL_REG_ONE (0x0A): {config.get('ctl_reg_one', 'unknown')}",
                    f"- CTL_REG_TWO (0x0B): {config.get('ctl_reg_two', 'unknown')}",
                    f"- CTL_REG_THREE (0x0D): {config.get('ctl_reg_three', 'unknown')}",
                    f"- 量程: {config.get('range_g', 'unknown')} G",
                    f"- 输出速率: {config.get('odr_hz', 'unknown')} Hz",
                    f"- 配置LSB: {config.get('lsb_to_ut', 'unknown')} uT/LSB",
                    f"- 按量程推导LSB: {config.get('expected_lsb_to_ut', 'unknown')} uT/LSB",
                    f"- LSB/G: {config.get('lsb_per_g', 'unknown')}",
                    f"- 配置LSB与量程推导是否一致: {'是' if lsb_consistent else '否'}",
                    "",
                ]
            )
        else:
            report_lines.extend(
                [
                    "- 本次采集中未收到 MCU 上报的磁力计配置快照。",
                    "",
                ]
            )

        report_lines.extend(
            [
            "## 评估方法",
            "",
            "- 基线段取每轮 TEST_START 前最近 3 秒的原始磁力计数据；若不足，则退回使用该轮开始前的全部样本。",
            "- 测试段取 TEST_START 到 TEST_STOP 之间的数据。",
            "- 脚本会在测试段内自动识别三阶段：阶段1(SD未工作)、阶段2(SD工作中)、阶段3(SD停止后恢复)。",
            "- 三阶段识别依据是相对阶段1初始稳定段的磁场向量变化量，自动寻找“扰动抬升”和“扰动回落”两个分界点。",
            "- 以基线段 X/Y 平均向量作为整轮参考航向，以阶段1平均 X/Y 向量作为三阶段对比参考航向。",
            "- 本方法评估的是“同一姿态、同一环境、只切换 SD 是否工作”条件下的相对航向偏移，不代表绝对航向精度。",
            "",
            "## SD干扰识别计算方式",
            "",
            "- 在 TEST_START 到 TEST_STOP 的测试段内，先取最前面一小段稳定样本作为阶段1种子窗口。",
            "- 以该窗口的平均向量作为参考，计算每个时刻的磁场变化量：`delta = sqrt((x-ref_x)^2 + (y-ref_y)^2 + (z-ref_z)^2)`。",
            "- 对 `delta` 做滑动平均，抑制单点抖动，得到更平滑的扰动强度序列。",
            "- 进入判定阈值 `high_threshold` 由初始稳定段的均值、方差和 95 分位共同决定；当平滑后的 `delta` 连续多点高于该阈值时，判定为 `SD干扰进入`，也就是阶段2起点。",
            "- 退出判定阈值 `low_threshold` 同样由初始稳定段统计量推导；当平滑后的 `delta` 在阶段2之后连续多点回落到该阈值以下时，判定为 `SD干扰退出`，也就是阶段3起点。",
            "- 如果整段测试都没有出现足够明显且持续的扰动抬升/回落，脚本不会强行拆分三阶段，而是退化为“整段测试统计”报告。",
            "",
            "## 各轮结果",
            "",
            ]
        )

        for result in cycle_results:
            summary: CycleSummary = result["summary"]
            baseline = result["baseline"]
            phase_stats = result["phase_stats"]
            transition = result["transition_metrics"]
            phase_boundaries = result["phase_boundaries"]
            phase_detection = result["phase_detection"]
            detection_info = result["detection_info"]
            schematic = result["schematic"]
            report_lines.extend(
                [
                    f"### 第 {summary.cycle_index} 轮",
                    "",
                    f"- 时间戳范围: {summary.start_timestamp_ms} ms -> {summary.stop_timestamp_ms} ms",
                    f"- 基线样本数: {summary.baseline_sample_count}",
                    f"- 测试样本数: {summary.active_sample_count}",
                    f"- 基线平均磁场: X={self._axis_scalar_text(baseline['x'])}, Y={self._axis_scalar_text(baseline['y'])}, Z={self._axis_scalar_text(baseline['z'])}",
                    f"- 基线航向: {summary.baseline_heading_deg:.3f}°",
                    f"- 基线磁场强度: {self._field_scalar_text(summary.baseline_field_magnitude_lsb, include_lsb=False)} ({summary.baseline_field_magnitude_mgs:.3f} mGs)",
                    f"- 最大绝对航向偏移: {summary.max_abs_heading_offset_deg:.3f}°",
                    f"- 平均绝对航向偏移: {summary.mean_abs_heading_offset_deg:.3f}°",
                    f"- 95分位绝对航向偏移: {summary.p95_abs_heading_offset_deg:.3f}°",
                    f"- 最大磁场向量变化量: {self._field_scalar_text(summary.max_field_delta_lsb, include_lsb=False)}",
                    f"- 平均磁场向量变化量: {self._field_scalar_text(summary.mean_field_delta_lsb, include_lsb=False)}",
                    f"- 最大偏移出现时间戳: {summary.max_offset_timestamp_ms} ms",
                    "",
                    "#### 返航S型航线示意数据",
                    "",
                    f"- SD启动前代表航向角: {schematic['before_heading_deg']:.3f}°",
                    f"- SD工作中代表航向角: {schematic['during_heading_deg']:.3f}° ({schematic['during_source']}, 偏移={schematic['during_offset_deg']:.3f}°)",
                    f"- SD关闭后代表航向角: {schematic['after_heading_deg']:.3f}° ({schematic['after_source']}, 偏移={schematic['after_offset_deg']:.3f}°)",
                    f"- 航线示意图文件: cycle_{summary.cycle_index:02d}_return_s_curve.png",
                    "",
                    "#### SD干扰识别阈值",
                    "",
                    f"- 种子窗口样本数: {detection_info['seed_count'] if detection_info['seed_count'] is not None else '未使用'}",
                    f"- 平滑窗口长度: {detection_info['smooth_window'] if detection_info['smooth_window'] is not None else '未使用'}",
                    f"- 连续判定点数: {detection_info['sustain_len'] if detection_info['sustain_len'] is not None else '未使用'}",
                    f"- 初始稳定段 quiet_mean: {self._field_scalar_text(detection_info['quiet_mean_lsb'], include_lsb=False) if detection_info['quiet_mean_lsb'] is not None else '未使用'}",
                    f"- 初始稳定段 quiet_std: {self._field_scalar_text(detection_info['quiet_std_lsb'], include_lsb=False) if detection_info['quiet_std_lsb'] is not None else '未使用'}",
                    f"- 初始稳定段 quiet_p95: {self._field_scalar_text(detection_info['quiet_p95_lsb'], include_lsb=False) if detection_info['quiet_p95_lsb'] is not None else '未使用'}",
                    f"- 进入阈值 high_threshold: {self._field_scalar_text(detection_info['high_threshold_lsb'], include_lsb=False) if detection_info['high_threshold_lsb'] is not None else '未生成'}",
                    f"- 退出阈值 low_threshold: {self._field_scalar_text(detection_info['low_threshold_lsb'], include_lsb=False) if detection_info['low_threshold_lsb'] is not None else '未生成'}",
                    "",
                ]
            )

            if phase_detection["mode"] == "auto":
                report_lines.extend(
                    [
                        f"- 自动识别的 SD 启动时刻: {phase_boundaries['phase_2_start_relative_s']:.3f} s",
                        f"- 自动识别的 SD 停止后恢复时刻: {phase_boundaries['phase_3_start_relative_s']:.3f} s",
                        "",
                        "#### 三阶段统计",
                        "",
                    ]
                )

                for phase_key in ("phase_1_idle", "phase_2_active", "phase_3_recover"):
                    phase = phase_stats[phase_key]
                    report_lines.extend(
                        [
                            f"- {phase.phase_name}:",
                            f"  时间范围 {phase.start_timestamp_ms} ms -> {phase.stop_timestamp_ms} ms，样本数 {phase.sample_count}",
                            f"  X均值/方差 = {self._axis_scalar_text(phase.mean_x)} / {self._axis_variance_text(phase.var_x)}",
                            f"  Y均值/方差 = {self._axis_scalar_text(phase.mean_y)} / {self._axis_variance_text(phase.var_y)}",
                            f"  Z均值/方差 = {self._axis_scalar_text(phase.mean_z)} / {self._axis_variance_text(phase.var_z)}",
                            f"  磁场强度均值/方差 = {self._field_scalar_text(phase.mean_field_magnitude_lsb, include_lsb=False)} / {self._field_variance_text(phase.var_field_magnitude_lsb, include_lsb=False)}",
                            f"  航向角均值/方差 = {phase.mean_heading_deg:.3f} deg / {phase.var_heading_deg:.3f} deg^2",
                            f"  相对阶段1的航向偏移均值/方差 = {phase.mean_heading_offset_deg:.3f} deg / {phase.var_heading_offset_deg:.3f} deg^2",
                            f"  相对阶段1的最大绝对航向偏移 = {phase.max_abs_heading_offset_deg:.3f} deg",
                            "",
                        ]
                    )

                report_lines.extend(
                    [
                        "#### 重点结论",
                        "",
                        f"- SD 启动后相对启动前的平均航向偏移: {transition['before_to_active_mean_heading_offset_deg']:.3f}°",
                        f"- SD 启动后相对启动前的最大绝对航向偏移: {transition['before_to_active_max_abs_heading_offset_deg']:.3f}°",
                        f"- SD 关闭后相对工作中的平均航向变化: {transition['active_to_recover_mean_heading_change_deg']:.3f}°",
                        f"- SD 关闭后相对启动前的残余平均航向偏移: {transition['before_to_recover_mean_heading_offset_deg']:.3f}°",
                        f"- SD 关闭后相对启动前的残余最大绝对航向偏移: {transition['before_to_recover_max_abs_heading_offset_deg']:.3f}°",
                        "",
                    ]
                )
            else:
                phase = phase_stats["phase_all"]
                report_lines.extend(
                    [
                        f"- 自动三阶段识别结果: 未识别到明显 SD 干扰阶段",
                        f"- 说明: {phase_detection['message']}",
                        "",
                        "#### 整段测试统计",
                        "",
                        f"- {phase.phase_name}:",
                        f"  时间范围 {phase.start_timestamp_ms} ms -> {phase.stop_timestamp_ms} ms，样本数 {phase.sample_count}",
                        f"  X均值/方差 = {self._axis_scalar_text(phase.mean_x)} / {self._axis_variance_text(phase.var_x)}",
                        f"  Y均值/方差 = {self._axis_scalar_text(phase.mean_y)} / {self._axis_variance_text(phase.var_y)}",
                        f"  Z均值/方差 = {self._axis_scalar_text(phase.mean_z)} / {self._axis_variance_text(phase.var_z)}",
                        f"  磁场强度均值/方差 = {self._field_scalar_text(phase.mean_field_magnitude_lsb, include_lsb=False)} / {self._field_variance_text(phase.var_field_magnitude_lsb, include_lsb=False)}",
                        f"  航向角均值/方差 = {phase.mean_heading_deg:.3f} deg / {phase.var_heading_deg:.3f} deg^2",
                        f"  相对基线的航向偏移均值/方差 = {phase.mean_heading_offset_deg:.3f} deg / {phase.var_heading_offset_deg:.3f} deg^2",
                        f"  相对基线的最大绝对航向偏移 = {phase.max_abs_heading_offset_deg:.3f} deg",
                        "",
                        "#### 一般性结论",
                        "",
                        f"- 整段测试相对基线的平均航向偏移: {transition['before_to_active_mean_heading_offset_deg']:.3f}°",
                        f"- 整段测试相对基线的最大绝对航向偏移: {transition['before_to_active_max_abs_heading_offset_deg']:.3f}°",
                        f"- 本轮未检测到可稳定分离的 SD 启动/工作/停止三阶段，通常表示干扰很弱，或者三阶段停留时间不足。",
                        "",
                    ]
                )

        report_lines.extend(
            [
                "## 结论提醒",
                "",
                "- 只要测试全过程保持设备姿态、附近磁环境、线束状态不变，未校准原始数据也可以用于做这种“相对偏移”评估。",
                "- 如果测试时姿态变化、手靠近、金属件移动或供电状态变化，结果会混入其他变量，不能直接归因于 SD 工作。",
                "",
            ]
        )

        with (export_dir / "evaluation_report.md").open("w", encoding="utf-8") as fp:
            fp.write("\n".join(report_lines))

    @staticmethod
    def _safe_float(value) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _data_reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                if not self.data_serial:
                    break
                raw = self.data_serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self.line_queue.put(line)
            except Exception as exc:
                self.line_queue.put(f"__ERROR__::{exc}")
                break

    def _drain_line_queue(self) -> None:
        if self.is_closing:
            return

        self._drain_line_queue_once()

        self.after_job = self.root.after(100, self._drain_line_queue)

    def _drain_line_queue_once(self) -> None:
        while True:
            try:
                line = self.line_queue.get_nowait()
            except queue.Empty:
                break

            if line.startswith("__ERROR__::"):
                self.log(f"数据串口读取异常: {line.split('::', 1)[1]}")
                self.status_var.set("数据串口异常")
                continue

            self._handle_data_line(line)

    def _handle_data_line(self, line: str) -> None:
        if line.startswith("MAG_RAW,"):
            parts = line.split(",")
            if len(parts) >= 5:
                try:
                    self.samples.append(
                        MagSample(
                            timestamp_ms=int(parts[1]),
                            x=float(parts[2]),
                            y=float(parts[3]),
                            z=float(parts[4]),
                        )
                    )
                except ValueError:
                    self.log(f"无法解析数据行: {line}")
        elif line.startswith("MAG_EVENT,"):
            parts = line.split(",", 3)
            if len(parts) >= 3:
                detail = parts[3] if len(parts) >= 4 else ""
                try:
                    event = MagEvent(timestamp_ms=int(parts[1]), name=parts[2], detail=detail)
                    self.events.append(event)
                    if event.name == "MAG_CONFIG":
                        self.config_snapshot = self._parse_detail_map(detail)
                    self.log(f"收到事件: {event.name} @ {event.timestamp_ms} ms {detail}".rstrip())
                except ValueError:
                    self.log(f"无法解析事件行: {line}")
        else:
            self.log(f"DATA> {line}")

    def log(self, message: str) -> None:
        if self.is_closing:
            return

        timestamp = datetime.now().strftime("%H:%M:%S")
        try:
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)
        except tk.TclError:
            pass

    def on_close(self) -> None:
        if self.is_closing:
            return

        self.is_closing = True

        if self.after_job:
            try:
                self.root.after_cancel(self.after_job)
            except tk.TclError:
                pass
            self.after_job = None

        try:
            self.disconnect_serials(stop_stream=False, wait_reader=False)
        except Exception:
            pass

        try:
            self.root.quit()
        except tk.TclError:
            pass

        try:
            self.root.destroy()
        except tk.TclError:
            pass

    @staticmethod
    def _parse_detail_map(detail: str) -> dict:
        result = {}
        for item in detail.split(","):
            item = item.strip()
            if not item or "=" not in item:
                continue
            key, value = item.split("=", 1)
            result[key.strip()] = value.strip()
        return result

    def _is_lsb_consistent(self, config: dict) -> bool:
        if not config:
            return False

        raw_flag = str(config.get("lsb_consistent", "")).strip()
        if raw_flag in {"0", "1"}:
            return raw_flag == "1"

        lsb_to_ut = self._safe_float(config.get("lsb_to_ut"))
        expected_lsb_to_ut = self._safe_float(config.get("expected_lsb_to_ut"))
        if lsb_to_ut <= 0.0 or expected_lsb_to_ut <= 0.0:
            return False

        return abs(lsb_to_ut - expected_lsb_to_ut) < 5e-4

    def _field_scalar_text(self, value_lsb: float, include_lsb: bool = True) -> str:
        config_lsb = self._safe_float(self.config_snapshot.get("lsb_to_ut"))
        if config_lsb > 0.0:
            value_ut = value_lsb * config_lsb
            if include_lsb:
                return f"{value_ut:.3f} uT ({value_lsb:.3f} LSB)"
            return f"{value_ut:.3f} uT"
        return f"{value_lsb:.3f} LSB"

    def _field_variance_text(self, value_var_lsb: float, include_lsb: bool = True) -> str:
        config_lsb = self._safe_float(self.config_snapshot.get("lsb_to_ut"))
        if config_lsb > 0.0:
            value_var_ut2 = value_var_lsb * config_lsb * config_lsb
            if include_lsb:
                return f"{value_var_ut2:.3f} uT^2 ({value_var_lsb:.3f} LSB^2)"
            return f"{value_var_ut2:.3f} uT^2"
        return f"{value_var_lsb:.3f} LSB^2"

    def _axis_scalar_text(self, value_lsb: float) -> str:
        config_lsb = self._safe_float(self.config_snapshot.get("lsb_to_ut"))
        if config_lsb > 0.0:
            return f"{value_lsb * config_lsb:.3f} uT"
        return f"{value_lsb:.3f} LSB"

    def _axis_variance_text(self, value_var_lsb: float) -> str:
        config_lsb = self._safe_float(self.config_snapshot.get("lsb_to_ut"))
        if config_lsb > 0.0:
            return f"{value_var_lsb * config_lsb * config_lsb:.3f} uT^2"
        return f"{value_var_lsb:.3f} LSB^2"


def main() -> None:
    root = tk.Tk()
    app = MagSdEvalApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
