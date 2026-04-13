from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Callable


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


def _imu_name_from_id(imu_id: int) -> str:
    return IMU_ID_TO_NAME.get(imu_id, f"IMU_{imu_id}")


def _imu_config(imu_id: int) -> dict[str, float]:
    return IMU_CONFIGS.get(imu_id, {"odr_hz": 0.0, "accel_range_g": 0.0, "gyro_range_dps": 0.0})


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def _std(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = _mean(values)
    variance = sum((value - avg) ** 2 for value in values) / float(len(values))
    return math.sqrt(max(variance, 0.0))


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(float(value) * float(value) for value in values) / float(len(values)))


def _format_float(value: float) -> str:
    return f"{float(value):.6f}"


def _format_duration_minutes_seconds(duration_s: float) -> str:
    total_seconds = max(int(round(float(duration_s))), 0)
    minutes, seconds = divmod(total_seconds, 60)
    return f"{minutes} 分 {seconds} 秒"


def _format_file_size(byte_count: int) -> str:
    value = float(byte_count)
    units = ["B", "KB", "MB", "GB"]
    unit_index = 0
    while value >= 1024.0 and unit_index < len(units) - 1:
        value /= 1024.0
        unit_index += 1
    return f"{value:.2f} {units[unit_index]}"


def _configure_plot_font(plt, font_manager) -> None:
    cjk_candidates = ["SimHei", "Microsoft YaHei", "SimSun", "Arial Unicode MS", "Noto Sans CJK SC", "Noto Sans JP"]
    cjk_font = next(
        (font.name for candidate in cjk_candidates for font in font_manager.fontManager.ttflist if font.name == candidate),
        None,
    )
    if cjk_font:
        plt.rcParams["font.family"] = cjk_font
    plt.rcParams["axes.unicode_minus"] = False


def _estimate_packet_metrics(timestamps_us: list[int]) -> tuple[float, float, float]:
    valid_steps = [float(curr - prev) for prev, curr in zip(timestamps_us[:-1], timestamps_us[1:]) if curr > prev]
    packet_period_mean_us = _mean(valid_steps) if valid_steps else 0.0
    packet_period_std_us = _std(valid_steps) if valid_steps else 0.0
    packet_rate_hz = 1_000_000.0 / packet_period_mean_us if packet_period_mean_us > 0.0 else 0.0
    return packet_period_mean_us, packet_period_std_us, packet_rate_hz


def _detect_first_shock_index(
    accel_axes_g: dict[str, list[float]],
    timestamps_us: list[int],
    full_scale_g: float,
) -> tuple[int, dict[str, float], dict[str, float], float]:
    sample_count = len(timestamps_us)
    _, _, packet_rate_hz = _estimate_packet_metrics(timestamps_us)
    seed_count = min(max(int(packet_rate_hz * 1.0), 800), max(sample_count // 4, 128))
    seed_count = min(max(seed_count, 32), sample_count)
    baseline_mean = {axis: _mean(values[:seed_count]) for axis, values in accel_axes_g.items()}
    baseline_std = {
        axis: _std([value - baseline_mean[axis] for value in values[:seed_count]])
        for axis, values in accel_axes_g.items()
    }
    threshold_g = max(max(baseline_std.values(), default=0.0) * 8.0, full_scale_g * 0.02, 0.30)

    first_index = seed_count
    for index in range(seed_count, sample_count):
        shock_monitor = max(
            abs(accel_axes_g["x"][index] - baseline_mean["x"]),
            abs(accel_axes_g["y"][index] - baseline_mean["y"]),
            abs(accel_axes_g["z"][index] - baseline_mean["z"]),
        )
        if shock_monitor >= threshold_g:
            first_index = index
            break

    return max(first_index, seed_count), baseline_mean, baseline_std, threshold_g


def _center_axis(values: list[float], mean_value: float) -> list[float]:
    return [float(value) - float(mean_value) for value in values]


def _compute_repeat_run_lengths(sample_signatures: list[tuple[int, ...]]) -> list[int]:
    if not sample_signatures:
        return []
    lengths = [1]
    for index in range(1, len(sample_signatures)):
        if sample_signatures[index] == sample_signatures[index - 1]:
            lengths.append(lengths[-1] + 1)
        else:
            lengths.append(1)
    return lengths


def _detect_shock_event_ranges(
    timestamps_us: list[int],
    dominant_centered_g: list[float],
    threshold_g: float,
) -> list[tuple[int, int]]:
    seed_indices = [index for index, value in enumerate(dominant_centered_g) if abs(float(value)) >= threshold_g]
    if not seed_indices:
        return []

    merge_gap_us = 120_000
    ranges: list[tuple[int, int]] = []
    start_index = seed_indices[0]
    end_index = seed_indices[0]
    for index in seed_indices[1:]:
        if timestamps_us[index] - timestamps_us[end_index] <= merge_gap_us:
            end_index = index
            continue
        ranges.append((start_index, end_index))
        start_index = index
        end_index = index
    ranges.append((start_index, end_index))
    return ranges


def _find_first_valid_index(
    start_index: int,
    end_index: int,
    timestamps_us: list[int],
    repeat_run_lengths: list[int],
    valid_step_limit_us: float,
) -> int:
    for index in range(max(start_index + 1, 1), end_index + 1):
        delta_us = timestamps_us[index] - timestamps_us[index - 1]
        if 0 < delta_us <= valid_step_limit_us and repeat_run_lengths[index] < 4:
            return index
    return start_index


def _find_stable_index(
    start_index: int,
    end_index: int,
    timestamps_us: list[int],
    dominant_centered_g: list[float],
    repeat_run_lengths: list[int],
    valid_step_limit_us: float,
    stable_window_samples: int,
    amplitude_threshold_g: float,
) -> tuple[int, bool]:
    if end_index <= start_index:
        return start_index, False

    last_start = max(end_index - stable_window_samples + 1, start_index)
    for candidate in range(start_index, last_start + 1):
        window = dominant_centered_g[candidate : candidate + stable_window_samples]
        if not window:
            continue
        if max(abs(value) for value in window) > amplitude_threshold_g:
            continue
        dt_window = [
            timestamps_us[index] - timestamps_us[index - 1]
            for index in range(max(candidate, 1), min(candidate + stable_window_samples, len(timestamps_us)))
        ]
        if any(delta_us <= 0 or delta_us > valid_step_limit_us for delta_us in dt_window):
            continue
        if any(
            repeat_run_lengths[index] >= 8
            for index in range(candidate, min(candidate + stable_window_samples, len(repeat_run_lengths)))
        ):
            continue
        return candidate, True
    return end_index, False


def _classify_shock_report(report: dict[str, object]) -> tuple[str, list[str]]:
    if int(report.get("shock_event_count", 0) or 0) <= 0:
        return "未检出", ["未检测到超过阈值的冲击事件"]

    notes: list[str] = []
    if int(report.get("dropout_flag_shock_any", 0) or 0) > 0:
        notes.append("冲击后出现时间戳异常或长时间重复输出")
    if float(report.get("recovery_time_s_max", 0.0) or 0.0) > 0.200:
        notes.append(f"最慢恢复时间 {float(report.get('recovery_time_s_max', 0.0)) * 1000.0:.1f} ms")
    if float(report.get("post_shock_shift_abs_max_g", 0.0) or 0.0) > 0.050:
        notes.append(f"冲击后偏移较大 {float(report.get('post_shock_shift_abs_max_g', 0.0)):.4f} g")
    if float(report.get("peak_ratio_shock_max", 0.0) or 0.0) >= 0.95:
        notes.append("峰值接近满量程，存在疑似边界冲击")

    if int(report.get("dropout_flag_shock_any", 0) or 0) > 0:
        return "失败", notes
    if notes:
        return "需关注", notes
    return "通过", ["冲击后未见掉线，恢复时间和偏移均较小"]


def _analyze_shock_single_imu_csv(
    imu_id: int,
    csv_path: Path,
    progress_callback: Callable[[str], None] | None = None,
) -> dict[str, object]:
    imu_name = _imu_name_from_id(imu_id)
    prefix = f"{imu_name}_"
    timestamp_column = f"{prefix}packet_timestamp_continuous"
    poll_timestamp_column = f"{prefix}poll_timestamp_us"
    temp_column = f"{prefix}temp_c"
    temp_raw_column = f"{prefix}temp_raw"
    accel_columns = {axis: f"{prefix}accel_{axis}_g" for axis in ("x", "y", "z")}
    gyro_columns = {axis: f"{prefix}gyro_{axis}_deg_s" for axis in ("x", "y", "z")}
    raw_accel_columns = {axis: f"{prefix}raw_accel_{axis}" for axis in ("x", "y", "z")}
    raw_gyro_columns = {axis: f"{prefix}raw_gyro_{axis}" for axis in ("x", "y", "z")}

    timestamps_us: list[int] = []
    poll_timestamps_us: list[int] = []
    temps_c: list[float] = []
    accel_axes_g = {axis: [] for axis in ("x", "y", "z")}
    gyro_axes_deg_s = {axis: [] for axis in ("x", "y", "z")}
    sample_signatures: list[tuple[int, ...]] = []

    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row_index, row in enumerate(reader, start=1):
            timestamps_us.append(int(float(row.get(timestamp_column, "0") or 0)))
            poll_timestamps_us.append(int(float(row.get(poll_timestamp_column, "0") or 0)))
            temps_c.append(float(row.get(temp_column, "0") or 0.0))
            temp_raw = int(float(row.get(temp_raw_column, "0") or 0))
            raw_accel = tuple(int(float(row.get(raw_accel_columns[axis], "0") or 0)) for axis in ("x", "y", "z"))
            raw_gyro = tuple(int(float(row.get(raw_gyro_columns[axis], "0") or 0)) for axis in ("x", "y", "z"))
            for axis in ("x", "y", "z"):
                accel_axes_g[axis].append(float(row.get(accel_columns[axis], "0") or 0.0))
                gyro_axes_deg_s[axis].append(float(row.get(gyro_columns[axis], "0") or 0.0))
            sample_signatures.append(raw_accel + raw_gyro + (temp_raw,))
            if progress_callback is not None and row_index % 12000 == 0:
                progress_callback(f"{imu_name}: 已载入 `{row_index}` 个样本")

    sample_count = len(timestamps_us)
    if sample_count <= 0:
        return {}

    first_timestamp_us = timestamps_us[0]
    elapsed_time_s = [(timestamp - first_timestamp_us) / 1_000_000.0 for timestamp in timestamps_us]
    packet_period_mean_us, packet_period_std_us, packet_rate_hz = _estimate_packet_metrics(timestamps_us)
    nominal_packet_period_us = 1_000_000.0 / float(_imu_config(imu_id).get("odr_hz", 0.0) or 1.0)
    full_scale_g = float(_imu_config(imu_id).get("accel_range_g", 0.0) or 0.0)

    first_shock_index, _, _, initial_threshold_g = _detect_first_shock_index(
        accel_axes_g=accel_axes_g,
        timestamps_us=timestamps_us,
        full_scale_g=full_scale_g,
    )
    pre_window_sample_count = max(min(first_shock_index, sample_count), 32)
    pre_window_end_index = pre_window_sample_count - 1
    baseline_mean = {axis: _mean(values[:pre_window_sample_count]) for axis, values in accel_axes_g.items()}
    baseline_std = {
        axis: _std([value - baseline_mean[axis] for value in values[:pre_window_sample_count]])
        for axis, values in accel_axes_g.items()
    }
    centered_axes_g = {axis: _center_axis(values, baseline_mean[axis]) for axis, values in accel_axes_g.items()}
    dominant_axis = max(("x", "y", "z"), key=lambda axis: _rms(centered_axes_g[axis]))
    dominant_centered_g = centered_axes_g[dominant_axis]
    dominant_raw_g = accel_axes_g[dominant_axis]
    dominant_threshold_g = max(baseline_std[dominant_axis] * 8.0, full_scale_g * 0.02, 0.30)
    event_ranges = _detect_shock_event_ranges(timestamps_us, dominant_centered_g, dominant_threshold_g)
    repeat_run_lengths = _compute_repeat_run_lengths(sample_signatures)
    valid_step_limit_us = (
        max(packet_period_mean_us, nominal_packet_period_us) * 1.8 if packet_period_mean_us > 0.0 else nominal_packet_period_us * 1.8
    )
    dropout_step_limit_us = (
        max(packet_period_mean_us, nominal_packet_period_us) * 3.0 if packet_period_mean_us > 0.0 else nominal_packet_period_us * 3.0
    )
    stable_window_samples = max(int(packet_rate_hz * 0.04), 24)

    shock_events: list[dict[str, object]] = []
    for event_index, (seed_start_index, seed_end_index) in enumerate(event_ranges, start=1):
        next_start_index = event_ranges[event_index][0] if event_index < len(event_ranges) else sample_count - 1
        search_end_index = max(next_start_index - 1, seed_end_index)
        peak_search_end_index = min(search_end_index, seed_end_index + max(int(packet_rate_hz * 0.05), 16))
        peak_index = max(
            range(seed_start_index, peak_search_end_index + 1),
            key=lambda index: abs(dominant_centered_g[index]),
        )
        peak_abs_excursion_g = abs(dominant_centered_g[peak_index])
        peak_ratio_shock = peak_abs_excursion_g / full_scale_g if full_scale_g > 0.0 else 0.0
        first_valid_index = _find_first_valid_index(
            start_index=seed_start_index,
            end_index=search_end_index,
            timestamps_us=timestamps_us,
            repeat_run_lengths=repeat_run_lengths,
            valid_step_limit_us=valid_step_limit_us,
        )
        amplitude_threshold_g = max(
            baseline_std[dominant_axis] * 6.0,
            min(max(peak_abs_excursion_g * 0.08, 0.12), max(full_scale_g * 0.08, 0.12)),
        )
        stable_index, stable_found = _find_stable_index(
            start_index=min(seed_end_index + 1, search_end_index),
            end_index=search_end_index,
            timestamps_us=timestamps_us,
            dominant_centered_g=dominant_centered_g,
            repeat_run_lengths=repeat_run_lengths,
            valid_step_limit_us=valid_step_limit_us,
            stable_window_samples=stable_window_samples,
            amplitude_threshold_g=amplitude_threshold_g,
        )
        post_window_end_index = min(stable_index + stable_window_samples - 1, search_end_index)
        post_mean_g = (
            _mean(dominant_raw_g[stable_index : post_window_end_index + 1])
            if stable_index <= post_window_end_index
            else dominant_raw_g[stable_index]
        )
        post_shock_shift_g = post_mean_g - baseline_mean[dominant_axis]
        has_gap = any(
            (timestamps_us[index] - timestamps_us[index - 1]) <= 0
            or (timestamps_us[index] - timestamps_us[index - 1]) > dropout_step_limit_us
            for index in range(max(seed_start_index + 1, 1), min(stable_index + stable_window_samples, sample_count))
        )
        repeated_after_shock = max(
            repeat_run_lengths[seed_start_index : min(stable_index + stable_window_samples, sample_count)] or [1]
        ) >= 8
        dropout_flag_shock = 1 if has_gap or repeated_after_shock else 0

        shock_events.append(
            {
                "event_index": event_index,
                "shock_start_index": seed_start_index,
                "shock_end_index": seed_end_index,
                "shock_peak_index": peak_index,
                "t_shock_s": elapsed_time_s[seed_start_index],
                "t_peak_s": elapsed_time_s[peak_index],
                "peak_abs_excursion_g": peak_abs_excursion_g,
                "peak_ratio_shock": peak_ratio_shock,
                "peak_ratio_shock_pct": peak_ratio_shock * 100.0,
                "dropout_flag_shock": dropout_flag_shock,
                "t_first_valid_s": elapsed_time_s[first_valid_index],
                "t_first_valid_latency_ms": max(elapsed_time_s[first_valid_index] - elapsed_time_s[seed_start_index], 0.0) * 1000.0,
                "t_stable_s": elapsed_time_s[stable_index],
                "recovery_time_s": max(elapsed_time_s[stable_index] - elapsed_time_s[seed_start_index], 0.0),
                "recovery_time_ms": max(elapsed_time_s[stable_index] - elapsed_time_s[seed_start_index], 0.0) * 1000.0,
                "stable_found": 1 if stable_found else 0,
                "post_shock_shift_g": post_shock_shift_g,
                "post_window_end_s": elapsed_time_s[post_window_end_index],
                "analysis_end_s": elapsed_time_s[search_end_index],
                "amplitude_threshold_g": amplitude_threshold_g,
                "shock_threshold_g": dominant_threshold_g,
            }
        )

    peak_ratio_shock_max = max((float(event["peak_ratio_shock"]) for event in shock_events), default=0.0)
    dropout_flag_any = 1 if any(int(event["dropout_flag_shock"]) > 0 for event in shock_events) else 0
    t_first_valid_latency_ms_min = min((float(event["t_first_valid_latency_ms"]) for event in shock_events), default=0.0)
    recovery_time_s_max = max((float(event["recovery_time_s"]) for event in shock_events), default=0.0)
    post_shock_shift_abs_max_g = max((abs(float(event["post_shock_shift_g"])) for event in shock_events), default=0.0)
    temp_min_c = min(temps_c) if temps_c else 0.0
    temp_max_c = max(temps_c) if temps_c else 0.0

    report = {
        "imu_id": imu_id,
        "imu_name": imu_name,
        "sample_count": sample_count,
        "duration_s": elapsed_time_s[-1] if elapsed_time_s else 0.0,
        "packet_rate_hz": packet_rate_hz,
        "packet_period_mean_us": packet_period_mean_us,
        "packet_period_std_us": packet_period_std_us,
        "configured_odr_hz": float(_imu_config(imu_id).get("odr_hz", 0.0) or 0.0),
        "accel_range_g": full_scale_g,
        "gyro_range_dps": float(_imu_config(imu_id).get("gyro_range_dps", 0.0) or 0.0),
        "dominant_axis": dominant_axis,
        "initial_threshold_g": initial_threshold_g,
        "shock_threshold_g": dominant_threshold_g,
        "pre_window_sample_count": pre_window_sample_count,
        "pre_window_end_time_s": elapsed_time_s[pre_window_end_index],
        "pre_window_mean_x_g": baseline_mean["x"],
        "pre_window_mean_y_g": baseline_mean["y"],
        "pre_window_mean_z_g": baseline_mean["z"],
        "pre_window_std_x_g": baseline_std["x"],
        "pre_window_std_y_g": baseline_std["y"],
        "pre_window_std_z_g": baseline_std["z"],
        "peak_ratio_shock_max": peak_ratio_shock_max,
        "peak_ratio_shock_max_pct": peak_ratio_shock_max * 100.0,
        "dropout_flag_shock_any": dropout_flag_any,
        "t_first_valid_latency_ms_min": t_first_valid_latency_ms_min,
        "recovery_time_s_max": recovery_time_s_max,
        "post_shock_shift_abs_max_g": post_shock_shift_abs_max_g,
        "shock_event_count": len(shock_events),
        "shock_events": shock_events,
        "temp_min_c": temp_min_c,
        "temp_max_c": temp_max_c,
        "elapsed_time_s": elapsed_time_s,
        "dominant_raw_g": dominant_raw_g,
        "dominant_centered_g": dominant_centered_g,
    }
    shock_status, shock_notes = _classify_shock_report(report)
    report["shock_status"] = shock_status
    report["shock_notes"] = shock_notes
    return report


def _write_shock_summary_csv(output_dir: Path, source_stem: str, per_imu_report: dict[int, dict[str, object]]) -> Path:
    csv_path = output_dir / f"{source_stem}_shock_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "imu_id",
                "imu_name",
                "shock_status",
                "dominant_axis",
                "sample_count",
                "packet_rate_hz",
                "pre_window_sample_count",
                "pre_window_end_time_s",
                "shock_event_count",
                "peak_ratio_shock_max",
                "peak_ratio_shock_max_pct",
                "dropout_flag_shock_any",
                "t_first_valid_latency_ms_min",
                "recovery_time_s_max",
                "post_shock_shift_abs_max_g",
                "temp_min_c",
                "temp_max_c",
                "shock_notes",
                "shock_chart_png",
            ]
        )
        for imu_id in sorted(int(key) for key in per_imu_report):
            sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
            writer.writerow(
                [
                    imu_id,
                    sr.get("imu_name", _imu_name_from_id(imu_id)),
                    sr.get("shock_status", ""),
                    sr.get("dominant_axis", ""),
                    int(sr.get("sample_count", 0)),
                    _format_float(float(sr.get("packet_rate_hz", 0.0))),
                    int(sr.get("pre_window_sample_count", 0)),
                    _format_float(float(sr.get("pre_window_end_time_s", 0.0))),
                    int(sr.get("shock_event_count", 0)),
                    _format_float(float(sr.get("peak_ratio_shock_max", 0.0))),
                    _format_float(float(sr.get("peak_ratio_shock_max_pct", 0.0))),
                    int(sr.get("dropout_flag_shock_any", 0)),
                    _format_float(float(sr.get("t_first_valid_latency_ms_min", 0.0))),
                    _format_float(float(sr.get("recovery_time_s_max", 0.0))),
                    _format_float(float(sr.get("post_shock_shift_abs_max_g", 0.0))),
                    _format_float(float(sr.get("temp_min_c", 0.0))),
                    _format_float(float(sr.get("temp_max_c", 0.0))),
                    "；".join(str(item) for item in sr.get("shock_notes", [])),
                    sr.get("shock_chart_png", ""),
                ]
            )
    return csv_path


def _write_shock_event_csv(output_dir: Path, source_stem: str, per_imu_report: dict[int, dict[str, object]]) -> Path:
    csv_path = output_dir / f"{source_stem}_shock_events.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "imu_id",
                "imu_name",
                "event_index",
                "dominant_axis",
                "t_shock_s",
                "t_peak_s",
                "peak_abs_excursion_g",
                "peak_ratio_shock",
                "peak_ratio_shock_pct",
                "dropout_flag_shock",
                "t_first_valid_s",
                "t_first_valid_latency_ms",
                "t_stable_s",
                "recovery_time_s",
                "recovery_time_ms",
                "stable_found",
                "post_shock_shift_g",
                "shock_threshold_g",
                "amplitude_threshold_g",
                "analysis_end_s",
            ]
        )
        for imu_id in sorted(int(key) for key in per_imu_report):
            sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
            for event in list(sr.get("shock_events", []) or []):
                writer.writerow(
                    [
                        imu_id,
                        sr.get("imu_name", _imu_name_from_id(imu_id)),
                        int(event.get("event_index", 0)),
                        sr.get("dominant_axis", ""),
                        _format_float(float(event.get("t_shock_s", 0.0))),
                        _format_float(float(event.get("t_peak_s", 0.0))),
                        _format_float(float(event.get("peak_abs_excursion_g", 0.0))),
                        _format_float(float(event.get("peak_ratio_shock", 0.0))),
                        _format_float(float(event.get("peak_ratio_shock_pct", 0.0))),
                        int(event.get("dropout_flag_shock", 0)),
                        _format_float(float(event.get("t_first_valid_s", 0.0))),
                        _format_float(float(event.get("t_first_valid_latency_ms", 0.0))),
                        _format_float(float(event.get("t_stable_s", 0.0))),
                        _format_float(float(event.get("recovery_time_s", 0.0))),
                        _format_float(float(event.get("recovery_time_ms", 0.0))),
                        int(event.get("stable_found", 0)),
                        _format_float(float(event.get("post_shock_shift_g", 0.0))),
                        _format_float(float(event.get("shock_threshold_g", 0.0))),
                        _format_float(float(event.get("amplitude_threshold_g", 0.0))),
                        _format_float(float(event.get("analysis_end_s", 0.0))),
                    ]
                )
    return csv_path


def _generate_shock_charts(source_stem: str, output_dir: Path, per_imu_report: dict[int, dict[str, object]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.font_manager as _fm
        import matplotlib.pyplot as plt
    except Exception:
        return

    _configure_plot_font(plt, _fm)

    for imu_id in sorted(int(key) for key in per_imu_report):
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        elapsed_time_s = list(sr.get("elapsed_time_s", []) or [])
        dominant_raw_g = list(sr.get("dominant_raw_g", []) or [])
        dominant_centered_g = list(sr.get("dominant_centered_g", []) or [])
        if not elapsed_time_s or not dominant_raw_g or not dominant_centered_g:
            continue

        chart_path = output_dir / f"{source_stem}_{sr.get('imu_name', _imu_name_from_id(imu_id))}_shock_timeline.png"
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        baseline_mean = float(sr.get(f"pre_window_mean_{sr.get('dominant_axis', 'z')}_g", 0.0) or 0.0)

        axes[0].plot(elapsed_time_s, dominant_raw_g, color="#2F6BFF", linewidth=0.9)
        axes[0].axhline(baseline_mean, color="#F28E2B", linestyle="--", linewidth=1.0)
        axes[0].axvspan(0.0, float(sr.get("pre_window_end_time_s", 0.0) or 0.0), color="#FAD7A0", alpha=0.25)
        axes[0].set_ylabel("Dominant Accel (g)")
        axes[0].set_title(f"{sr.get('imu_name', _imu_name_from_id(imu_id))}: dominant axis `{sr.get('dominant_axis', '-')}` raw")
        axes[0].grid(True, linestyle=":", linewidth=0.6, alpha=0.7)

        axes[1].plot(elapsed_time_s, dominant_centered_g, color="#D94841", linewidth=0.9)
        threshold_g = float(sr.get("shock_threshold_g", 0.0) or 0.0)
        axes[1].axhline(threshold_g, color="#666666", linestyle="--", linewidth=0.9)
        axes[1].axhline(-threshold_g, color="#666666", linestyle="--", linewidth=0.9)
        axes[1].set_ylabel("Dominant Accel Debiased (g)")
        axes[1].set_xlabel("Elapsed Time (s)")
        axes[1].grid(True, linestyle=":", linewidth=0.6, alpha=0.7)

        for event in list(sr.get("shock_events", []) or []):
            t_shock_s = float(event.get("t_shock_s", 0.0) or 0.0)
            t_peak_s = float(event.get("t_peak_s", 0.0) or 0.0)
            t_first_valid_s = float(event.get("t_first_valid_s", 0.0) or 0.0)
            t_stable_s = float(event.get("t_stable_s", 0.0) or 0.0)
            axes[0].axvline(t_shock_s, color="#D94841", linestyle="--", linewidth=0.9)
            axes[0].axvline(t_stable_s, color="#2A9D8F", linestyle="--", linewidth=0.9)
            axes[0].axvspan(t_shock_s, t_stable_s, color="#D94841", alpha=0.08)
            axes[1].axvline(t_shock_s, color="#D94841", linestyle="--", linewidth=0.9)
            axes[1].axvline(t_first_valid_s, color="#2F6BFF", linestyle=":", linewidth=0.9)
            axes[1].axvline(t_stable_s, color="#2A9D8F", linestyle="--", linewidth=0.9)
            axes[1].scatter([t_peak_s], [float(dominant_centered_g[int(event.get("shock_peak_index", 0))])], color="#F28E2B", s=22, zorder=5)

        fig.tight_layout()
        fig.savefig(chart_path, dpi=140, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        sr["shock_chart_png"] = chart_path.name


def analyze_shock_per_imu_from_csvs(
    per_imu_csv_paths: dict[int, Path],
    source: Path,
    output_dir: Path,
    progress_callback: Callable[[str], None] | None = None,
) -> tuple[dict[int, dict[str, object]], Path, Path]:
    per_imu_report: dict[int, dict[str, object]] = {}
    for imu_id, csv_path in sorted(per_imu_csv_paths.items()):
        if progress_callback is not None:
            progress_callback(f"{_imu_name_from_id(imu_id)}: 开始按 3.5 方案统计冲击恢复指标")
        per_imu_report[imu_id] = _analyze_shock_single_imu_csv(
            imu_id=imu_id,
            csv_path=csv_path,
            progress_callback=progress_callback,
        )
    _generate_shock_charts(source.stem, output_dir, per_imu_report)
    summary_csv_path = _write_shock_summary_csv(output_dir, source.stem, per_imu_report)
    event_csv_path = _write_shock_event_csv(output_dir, source.stem, per_imu_report)
    return per_imu_report, summary_csv_path, event_csv_path


def build_shock_markdown_lines(source: Path, summary: dict[str, object], destination: Path) -> list[str]:
    _ = destination
    per_imu_report = summary.get("per_imu_report", {}) or {}
    all_imu_ids = [
        imu_id
        for imu_id in sorted(int(k) for k in per_imu_report)
        if per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id))
    ]
    if not all_imu_ids:
        return ["# 冲击后数据恢复分析报告", "", f"- 源文件: `{source.name}`", "", "- 未解析到可用于冲击恢复分析的 IMU 数据。"]

    status_counts: dict[str, int] = {}
    total_event_count = 0
    worst_recovery_time_s = 0.0
    peak_ratio_max_pct = 0.0
    dropout_count = 0
    for imu_id in all_imu_ids:
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        status = str(sr.get("shock_status", "需关注"))
        status_counts[status] = status_counts.get(status, 0) + 1
        total_event_count += int(sr.get("shock_event_count", 0) or 0)
        worst_recovery_time_s = max(worst_recovery_time_s, float(sr.get("recovery_time_s_max", 0.0) or 0.0))
        peak_ratio_max_pct = max(peak_ratio_max_pct, float(sr.get("peak_ratio_shock_max_pct", 0.0) or 0.0))
        dropout_count += int(sr.get("dropout_flag_shock_any", 0) or 0)

    source_size_text = str(summary.get("source_size_text", _format_file_size(source.stat().st_size if source.exists() else 0)))
    duration_s = max(float((per_imu_report.get(imu_id) or {}).get("duration_s", 0.0) or 0.0) for imu_id in all_imu_ids)

    lines = [
        "# 2.5 冲击后数据恢复分析报告",
        "",
        "## 2.5.1 结论先看",
        "",
        f"- 数据文件: `{source.name}`，序号 `{source.stem}`，文件大小 `{source_size_text}`，记录时长约 `{_format_duration_minutes_seconds(duration_s)}`。",
        f"- 本次共识别到 `{total_event_count}` 个冲击事件，最大峰值比 `{peak_ratio_max_pct:.1f}% FS`，最慢恢复时间 `{worst_recovery_time_s * 1000.0:.1f} ms`。",
        f"- IMU 结论分布: 通过 `{status_counts.get('通过', 0)}` 个，需关注 `{status_counts.get('需关注', 0)}` 个，失败 `{status_counts.get('失败', 0)}` 个，未检出 `{status_counts.get('未检出', 0)}` 个。",
        f"- `dropout_flag_shock = 1` 的 IMU 共 `{dropout_count}` 个，说明本次没有识别到冲击后掉线或明显卡死的样本。" if dropout_count == 0 else f"- `dropout_flag_shock = 1` 的 IMU 共 `{dropout_count}` 个，需要重点关注冲击后是否存在断流或卡死。",
        "- 本报告按 `document/02-各方案选项备注.md` 中 `3.5` 的方案执行：先选主响应轴，再检测冲击事件，随后统计 `peak_ratio_shock / dropout_flag_shock / t_first_valid / recovery_time / post_shock_shift`。",
        "",
        "### 2.5.1.1 各 IMU 结论",
        "",
    ]
    for imu_id in all_imu_ids:
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        notes = "；".join(str(item) for item in sr.get("shock_notes", []) or [])
        lines.append(
            "- "
            f"`{sr.get('imu_name', _imu_name_from_id(imu_id))}`: `{sr.get('shock_status', '需关注')}`，"
            f"主响应轴 `{sr.get('dominant_axis', '-')}`，"
            f"冲击事件 `{int(sr.get('shock_event_count', 0))}` 个，"
            f"最大峰值比 `{float(sr.get('peak_ratio_shock_max_pct', 0.0)):.1f}% FS`，"
            f"最慢恢复 `{float(sr.get('recovery_time_s_max', 0.0)) * 1000.0:.1f} ms`，"
            f"最大偏移 `{float(sr.get('post_shock_shift_abs_max_g', 0.0)):.4f} g`"
            f"{f'；{notes}' if notes else ''}。"
        )

    lines.extend(
        [
            "",
            "## 2.5.2 基础配置与实测采样",
            "",
            "| IMU | 样本数 | 配置 ODR (Hz) | 实测采样率 (Hz) | 主响应轴 | 冲击前稳定段样本数 | 冲击前稳定段结束时间 (s) | 温度范围 (°C) |",
            "| --- | ---: | ---: | ---: | --- | ---: | ---: | --- |",
        ]
    )
    for imu_id in all_imu_ids:
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        lines.append(
            "| "
            + " | ".join(
                [
                    str(sr.get("imu_name", _imu_name_from_id(imu_id))),
                    str(int(sr.get("sample_count", 0))),
                    f"{float(sr.get('configured_odr_hz', 0.0)):.3f}",
                    f"{float(sr.get('packet_rate_hz', 0.0)):.6f}",
                    str(sr.get("dominant_axis", "")),
                    str(int(sr.get("pre_window_sample_count", 0))),
                    f"{float(sr.get('pre_window_end_time_s', 0.0)):.6f}",
                    f"{float(sr.get('temp_min_c', 0.0)):.3f} ~ {float(sr.get('temp_max_c', 0.0)):.3f}",
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## 2.5.3 冲击恢复指标汇总",
            "",
            "| IMU | 结果 | 主响应轴 | 事件数 | 峰值比 (%FS) | dropout_flag | 首个有效样本最短延迟 (ms) | 最慢恢复时间 (ms) | 最大冲击后偏移 (g) |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for imu_id in all_imu_ids:
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        lines.append(
            "| "
            + " | ".join(
                [
                    str(sr.get("imu_name", _imu_name_from_id(imu_id))),
                    str(sr.get("shock_status", "")),
                    str(sr.get("dominant_axis", "")),
                    str(int(sr.get("shock_event_count", 0))),
                    f"{float(sr.get('peak_ratio_shock_max_pct', 0.0)):.3f}",
                    str(int(sr.get("dropout_flag_shock_any", 0))),
                    f"{float(sr.get('t_first_valid_latency_ms_min', 0.0)):.3f}",
                    f"{float(sr.get('recovery_time_s_max', 0.0)) * 1000.0:.3f}",
                    f"{float(sr.get('post_shock_shift_abs_max_g', 0.0)):.6f}",
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## 2.5.4 冲击事件明细",
            "",
            "| IMU | Event | t_shock (s) | 峰值比 (%FS) | dropout_flag | t_first_valid 延迟 (ms) | recovery_time (ms) | post_shock_shift (g) |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for imu_id in all_imu_ids:
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        for event in list(sr.get("shock_events", []) or []):
            lines.append(
                "| "
                + " | ".join(
                    [
                        str(sr.get("imu_name", _imu_name_from_id(imu_id))),
                        str(int(event.get("event_index", 0))),
                        f"{float(event.get('t_shock_s', 0.0)):.6f}",
                        f"{float(event.get('peak_ratio_shock_pct', 0.0)):.6f}",
                        str(int(event.get("dropout_flag_shock", 0))),
                        f"{float(event.get('t_first_valid_latency_ms', 0.0)):.6f}",
                        f"{float(event.get('recovery_time_ms', 0.0)):.6f}",
                        f"{float(event.get('post_shock_shift_g', 0.0)):.6f}",
                    ]
                )
                + " |"
            )

    lines.extend(
        [
            "",
            "## 2.5.5 对比图",
            "",
        ]
    )
    for chart_index, imu_id in enumerate(all_imu_ids, start=1):
        sr = per_imu_report.get(imu_id) or per_imu_report.get(str(imu_id)) or {}
        chart_name = str(sr.get("shock_chart_png", "") or "").strip()
        if not chart_name:
            continue
        lines.extend(
            [
                f"### 2.5.5.{chart_index} {sr.get('imu_name', _imu_name_from_id(imu_id))}",
                "",
                f"- 主响应轴: `{sr.get('dominant_axis', '-')}`。",
                f"- 冲击前稳定段结束时间: `{float(sr.get('pre_window_end_time_s', 0.0)):.6f} s`。",
                f"- 图中红色虚线表示 `t_shock`，绿色虚线表示 `t_stable`，蓝色点划线表示 `t_first_valid`。",
                f"![{sr.get('imu_name', _imu_name_from_id(imu_id))} shock timeline](./{chart_name})",
                "",
            ]
        )

    lines.extend(
        [
            "## 2.5.6 判定逻辑",
            "",
            "- `axis_dom`: 先用冲击前稳定段均值去偏置，再比较三轴加速度全段 RMS，选择主响应轴。",
            "- `peak_ratio_shock`: 使用主响应轴相对冲击前均值的峰值 excursion 与满量程 `FS_acc` 相比得到。",
            "- `dropout_flag_shock`: 冲击后若出现显著时间戳大步长、非正向时间戳或长时间重复值，则记为 `1`。",
            "- `t_first_valid`: 从 `t_shock` 之后找到首个时间连续、未出现明显重复卡死的有效样本。",
            "- `recovery_time`: 从 `t_shock` 到主响应轴重新回到低幅稳定窗口的时间。",
            "- `post_shock_shift`: 使用主响应轴冲击前稳定均值与冲击后稳定窗口均值的差值表示。",
            "- 上述稳定阈值和恢复判据是按文档 `3.5` 的描述做的工程化离线实现，属于从原始数据推断的分析结果。",
            "",
            "## 2.5.7 导出文件",
            "",
            f"- 冲击汇总 CSV: `{summary.get('shock_summary_csv', '-')}`。",
            f"- 冲击事件 CSV: `{summary.get('shock_events_csv', '-')}`。",
            f"- 逐 poll 对比 CSV: `{summary.get('comparison_csv', '-')}`。",
            f"- 每颗 IMU 的逐包 CSV: `{summary.get('per_imu_csvs', '-')}`。",
        ]
    )
    return lines
