from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


GRAVITY_MPS2 = 9.80665
REPORT_BEGIN_MARKER = "<!-- STATIC_BIAS_ANALYSIS:BEGIN -->"
REPORT_END_MARKER = "<!-- STATIC_BIAS_ANALYSIS:END -->"
IMU_CSV_PATTERN = re.compile(r"^(?P<sequence>\d+)_(?P<imu_name>(?:42688|45686)_[AB])\.csv$")


@dataclass(frozen=True)
class AxisSpec:
    family: str
    axis: str
    suffix: str
    input_unit: str
    output_unit: str
    integrated_suffix: str
    scale: float
    short_label: str


@dataclass(frozen=True)
class StaticWindow:
    sample_count: int
    end_index: int
    end_time_s: float
    detection_column: str
    baseline_mean: float
    baseline_std: float
    threshold_std: float
    threshold_peak: float


@dataclass
class AxisAnalysis:
    spec: AxisSpec
    column_name: str
    bias_column_name: str
    debiased_column_name: str
    integrated_column_name: str
    bias_value: float
    raw_values: list[float]
    debiased_values: list[float]
    integrated_values: list[float]


@dataclass
class ImuAnalysis:
    sequence: str
    imu_name: str
    csv_path: Path
    output_csv_path: Path
    output_png_path: Path
    timestamp_column: str
    elapsed_time_s: list[float]
    timestamps_continuous: list[int]
    sample_rate_hz: float
    static_window: StaticWindow
    axis_results: list[AxisAnalysis]


AXIS_SPECS = [
    AxisSpec(
        family="accel",
        axis="x",
        suffix="accel_x_g",
        input_unit="g",
        output_unit="m/s",
        integrated_suffix="debiased_integrated_velocity_m_s",
        scale=GRAVITY_MPS2,
        short_label="Accel X",
    ),
    AxisSpec(
        family="accel",
        axis="y",
        suffix="accel_y_g",
        input_unit="g",
        output_unit="m/s",
        integrated_suffix="debiased_integrated_velocity_m_s",
        scale=GRAVITY_MPS2,
        short_label="Accel Y",
    ),
    AxisSpec(
        family="accel",
        axis="z",
        suffix="accel_z_g",
        input_unit="g",
        output_unit="m/s",
        integrated_suffix="debiased_integrated_velocity_m_s",
        scale=GRAVITY_MPS2,
        short_label="Accel Z",
    ),
    AxisSpec(
        family="gyro",
        axis="x",
        suffix="gyro_x_deg_s",
        input_unit="deg/s",
        output_unit="deg",
        integrated_suffix="debiased_integrated_angle_deg",
        scale=1.0,
        short_label="Gyro X",
    ),
    AxisSpec(
        family="gyro",
        axis="y",
        suffix="gyro_y_deg_s",
        input_unit="deg/s",
        output_unit="deg",
        integrated_suffix="debiased_integrated_angle_deg",
        scale=1.0,
        short_label="Gyro Y",
    ),
    AxisSpec(
        family="gyro",
        axis="z",
        suffix="gyro_z_deg_s",
        input_unit="deg/s",
        output_unit="deg",
        integrated_suffix="debiased_integrated_angle_deg",
        scale=1.0,
        short_label="Gyro Z",
    ),
]


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def _std(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = _mean(values)
    variance = sum((value - avg) ** 2 for value in values) / float(len(values))
    return math.sqrt(variance)


def _integrate(values: list[float], dt: float, scale: float) -> list[float]:
    integrated: list[float] = []
    current = 0.0
    for index, value in enumerate(values):
        if index == 0:
            integrated.append(current)
            continue
        current += value * scale * dt
        integrated.append(current)
    return integrated


def _estimate_sample_rate_hz(timestamps_continuous: list[int]) -> float:
    diffs = [
        float(curr - prev)
        for prev, curr in zip(timestamps_continuous[:-1], timestamps_continuous[1:])
        if curr > prev
    ]
    if not diffs:
        return 0.0
    mean_diff_us = _mean(diffs)
    if mean_diff_us <= 0.0:
        return 0.0
    return 1_000_000.0 / mean_diff_us


def _detect_static_window(
    elapsed_time_s: list[float],
    timestamps_continuous: list[int],
    accel_z_values: list[float],
    detection_column: str,
) -> StaticWindow:
    sample_rate_hz = _estimate_sample_rate_hz(timestamps_continuous)
    total_count = len(accel_z_values)
    if total_count < 64:
        end_index = max(total_count - 1, 0)
        return StaticWindow(
            sample_count=max(total_count, 1),
            end_index=end_index,
            end_time_s=elapsed_time_s[end_index] if elapsed_time_s else 0.0,
            detection_column=detection_column,
            baseline_mean=_mean(accel_z_values),
            baseline_std=_std(accel_z_values),
            threshold_std=0.0,
            threshold_peak=0.0,
        )

    seed_count = min(max(int(sample_rate_hz * 1.5), 1000), max(total_count // 4, 256))
    seed_count = min(seed_count, total_count)
    seed_values = accel_z_values[:seed_count]
    baseline_mean = _mean(seed_values)
    baseline_std = _std(seed_values)

    window_size = min(max(int(sample_rate_hz * 0.2), 160), max(total_count // 6, 64))
    step_size = max(window_size // 10, 10)
    threshold_std = max(baseline_std * 5.0, 0.15)
    threshold_peak = max(baseline_std * 10.0, 0.8)
    onset_index: int | None = None

    for start in range(seed_count, total_count - window_size, step_size):
        window = accel_z_values[start : start + window_size]
        peak = max(abs(value - baseline_mean) for value in window)
        window_std = _std(window)
        if window_std > threshold_std or peak > threshold_peak:
            onset_index = start
            break

    if onset_index is None:
        onset_index = seed_count

    refine_threshold = max(baseline_std * 6.0, 0.4)
    refine_start = max(seed_count, onset_index - window_size)
    for index in range(refine_start, onset_index):
        if abs(accel_z_values[index] - baseline_mean) >= refine_threshold:
            onset_index = index
            break

    sample_count = max(onset_index, 32)
    sample_count = min(sample_count, total_count)
    end_index = sample_count - 1
    return StaticWindow(
        sample_count=sample_count,
        end_index=end_index,
        end_time_s=elapsed_time_s[end_index],
        detection_column=detection_column,
        baseline_mean=baseline_mean,
        baseline_std=baseline_std,
        threshold_std=threshold_std,
        threshold_peak=threshold_peak,
    )


def _load_imu_analysis(csv_path: Path, dt: float) -> ImuAnalysis:
    match = IMU_CSV_PATTERN.match(csv_path.name)
    if not match:
        raise ValueError(f"unsupported imu csv filename: {csv_path.name}")

    sequence = match.group("sequence")
    imu_name = match.group("imu_name")
    prefix = f"{imu_name}_"
    timestamp_column = f"{prefix}packet_timestamp_continuous"

    fieldnames: list[str] = []
    raw_rows: list[dict[str, str]] = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        raw_rows = list(reader)

    if timestamp_column not in fieldnames:
        raise ValueError(f"missing timestamp column: {timestamp_column}")

    timestamps_continuous = [int(float(row[timestamp_column])) for row in raw_rows]
    t0 = timestamps_continuous[0]
    elapsed_time_s = [(timestamp - t0) / 1_000_000.0 for timestamp in timestamps_continuous]
    sample_rate_hz = _estimate_sample_rate_hz(timestamps_continuous)

    accel_z_column = f"{prefix}accel_z_g"
    accel_z_values = [float(row[accel_z_column]) for row in raw_rows]
    static_window = _detect_static_window(
        elapsed_time_s=elapsed_time_s,
        timestamps_continuous=timestamps_continuous,
        accel_z_values=accel_z_values,
        detection_column=accel_z_column,
    )

    axis_results: list[AxisAnalysis] = []
    for spec in AXIS_SPECS:
        column_name = f"{prefix}{spec.suffix}"
        if column_name not in fieldnames:
            raise ValueError(f"missing axis column: {column_name}")
        raw_values = [float(row[column_name]) for row in raw_rows]
        bias_value = _mean(raw_values[: static_window.sample_count])
        debiased_values = [value - bias_value for value in raw_values]
        integrated_values = _integrate(debiased_values, dt=dt, scale=spec.scale)
        axis_results.append(
            AxisAnalysis(
                spec=spec,
                column_name=column_name,
                bias_column_name=f"{column_name}_static_bias",
                debiased_column_name=f"{column_name}_debiased",
                integrated_column_name=f"{column_name}_{spec.integrated_suffix}",
                bias_value=bias_value,
                raw_values=raw_values,
                debiased_values=debiased_values,
                integrated_values=integrated_values,
            )
        )

    return ImuAnalysis(
        sequence=sequence,
        imu_name=imu_name,
        csv_path=csv_path,
        output_csv_path=csv_path.parent / f"{sequence}_{imu_name}_static_bias_6axis.csv",
        output_png_path=csv_path.parent / f"{sequence}_{imu_name}_static_bias_6axis.png",
        timestamp_column=timestamp_column,
        elapsed_time_s=elapsed_time_s,
        timestamps_continuous=timestamps_continuous,
        sample_rate_hz=sample_rate_hz,
        static_window=static_window,
        axis_results=axis_results,
    )


def _write_imu_csv(analysis: ImuAnalysis) -> Path:
    fieldnames = [
        "sample_index",
        "elapsed_time_s",
        analysis.timestamp_column,
        "bias_window_sample_count",
        "bias_window_end_time_s",
    ]
    for axis_result in analysis.axis_results:
        fieldnames.extend(
            [
                axis_result.column_name,
                axis_result.bias_column_name,
                axis_result.debiased_column_name,
                axis_result.integrated_column_name,
            ]
        )

    with analysis.output_csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        sample_count = len(analysis.elapsed_time_s)
        for index in range(sample_count):
            row = {
                "sample_index": index,
                "elapsed_time_s": f"{analysis.elapsed_time_s[index]:.6f}",
                analysis.timestamp_column: str(analysis.timestamps_continuous[index]),
                "bias_window_sample_count": analysis.static_window.sample_count,
                "bias_window_end_time_s": f"{analysis.static_window.end_time_s:.6f}",
            }
            for axis_result in analysis.axis_results:
                row[axis_result.column_name] = f"{axis_result.raw_values[index]:.9f}"
                row[axis_result.bias_column_name] = f"{axis_result.bias_value:.9f}"
                row[axis_result.debiased_column_name] = f"{axis_result.debiased_values[index]:.9f}"
                row[axis_result.integrated_column_name] = f"{axis_result.integrated_values[index]:.9f}"
            writer.writerow(row)
    return analysis.output_csv_path


def _plot_imu_analysis(analysis: ImuAnalysis, dt: float) -> Path:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"matplotlib unavailable: {exc}") from exc

    fig, axes = plt.subplots(
        len(analysis.axis_results),
        3,
        figsize=(18, 21),
        sharex="col",
    )
    if len(analysis.axis_results) == 1:
        axes = [axes]

    raw_color = "#2F6BFF"
    bias_color = "#F28E2B"
    debiased_color = "#2A9D8F"
    integrated_color = "#D94841"
    static_fill_color = "#FAD7A0"

    for row_index, axis_result in enumerate(analysis.axis_results):
        raw_ax = axes[row_index][0]
        debiased_ax = axes[row_index][1]
        integrated_ax = axes[row_index][2]

        raw_ax.plot(analysis.elapsed_time_s, axis_result.raw_values, color=raw_color, linewidth=0.8)
        raw_ax.axhline(axis_result.bias_value, color=bias_color, linestyle="--", linewidth=1.0)
        raw_ax.axvspan(0.0, analysis.static_window.end_time_s, color=static_fill_color, alpha=0.28)
        raw_ax.set_ylabel(f"{axis_result.spec.short_label}\n({axis_result.spec.input_unit})")
        raw_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.75)

        debiased_ax.plot(analysis.elapsed_time_s, axis_result.debiased_values, color=debiased_color, linewidth=0.8)
        debiased_ax.axhline(0.0, color="#666666", linestyle="--", linewidth=0.9)
        debiased_ax.axvspan(0.0, analysis.static_window.end_time_s, color=static_fill_color, alpha=0.28)
        debiased_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.75)

        integrated_ax.plot(analysis.elapsed_time_s, axis_result.integrated_values, color=integrated_color, linewidth=0.8)
        integrated_ax.axvline(analysis.static_window.end_time_s, color="#999999", linestyle="--", linewidth=0.9)
        integrated_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.75)
        integrated_ax.set_ylabel(axis_result.spec.output_unit)

        if row_index == 0:
            raw_ax.set_title("Raw + Static Bias")
            debiased_ax.set_title("Debiased")
            integrated_ax.set_title(f"Integrated (dt={dt:.6f} s)")

    for column_index in range(3):
        axes[-1][column_index].set_xlabel("Elapsed Time (s)")

    fig.suptitle(
        f"{analysis.sequence}_{analysis.imu_name} static-window debias + integration\n"
        f"Static window: 0 ~ {analysis.static_window.end_time_s:.3f} s, "
        f"samples={analysis.static_window.sample_count}, detect={analysis.static_window.detection_column}",
        fontsize=14,
        y=0.995,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.985))
    fig.savefig(analysis.output_png_path, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return analysis.output_png_path


def _write_summary_csv(output_path: Path, analyses: list[ImuAnalysis], dt: float) -> Path:
    fieldnames = [
        "sequence",
        "imu_name",
        "axis",
        "sensor_family",
        "source_column",
        "static_window_samples",
        "static_window_end_time_s",
        "detection_column",
        "estimated_bias",
        "input_unit",
        "integrated_unit",
        "integration_dt_s",
        "integrated_final",
        "integrated_min",
        "integrated_max",
        "output_csv",
        "output_png",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for analysis in analyses:
            for axis_result in analysis.axis_results:
                writer.writerow(
                    {
                        "sequence": analysis.sequence,
                        "imu_name": analysis.imu_name,
                        "axis": axis_result.spec.axis,
                        "sensor_family": axis_result.spec.family,
                        "source_column": axis_result.column_name,
                        "static_window_samples": analysis.static_window.sample_count,
                        "static_window_end_time_s": f"{analysis.static_window.end_time_s:.6f}",
                        "detection_column": analysis.static_window.detection_column,
                        "estimated_bias": f"{axis_result.bias_value:.9f}",
                        "input_unit": axis_result.spec.input_unit,
                        "integrated_unit": axis_result.spec.output_unit,
                        "integration_dt_s": f"{dt:.6f}",
                        "integrated_final": f"{axis_result.integrated_values[-1]:.9f}",
                        "integrated_min": f"{min(axis_result.integrated_values):.9f}",
                        "integrated_max": f"{max(axis_result.integrated_values):.9f}",
                        "output_csv": analysis.output_csv_path.name,
                        "output_png": analysis.output_png_path.name,
                    }
                )
    return output_path


def _build_report_section(analyses: list[ImuAnalysis], summary_csv_path: Path, dt: float) -> str:
    lines: list[str] = [
        REPORT_BEGIN_MARKER,
        "",
        "## 静止段偏置去除积分分析",
        "",
        "### 方法说明",
        "",
        f"- 本章节额外对 `{len(analyses)}` 颗 IMU 的 `6` 个轴做了“前静止段估计 bias -> 去 bias -> 离散积分”的后处理。",
        "- 静止段检测统一使用每颗 IMU 的 `accel_z_g`：先取开头约 `1.5 s` 种子区间估计基线均值和标准差，再用滑动窗口寻找首个明显进入振动的区间。",
        "- 检测到的前静止段会同时用于该 IMU 的 `3` 个加速度轴和 `3` 个陀螺轴 bias 估计。",
        f"- 离散积分步长按本次分析固定使用 `dt = {dt:.6f} s`。",
        "- 加速度轴积分输出单位为 `m/s`，陀螺轴积分输出单位为 `deg`。",
        "",
        "### 前静止段检测汇总",
        "",
        "| IMU | 静止段样本数 | 静止段结束时间 (s) | 实测采样率 (Hz) | 检测参考列 | 基线均值 (g) | 基线标准差 (g) |",
        "| --- | ---: | ---: | ---: | --- | ---: | ---: |",
    ]
    for analysis in analyses:
        lines.append(
            f"| {analysis.imu_name} | {analysis.static_window.sample_count} | "
            f"{analysis.static_window.end_time_s:.6f} | {analysis.sample_rate_hz:.6f} | "
            f"{analysis.static_window.detection_column} | {analysis.static_window.baseline_mean:.9f} | "
            f"{analysis.static_window.baseline_std:.9f} |"
        )

    lines.extend(
        [
            "",
            "### 6 轴 Bias 与积分结果摘要",
            "",
            "| IMU | Axis | 输入单位 | Bias | 积分输出单位 | 末值 | 最小值 | 最大值 |",
            "| --- | --- | --- | ---: | --- | ---: | ---: | ---: |",
        ]
    )
    for analysis in analyses:
        for axis_result in analysis.axis_results:
            lines.append(
                f"| {analysis.imu_name} | {axis_result.spec.short_label} | {axis_result.spec.input_unit} | "
                f"{axis_result.bias_value:.9f} | {axis_result.spec.output_unit} | "
                f"{axis_result.integrated_values[-1]:.9f} | {min(axis_result.integrated_values):.9f} | "
                f"{max(axis_result.integrated_values):.9f} |"
            )

    lines.extend(
        [
            "",
            "### 输出文件",
            "",
            f"- 汇总 CSV: [{summary_csv_path.name}]({summary_csv_path.name})",
        ]
    )
    for analysis in analyses:
        lines.append(
            f"- `{analysis.imu_name}`: CSV [{analysis.output_csv_path.name}]({analysis.output_csv_path.name})，"
            f"总览图 [{analysis.output_png_path.name}]({analysis.output_png_path.name})"
        )

    lines.extend(["", "### 6 轴总览图", ""])
    for analysis in analyses:
        lines.extend(
            [
                f"#### {analysis.imu_name}",
                "",
                f"- 前静止段: `0 ~ {analysis.static_window.end_time_s:.6f} s`，样本 `{analysis.static_window.sample_count}`。",
                f"- 输出 CSV: [{analysis.output_csv_path.name}]({analysis.output_csv_path.name})",
                f"- 总览图: [{analysis.output_png_path.name}]({analysis.output_png_path.name})",
                "",
                f"![{analysis.imu_name} 6-axis static bias integration]({analysis.output_png_path.name})",
                "",
            ]
        )

    lines.append(REPORT_END_MARKER)
    lines.append("")
    return "\n".join(lines)


def _update_report(report_path: Path, section_text: str) -> Path:
    if report_path.exists():
        original_text = report_path.read_text(encoding="utf-8")
    else:
        original_text = "# 振动分析报告\n\n"

    begin_index = original_text.find(REPORT_BEGIN_MARKER)
    end_index = original_text.find(REPORT_END_MARKER)
    if begin_index != -1 and end_index != -1 and end_index >= begin_index:
        end_index += len(REPORT_END_MARKER)
        updated_text = original_text[:begin_index].rstrip() + "\n\n" + section_text
        trailing_text = original_text[end_index:].lstrip()
        if trailing_text:
            updated_text += "\n" + trailing_text
    else:
        updated_text = original_text.rstrip() + "\n\n" + section_text

    report_path.write_text(updated_text.rstrip() + "\n", encoding="utf-8")
    return report_path


def _find_report_path(input_dir: Path) -> Path:
    candidates = sorted(input_dir.glob("*_vibe_analysis.md"))
    if not candidates:
        return input_dir / "vibe_analysis.md"
    return candidates[0]


def _find_imu_csv_paths(input_dir: Path) -> list[Path]:
    paths = [path for path in sorted(input_dir.glob("*.csv")) if IMU_CSV_PATTERN.match(path.name)]
    if not paths:
        raise FileNotFoundError(f"no imu csv files found in {input_dir}")
    return paths


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Use the initial static window to estimate per-axis bias, then integrate all 6 axes for every IMU CSV in a vibration result directory and update the markdown report."
    )
    parser.add_argument("input_dir", help="Directory containing per-IMU vibration CSVs and the vibe markdown report")
    parser.add_argument("--dt", type=float, default=0.01, help="Discrete integration step in seconds, default: 0.01")
    parser.add_argument("--report", help="Markdown report path to update, default: auto-detect *_vibe_analysis.md")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists():
        raise FileNotFoundError(input_dir)

    imu_csv_paths = _find_imu_csv_paths(input_dir)
    analyses = [_load_imu_analysis(csv_path, dt=args.dt) for csv_path in imu_csv_paths]

    for analysis in analyses:
        _write_imu_csv(analysis)
        _plot_imu_analysis(analysis, dt=args.dt)

    summary_csv_path = input_dir / f"{analyses[0].sequence}_static_bias_6axis_summary.csv"
    _write_summary_csv(summary_csv_path, analyses, dt=args.dt)

    report_path = Path(args.report).resolve() if args.report else _find_report_path(input_dir)
    section_text = _build_report_section(analyses, summary_csv_path, dt=args.dt)
    _update_report(report_path, section_text)

    print(f"input_dir={input_dir}")
    print(f"report_path={report_path}")
    print(f"summary_csv={summary_csv_path}")
    for analysis in analyses:
        print(
            f"imu={analysis.imu_name} static_end_s={analysis.static_window.end_time_s:.6f} "
            f"samples={analysis.static_window.sample_count} csv={analysis.output_csv_path} png={analysis.output_png_path}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
