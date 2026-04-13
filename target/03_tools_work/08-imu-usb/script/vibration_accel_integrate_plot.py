from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path


GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class SamplePoint:
    sample_index: int
    source_row_index: int
    accel_g: float
    packet_timestamp_continuous: int | None


def _filename_token(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text.replace("-", "m").replace(".", "p") or "0"


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def _std(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = _mean(values)
    variance = sum((value - avg) ** 2 for value in values) / float(len(values))
    return variance ** 0.5


def _build_default_output_stem(input_stem: str, accel_column: str, dt: float) -> str:
    accel_suffix = accel_column
    for marker in ("_accel_", "_accl_"):
        marker_index = accel_column.lower().find(marker)
        if marker_index > 0:
            column_prefix = accel_column[:marker_index]
            if input_stem.endswith(column_prefix):
                accel_suffix = accel_column[marker_index + 1 :]
            break
    return f"{input_stem}_{accel_suffix}_debiased_integrated_dt_{_filename_token(dt)}"


def _resolve_column(fieldnames: list[str], requested: str | None) -> str:
    normalized = {name.lower(): name for name in fieldnames}
    if requested:
        exact = normalized.get(requested.lower())
        if exact:
            return exact
        suffix_matches = [name for name in fieldnames if name.lower().endswith(requested.lower())]
        if len(suffix_matches) == 1:
            return suffix_matches[0]
        raise ValueError(f"column not found: {requested}")

    for suffix in ("_accel_z_g", "_accl_g", "_accel_g"):
        matches = [name for name in fieldnames if name.lower().endswith(suffix)]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError(f"multiple columns match suffix {suffix}: {matches}")

    raise ValueError("unable to auto-detect accel column, please pass --column")


def _resolve_timestamp_column(fieldnames: list[str], accel_column: str) -> str | None:
    prefix = accel_column[:-len("accel_z_g")] if accel_column.lower().endswith("accel_z_g") else ""
    preferred = f"{prefix}packet_timestamp_continuous" if prefix else ""
    if preferred and preferred in fieldnames:
        return preferred

    matches = [name for name in fieldnames if name.lower().endswith("packet_timestamp_continuous")]
    if len(matches) == 1:
        return matches[0]
    return None


def _load_samples(csv_path: Path, requested_column: str | None) -> tuple[str, str | None, list[SamplePoint]]:
    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        if not fieldnames:
            raise ValueError(f"csv has no header: {csv_path}")

        accel_column = _resolve_column(fieldnames, requested_column)
        timestamp_column = _resolve_timestamp_column(fieldnames, accel_column)

        samples: list[SamplePoint] = []
        for row_index, row in enumerate(reader, start=2):
            raw_value = str(row.get(accel_column, "") or "").strip()
            if not raw_value:
                continue

            timestamp_value = None
            if timestamp_column:
                raw_timestamp = str(row.get(timestamp_column, "") or "").strip()
                if raw_timestamp:
                    timestamp_value = int(float(raw_timestamp))

            samples.append(
                SamplePoint(
                    sample_index=len(samples),
                    source_row_index=row_index,
                    accel_g=float(raw_value),
                    packet_timestamp_continuous=timestamp_value,
                )
            )

    if not samples:
        raise ValueError(f"no valid data found in column {requested_column or '<auto>'}")

    return accel_column, timestamp_column, samples


def _estimate_sample_rate_hz(samples: list[SamplePoint]) -> float:
    diffs = [
        float(curr.packet_timestamp_continuous - prev.packet_timestamp_continuous)
        for prev, curr in zip(samples[:-1], samples[1:])
        if prev.packet_timestamp_continuous is not None
        and curr.packet_timestamp_continuous is not None
        and curr.packet_timestamp_continuous > prev.packet_timestamp_continuous
    ]
    if not diffs:
        return 0.0
    mean_diff_us = _mean(diffs)
    if mean_diff_us <= 0.0:
        return 0.0
    return 1_000_000.0 / mean_diff_us


def _detect_static_window_end_index(samples: list[SamplePoint]) -> int:
    if len(samples) < 64:
        return max(len(samples) - 1, 0)

    values = [sample.accel_g for sample in samples]
    sample_rate_hz = _estimate_sample_rate_hz(samples)
    seed_count = min(max(int(sample_rate_hz * 1.5), 1000), max(len(samples) // 4, 256))
    seed_count = min(seed_count, len(samples))
    seed_values = values[:seed_count]
    baseline_mean = _mean(seed_values)
    baseline_std = _std(seed_values)

    window_size = min(max(int(sample_rate_hz * 0.2), 160), max(len(samples) // 6, 64))
    step_size = max(window_size // 10, 10)
    threshold_std = max(baseline_std * 5.0, 0.15)
    threshold_peak = max(baseline_std * 10.0, 0.8)
    onset_index = seed_count

    for start in range(seed_count, len(samples) - window_size, step_size):
        window = values[start : start + window_size]
        window_std = _std(window)
        peak = max(abs(value - baseline_mean) for value in window)
        if window_std > threshold_std or peak > threshold_peak:
            onset_index = start
            break

    refine_threshold = max(baseline_std * 6.0, 0.4)
    refine_start = max(seed_count, onset_index - window_size)
    for index in range(refine_start, onset_index):
        if abs(values[index] - baseline_mean) >= refine_threshold:
            onset_index = index
            break

    return max(min(onset_index - 1, len(samples) - 1), 0)


def _estimate_bias_g(samples: list[SamplePoint]) -> tuple[float, int]:
    end_index = _detect_static_window_end_index(samples)
    return _mean([sample.accel_g for sample in samples[: end_index + 1]]), end_index


def _remove_bias_from_accel(samples: list[SamplePoint], bias_g: float) -> list[float]:
    return [sample.accel_g - bias_g for sample in samples]


def _integrate_accel_to_velocity(accel_g_values: list[float], dt: float) -> list[float]:
    if dt <= 0.0:
        raise ValueError("dt must be > 0")

    velocity_m_s: list[float] = []
    current_velocity = 0.0
    for index, accel_g in enumerate(accel_g_values):
        if index == 0:
            velocity_m_s.append(current_velocity)
            continue
        current_velocity += accel_g * GRAVITY_MPS2 * dt
        velocity_m_s.append(current_velocity)
    return velocity_m_s


def _build_elapsed_time_s(samples: list[SamplePoint], dt: float) -> list[float]:
    if samples and all(sample.packet_timestamp_continuous is not None for sample in samples):
        start = int(samples[0].packet_timestamp_continuous or 0)
        return [((int(sample.packet_timestamp_continuous or 0) - start) / 1_000_000.0) for sample in samples]
    return [sample.sample_index * dt for sample in samples]


def _write_output_csv(
    output_csv: Path,
    accel_column: str,
    timestamp_column: str | None,
    dt: float,
    bias_g: float,
    bias_window_end_index: int,
    elapsed_time_s: list[float],
    samples: list[SamplePoint],
    debiased_accel_g: list[float],
    velocity_m_s: list[float],
) -> Path:
    velocity_g_s = [value / GRAVITY_MPS2 for value in velocity_m_s]
    fieldnames = [
        "sample_index",
        "time_s",
        "source_row_index",
        "estimated_bias_g",
        "bias_window_sample_count",
        "bias_window_end_time_s",
    ]
    if timestamp_column:
        fieldnames.append(timestamp_column)
    fieldnames.extend(
        [
            accel_column,
            f"{accel_column}_debiased_g",
            f"{accel_column}_debiased_integrated_g_s",
            f"{accel_column}_debiased_integrated_velocity_m_s",
        ]
    )

    with output_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample, corrected_accel_g, integrated_g_s, integrated_velocity in zip(
            samples,
            debiased_accel_g,
            velocity_g_s,
            velocity_m_s,
        ):
            row = {
                "sample_index": sample.sample_index,
                "time_s": f"{elapsed_time_s[sample.sample_index]:.6f}",
                "source_row_index": sample.source_row_index,
                "estimated_bias_g": f"{bias_g:.9f}",
                "bias_window_sample_count": bias_window_end_index + 1,
                "bias_window_end_time_s": f"{elapsed_time_s[bias_window_end_index]:.6f}",
                accel_column: f"{sample.accel_g:.9f}",
                f"{accel_column}_debiased_g": f"{corrected_accel_g:.9f}",
                f"{accel_column}_debiased_integrated_g_s": f"{integrated_g_s:.9f}",
                f"{accel_column}_debiased_integrated_velocity_m_s": f"{integrated_velocity:.9f}",
            }
            if timestamp_column:
                row[timestamp_column] = (
                    "" if sample.packet_timestamp_continuous is None else str(sample.packet_timestamp_continuous)
                )
            writer.writerow(row)

    return output_csv


def _plot_results(
    output_png: Path,
    csv_name: str,
    accel_column: str,
    dt: float,
    bias_g: float,
    bias_window_end_index: int,
    elapsed_time_s: list[float],
    samples: list[SamplePoint],
    debiased_accel_g: list[float],
    velocity_m_s: list[float],
) -> Path:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"matplotlib unavailable: {exc}") from exc

    time_s = elapsed_time_s
    accel_g = [sample.accel_g for sample in samples]

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    axes[0].plot(time_s, accel_g, color="#2F6BFF", linewidth=0.9)
    axes[0].axhline(bias_g, color="#F28E2B", linewidth=1.2, linestyle="--", label=f"Bias={bias_g:.6f} g")
    axes[0].axvspan(0.0, time_s[bias_window_end_index], color="#FAD7A0", alpha=0.28, label="Static window")
    axes[0].set_ylabel("Accel (g)")
    axes[0].set_title(f"{csv_name}: {accel_column}")
    axes[0].grid(True, linestyle=":", linewidth=0.6, alpha=0.8)
    axes[0].legend(loc="upper right")

    axes[1].plot(time_s, debiased_accel_g, color="#2A9D8F", linewidth=0.9)
    axes[1].axhline(0.0, color="#666666", linewidth=1.0, linestyle="--")
    axes[1].axvspan(0.0, time_s[bias_window_end_index], color="#FAD7A0", alpha=0.28)
    axes[1].set_ylabel("Debiased Accel (g)")
    axes[1].set_title("Bias removed acceleration")
    axes[1].grid(True, linestyle=":", linewidth=0.6, alpha=0.8)

    axes[2].plot(time_s, velocity_m_s, color="#D94841", linewidth=0.9)
    axes[2].axvline(time_s[bias_window_end_index], color="#999999", linewidth=1.0, linestyle="--")
    axes[2].set_xlabel(f"Integrated Time (dt={dt:.6f} s)")
    axes[2].set_ylabel("Velocity (m/s)")
    axes[2].set_title("Debiased integrated result: y[k] = y[k-1] + (x[k] - bias) * dt")
    axes[2].grid(True, linestyle=":", linewidth=0.6, alpha=0.8)

    fig.tight_layout()
    fig.savefig(output_png, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return output_png


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Estimate accel bias, remove it, then integrate to velocity and export an independent CSV + plot."
    )
    parser.add_argument("input_csv", help="Source CSV path")
    parser.add_argument(
        "--column",
        help="Accel column to integrate. If omitted, the script auto-detects *_accel_z_g first.",
    )
    parser.add_argument("--dt", type=float, default=0.01, help="Integration step in seconds, default: 0.01")
    parser.add_argument("--output-dir", help="Output directory, default: same directory as input CSV")
    parser.add_argument(
        "--output-stem",
        help="Output file stem. Default: <input_stem>_<column>_integrated_dt_<token>",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    input_csv = Path(args.input_csv).resolve()
    if not input_csv.exists():
        raise FileNotFoundError(input_csv)

    output_dir = Path(args.output_dir).resolve() if args.output_dir else input_csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    accel_column, timestamp_column, samples = _load_samples(input_csv, args.column)
    bias_g, bias_window_end_index = _estimate_bias_g(samples)
    elapsed_time_s = _build_elapsed_time_s(samples, args.dt)
    debiased_accel_g = _remove_bias_from_accel(samples, bias_g)
    velocity_m_s = _integrate_accel_to_velocity(debiased_accel_g, args.dt)

    stem = args.output_stem or _build_default_output_stem(input_csv.stem, accel_column, args.dt)
    output_csv = output_dir / f"{stem}.csv"
    output_png = output_dir / f"{stem}.png"

    _write_output_csv(
        output_csv,
        accel_column,
        timestamp_column,
        args.dt,
        bias_g,
        bias_window_end_index,
        elapsed_time_s,
        samples,
        debiased_accel_g,
        velocity_m_s,
    )
    _plot_results(
        output_png,
        input_csv.name,
        accel_column,
        args.dt,
        bias_g,
        bias_window_end_index,
        elapsed_time_s,
        samples,
        debiased_accel_g,
        velocity_m_s,
    )

    print(f"input_csv={input_csv}")
    print(f"column={accel_column}")
    print(f"timestamp_column={timestamp_column or '-'}")
    print(f"samples={len(samples)}")
    print(f"estimated_bias_g={bias_g}")
    print(f"bias_window_sample_count={bias_window_end_index + 1}")
    print(f"dt_s={args.dt}")
    print(f"output_csv={output_csv}")
    print(f"output_png={output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
