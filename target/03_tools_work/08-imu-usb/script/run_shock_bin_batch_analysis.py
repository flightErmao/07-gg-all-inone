from __future__ import annotations

import argparse
import time
from pathlib import Path

from imu_usb_tool import ANALYSIS_OUTPUT_DIRS, analyze_real_imu_bin_file


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Batch analyze all BIN files in a folder with the shock-recovery pipeline."
    )
    parser.add_argument("input_dir", help="Directory containing one or more .BIN files")
    parser.add_argument(
        "--output-root",
        help="Output root directory, default: data/05_shock",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists():
        raise FileNotFoundError(input_dir)

    output_root = Path(args.output_root).resolve() if args.output_root else ANALYSIS_OUTPUT_DIRS["shock"].resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    bin_paths = sorted(input_dir.glob("*.BIN")) + sorted(input_dir.glob("*.bin"))
    unique_paths: list[Path] = []
    seen: set[str] = set()
    for path in bin_paths:
        key = str(path.resolve()).lower()
        if key in seen:
            continue
        seen.add(key)
        unique_paths.append(path.resolve())

    if not unique_paths:
        raise FileNotFoundError(f"no BIN files found in {input_dir}")

    batch_stamp = time.strftime("%Y%m%d_%H%M%S")
    for source in unique_paths:
        output_dir = output_root / f"{source.stem}_{batch_stamp}"
        output_dir.mkdir(parents=True, exist_ok=True)
        print(f"[shock] start: {source}")
        print(f"[shock] output: {output_dir}")
        csv_path, packet_csv_path, md_path, _summary = analyze_real_imu_bin_file(
            "shock",
            source,
            output_dir=output_dir,
            progress_callback=lambda message: print(f"  {message}"),
        )
        print(f"[shock] comparison_csv: {csv_path}")
        if packet_csv_path != csv_path:
            print(f"[shock] packet_csv: {packet_csv_path}")
        print(f"[shock] report: {md_path}")
        print("")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
