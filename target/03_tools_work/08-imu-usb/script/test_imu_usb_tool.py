from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from imu_usb_tool import (
    _aggregate_bias_run_records,
    _analyze_bias_single_imu_csv,
    _analyze_vibe_per_imu_from_csvs,
    _build_vibe_markdown_lines,
    _collect_poll_comparison_stats,
    _packet_s16,
    _packet_timestamp_u16,
    _write_vibe_summary_csv,
    parse_noise_start_response,
    parse_noise_status_response,
)


class TestParseNoiseStatusResponse(unittest.TestCase):

    def test_parses_start_response(self) -> None:
        response = (
            "ACK cmd=noise_test_start received\r\n"
            "RESULT cmd=noise_test_start status=accepted detected=4 duration_s=600 "
            "dir=0:/03_arw file=0:/03_arw/141.BIN index=141\r\n"
        )

        result = parse_noise_start_response(response)

        self.assertEqual(result.status, "accepted")
        self.assertEqual(result.detected, 4)
        self.assertEqual(result.duration_s, 600)
        self.assertEqual(result.dir, "0:/03_arw")
        self.assertEqual(result.file, "0:/03_arw/141.BIN")
        self.assertEqual(result.index, 141)

    def test_parses_clean_response(self) -> None:
        response = (
            "ACK cmd=noise_test_status received\r\n"
            "NOISE_TEST_STATUS recording=1 frames=123 duration_s=310 file=03_arw/059.BIN last_error=0\r\n"
            "RESULT cmd=noise_test_status status=ok\r\n"
        )

        status = parse_noise_status_response(response)

        self.assertEqual(status.recording, 1)
        self.assertEqual(status.frames, 123)
        self.assertEqual(status.duration_s, 310)
        self.assertEqual(status.file, "03_arw/059.BIN")
        self.assertEqual(status.flushes, 0)
        self.assertEqual(status.max_gap_ms, 0)
        self.assertEqual(status.last_error, 0)
        self.assertEqual(status.last_run_ok, 0)
        self.assertEqual(status.status, "ok")

    def test_parses_response_with_nul_between_fields(self) -> None:
        response = (
            "ACK cmd=noise_test_status received\r\n"
            "NOISE_TEST_STATUS recording=1 frames=123 duration_s=310 "
            "file=03_arw/059.BIN\x00RESULT cmd=noise_test_status status=ok\r\n"
            "last_error=0\r\n"
        )

        status = parse_noise_status_response(response)

        self.assertEqual(status.max_gap_ms, 0)
        self.assertEqual(status.last_error, 0)
        self.assertEqual(status.last_run_ok, 0)


class TestBiasAnalysis(unittest.TestCase):

    def test_45686_fifo_packet_values_are_decoded_as_little_endian(self) -> None:
        packet = bytes.fromhex("68 DA 01 6E F8 BB 01 00 00 FF FF FD FF 00 83 7D")

        self.assertEqual(_packet_s16(packet, 1, 3), 474)
        self.assertEqual(_packet_s16(packet, 3, 3), -1938)
        self.assertEqual(_packet_s16(packet, 5, 3), 443)
        self.assertEqual(_packet_s16(packet, 7, 3), 0)
        self.assertEqual(_packet_s16(packet, 9, 3), -1)
        self.assertEqual(_packet_s16(packet, 11, 3), -3)
        self.assertEqual(_packet_timestamp_u16(packet, 3), 32131)

    def _write_42688_bias_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "packet_index",
                    "42688_A_poll_timestamp_us",
                    "42688_A_packet_index_in_capture",
                    "42688_A_packet_size",
                    "42688_A_packet_timestamp_u16",
                    "42688_A_packet_timestamp_continuous",
                    "42688_A_fifo_header",
                    "42688_A_raw_accel_x",
                    "42688_A_raw_accel_y",
                    "42688_A_raw_accel_z",
                    "42688_A_raw_gyro_x",
                    "42688_A_raw_gyro_y",
                    "42688_A_raw_gyro_z",
                    "42688_A_temp_raw",
                    "42688_A_accel_x_g",
                    "42688_A_accel_y_g",
                    "42688_A_accel_z_g",
                    "42688_A_gyro_x_deg_s",
                    "42688_A_gyro_y_deg_s",
                    "42688_A_gyro_z_deg_s",
                    "42688_A_temp_c",
                ]
            )
            writer.writerows(
                [
                    [1, 100000, 1, 16, 0, 0, 104, 0, 0, 0, 0, 0, 0, 25, 0.001, -0.001, 0.100, 1.0, -1.0, 0.10, 25.0],
                    [2, 101000, 1, 16, 1000, 1000, 104, 0, 0, 0, 0, 0, 0, 26, 0.002, -0.002, 0.101, 3.0, -3.0, 0.20, 26.0],
                    [3, 102000, 1, 16, 2000, 2000, 104, 0, 0, 0, 0, 0, 0, 27, 0.003, -0.003, 0.102, 5.0, -5.0, 0.30, 27.0],
                ]
            )

    def test_single_imu_bias_stats_use_sample_std(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "demo_42688_A.csv"
            self._write_42688_bias_csv(csv_path)

            report = _analyze_bias_single_imu_csv(1, csv_path)

            self.assertEqual(report["sample_count"], 3)
            self.assertAlmostEqual(float(report["duration_s"]), 0.002, places=6)
            self.assertAlmostEqual(report["accel"]["x"]["mean_mg"], 2.0, places=6)
            self.assertAlmostEqual(report["accel"]["x"]["std_mg"], 1.0, places=6)
            self.assertAlmostEqual(report["accel"]["x"]["range_mg"], 2.0, places=6)
            self.assertAlmostEqual(report["gyro"]["x"]["mean_deg_s"], 3.0, places=6)
            self.assertAlmostEqual(report["gyro"]["x"]["std_deg_s"], 2.0, places=6)
            self.assertAlmostEqual(report["gyro"]["x"]["range_deg_s"], 4.0, places=6)
            self.assertAlmostEqual(float(report["temp_std_c"]), 1.0, places=6)

    def test_bias_aggregate_uses_run_means_for_power_cycle_metrics(self) -> None:
        run_records = [
            {
                "run_index": 1,
                "imu_id": 1,
                "imu_name": "42688_A",
                "family": "42688",
                "sample_count": 100,
                "duration_s": 60.0,
                "temp_mean_c": 25.0,
                "acc_mean_x_mg": 10.0,
                "acc_mean_y_mg": 20.0,
                "acc_mean_z_mg": 30.0,
                "acc_std_x_mg": 1.0,
                "acc_std_y_mg": 2.0,
                "acc_std_z_mg": 3.0,
                "acc_range_x_mg": 4.0,
                "acc_range_y_mg": 5.0,
                "acc_range_z_mg": 6.0,
                "gyro_mean_x_deg_s": 0.10,
                "gyro_mean_y_deg_s": 0.20,
                "gyro_mean_z_deg_s": 0.30,
                "gyro_std_x_deg_s": 0.01,
                "gyro_std_y_deg_s": 0.02,
                "gyro_std_z_deg_s": 0.03,
                "gyro_range_x_deg_s": 0.04,
                "gyro_range_y_deg_s": 0.05,
                "gyro_range_z_deg_s": 0.06,
            },
            {
                "run_index": 2,
                "imu_id": 1,
                "imu_name": "42688_A",
                "family": "42688",
                "sample_count": 100,
                "duration_s": 60.0,
                "temp_mean_c": 27.0,
                "acc_mean_x_mg": 14.0,
                "acc_mean_y_mg": 18.0,
                "acc_mean_z_mg": 36.0,
                "acc_std_x_mg": 3.0,
                "acc_std_y_mg": 4.0,
                "acc_std_z_mg": 5.0,
                "acc_range_x_mg": 8.0,
                "acc_range_y_mg": 9.0,
                "acc_range_z_mg": 10.0,
                "gyro_mean_x_deg_s": 0.18,
                "gyro_mean_y_deg_s": 0.22,
                "gyro_mean_z_deg_s": 0.26,
                "gyro_std_x_deg_s": 0.03,
                "gyro_std_y_deg_s": 0.04,
                "gyro_std_z_deg_s": 0.05,
                "gyro_range_x_deg_s": 0.08,
                "gyro_range_y_deg_s": 0.09,
                "gyro_range_z_deg_s": 0.10,
            },
        ]

        aggregate = _aggregate_bias_run_records(run_records)
        imu = aggregate[1]

        self.assertEqual(imu["run_count"], 2)
        self.assertAlmostEqual(imu["accel"]["x"]["mean_mg"], 12.0, places=6)
        self.assertAlmostEqual(imu["accel"]["x"]["std_mg"], 2.8284271247, places=6)
        self.assertAlmostEqual(imu["accel"]["x"]["delta_mg"], 4.0, places=6)
        self.assertAlmostEqual(imu["accel"]["x"]["within_std_mean_mg"], 2.0, places=6)
        self.assertAlmostEqual(imu["gyro"]["x"]["mean_deg_s"], 0.14, places=6)
        self.assertAlmostEqual(imu["gyro"]["x"]["std_deg_s"], 0.0565685425, places=6)
        self.assertAlmostEqual(imu["gyro"]["x"]["delta_deg_s"], 0.08, places=6)


class TestVibeAnalysis(unittest.TestCase):

    def _write_45686_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "packet_index",
                    "45686_A_poll_timestamp_us",
                    "45686_A_packet_index_in_capture",
                    "45686_A_packet_size",
                    "45686_A_packet_timestamp_u16",
                    "45686_A_packet_timestamp_continuous",
                    "45686_A_fifo_header",
                    "45686_A_raw_accel_x",
                    "45686_A_raw_accel_y",
                    "45686_A_raw_accel_z",
                    "45686_A_raw_gyro_x",
                    "45686_A_raw_gyro_y",
                    "45686_A_raw_gyro_z",
                    "45686_A_temp_raw",
                    "45686_A_accel_x_g",
                    "45686_A_accel_y_g",
                    "45686_A_accel_z_g",
                    "45686_A_gyro_x_deg_s",
                    "45686_A_gyro_y_deg_s",
                    "45686_A_gyro_z_deg_s",
                    "45686_A_temp_c",
                ]
            )
            rows = [
                [1, 100000, 1, 16, 0, 0, 104, 10, 20, 30, 40, 50, 60, 24, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                [2, 100000, 2, 16, 625, 625, 104, 11, 21, 31, 41, 51, 61, 24, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                [3, 101000, 1, 16, 1250, 1250, 104, 12, 22, 32, 42, 52, 62, 24, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                [4, 103000, 1, 16, 2500, 2500, 104, 32767, 0, 0, 0, -32768, 0, 24, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                [5, 103000, 2, 16, 2500, 2500, 104, 32767, 0, 0, 0, -32768, 0, 24, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                [6, 103000, 3, 16, 3125, 3125, 104, 32767, 0, 0, 0, -32768, 0, 24, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
            ]
            writer.writerows(rows)

    def _write_poll_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "poll_count",
                    "45686_A_pkt_count",
                    "45686_A_accel_x_g",
                    "45686_A_accel_y_g",
                    "45686_A_accel_z_g",
                    "45686_A_gyro_x_deg_s",
                    "45686_A_gyro_y_deg_s",
                    "45686_A_gyro_z_deg_s",
                    "45686_A_temp_c",
                ]
            )
            writer.writerows(
                [
                    [100, 2, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                    [101, 1, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 24.0],
                    [103, 0, "", "", "", "", "", "", ""],
                ]
            )

    def test_vibe_analysis_detects_continuity_issues(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            imu_csv = temp_path / "demo_45686_A.csv"
            poll_csv = temp_path / "demo_poll_compare.csv"
            self._write_45686_csv(imu_csv)
            self._write_poll_csv(poll_csv)

            poll_stats = _collect_poll_comparison_stats(poll_csv)
            report = _analyze_vibe_per_imu_from_csvs({3: imu_csv}, poll_stats)
            sr = report[3]

            self.assertEqual(poll_stats["poll_gap_count"], 1)
            self.assertEqual(poll_stats["missing_poll_count"], 1)
            self.assertEqual(sr["timestamp_gap_count"], 1)
            self.assertEqual(sr["estimated_missing_samples"], 1)
            self.assertEqual(sr["duplicate_timestamp_count"], 1)
            self.assertEqual(sr["longest_identical_run_samples"], 3)
            self.assertEqual(sr["zero_packet_poll_count"], 1)
            self.assertEqual(sr["continuity_status"], "失败")
            self.assertGreater(float(sr["saturated_packet_ratio_pct"]), 0.0)

    def test_vibe_markdown_and_summary_csv_include_key_findings(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            imu_csv = temp_path / "demo_45686_A.csv"
            poll_csv = temp_path / "demo_poll_compare.csv"
            source = temp_path / "demo.BIN"
            source.write_bytes(b"\x00" * 1024)
            self._write_45686_csv(imu_csv)
            self._write_poll_csv(poll_csv)

            poll_stats = _collect_poll_comparison_stats(poll_csv)
            report = _analyze_vibe_per_imu_from_csvs({3: imu_csv}, poll_stats)
            md_path = temp_path / "demo_vibe_analysis.md"
            summary_csv = _write_vibe_summary_csv(md_path, report)

            summary = {
                "test": "vibe",
                "source_size_text": "1.00 KB",
                "poll_row_count": poll_stats["poll_row_count"],
                "poll_total_duration_s": poll_stats["poll_total_duration_s"],
                "poll_gap_count": poll_stats["poll_gap_count"],
                "missing_poll_count": poll_stats["missing_poll_count"],
                "poll_reversal_count": poll_stats["poll_reversal_count"],
                "comparison_csv": poll_csv.name,
                "per_imu_csvs": imu_csv.name,
                "vibe_summary_csv": summary_csv.name,
                "per_imu_report": report,
            }
            markdown = "\n".join(_build_vibe_markdown_lines(source, summary, md_path))

            self.assertIn("# 振动环境输出连续性分析结果", markdown)
            self.assertIn("估算丢样", markdown)
            self.assertIn("满量程裁剪", markdown)
            self.assertTrue(summary_csv.exists())
            self.assertIn("continuity_status", summary_csv.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
