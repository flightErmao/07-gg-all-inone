from __future__ import annotations

import unittest

from imu_usb_tool import parse_noise_start_response, parse_noise_status_response


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
