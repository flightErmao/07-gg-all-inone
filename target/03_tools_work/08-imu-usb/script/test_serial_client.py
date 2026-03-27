"""
Unit tests for serial_client.DeviceClient (no hardware required).

Tests focus on the shell-mode send_command / _read_shell_response / _strip_shell_artifacts
behaviour, especially the allow_disconnect=True path that was broken.
"""
from __future__ import annotations

import threading
import time
import unittest
from unittest.mock import MagicMock, patch, call
from serial import SerialException

from serial_client import DeviceClient


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _make_client_with_mock_serial(read_side_effect=None, read_return=None):
    """Return a DeviceClient whose internal serial port is a MagicMock."""
    client = DeviceClient.__new__(DeviceClient)
    client._lock = threading.Lock()
    mock_serial = MagicMock()
    mock_serial.is_open = True
    if read_side_effect is not None:
        mock_serial.read.side_effect = read_side_effect
    elif read_return is not None:
        mock_serial.read.return_value = read_return
    else:
        mock_serial.read.return_value = b""
    client._serial = mock_serial
    return client, mock_serial


# ---------------------------------------------------------------------------
# _strip_shell_artifacts
# ---------------------------------------------------------------------------

class TestStripShellArtifacts(unittest.TestCase):

    def _strip(self, text, cmd):
        return DeviceClient._strip_shell_artifacts(text, cmd)

    def test_removes_echo_and_prompt(self):
        text = "status\r\nOK: CDC mode\r\nmsh />"
        self.assertEqual(self._strip(text, "status"), "OK: CDC mode")

    def test_crlf_and_lf_normalized(self):
        text = "status\r\nline1\r\nline2\r\nmsh />"
        self.assertEqual(self._strip(text, "status"), "line1\nline2")

    def test_prompt_only_after_echo(self):
        # Device echoes command then disconnects – no real response body
        text = "enter_cdc\r\nmsh />"
        self.assertEqual(self._strip(text, "enter_cdc"), "")

    def test_echo_not_present(self):
        # Shell echo disabled – just response + prompt
        text = "OK\r\nmsh />"
        self.assertEqual(self._strip(text, "status"), "OK")

    def test_strips_hash_prompt(self):
        text = "status\r\nrunning\r\n# "
        self.assertEqual(self._strip(text, "status"), "running")

    def test_strips_dollar_prompt(self):
        text = "status\r\nrunning\r\n$ "
        self.assertEqual(self._strip(text, "status"), "running")

    def test_multiline_response(self):
        text = "status\r\nmode: CDC\r\nuptime: 42\r\nmsh />"
        self.assertEqual(self._strip(text, "status"), "mode: CDC\nuptime: 42")


# ---------------------------------------------------------------------------
# _read_shell_response
# ---------------------------------------------------------------------------

class TestReadShellResponse(unittest.TestCase):

    def test_returns_on_prompt(self):
        """Stops reading as soon as shell prompt is detected."""
        client, mock_serial = _make_client_with_mock_serial(
            read_side_effect=[
                b"status\r\nOK\r\nmsh />",
                b"",
            ]
        )
        result = client._read_shell_response("status", timeout=2.0)
        self.assertEqual(result, "OK")
        # Should have stopped after first read (prompt found)
        self.assertEqual(mock_serial.read.call_count, 1)

    def test_returns_after_empty_streak(self):
        """Returns after 3 consecutive empty reads once data has been received."""
        # First read returns echo only (no prompt), subsequent reads empty
        reads = [b"enter_cdc\r\n"] + [b""] * 5
        client, mock_serial = _make_client_with_mock_serial(
            read_side_effect=reads
        )
        t0 = time.time()
        result = client._read_shell_response("enter_cdc", timeout=5.0)
        elapsed = time.time() - t0
        self.assertEqual(result, "")          # echo stripped, nothing left
        self.assertLess(elapsed, 2.0)         # should NOT wait full 5 s

    def test_raises_timeout_when_no_data(self):
        """Raises TimeoutError when nothing is received at all."""
        client, mock_serial = _make_client_with_mock_serial(read_return=b"")
        with self.assertRaises(TimeoutError):
            client._read_shell_response("status", timeout=0.3)

    def test_accumulates_across_multiple_reads(self):
        """Handles response split across several read() calls."""
        client, mock_serial = _make_client_with_mock_serial(
            read_side_effect=[
                b"status\r\n",
                b"OK\r\n",
                b"msh />",
                b"",
            ]
        )
        result = client._read_shell_response("status", timeout=2.0)
        self.assertEqual(result, "OK")


# ---------------------------------------------------------------------------
# send_command – allow_disconnect behaviour (the main bug)
# ---------------------------------------------------------------------------

class TestSendCommandAllowDisconnect(unittest.TestCase):

    def _client_that_raises_on_read(self, exc):
        client, mock_serial = _make_client_with_mock_serial()
        mock_serial.reset_input_buffer = MagicMock()
        mock_serial.write = MagicMock()
        mock_serial.flush = MagicMock()
        mock_serial.read.side_effect = exc
        return client

    def test_serial_exception_returns_empty_when_allow_disconnect(self):
        """SerialException during read returns '' when allow_disconnect=True."""
        client = self._client_that_raises_on_read(SerialException("disconnected"))
        result = client.send_command("enter_cdc", timeout=0.5, allow_disconnect=True)
        self.assertEqual(result, "")

    def test_timeout_error_returns_empty_when_allow_disconnect(self):
        """
        TimeoutError (buf empty, no response) must also return '' when
        allow_disconnect=True.  This was the bug: only SerialException was caught.
        """
        # All reads return empty → _read_shell_response raises TimeoutError
        client, mock_serial = _make_client_with_mock_serial(read_return=b"")
        mock_serial.reset_input_buffer = MagicMock()
        mock_serial.write = MagicMock()
        mock_serial.flush = MagicMock()
        result = client.send_command("enter_cdc", timeout=0.3, allow_disconnect=True)
        self.assertEqual(result, "")

    def test_serial_exception_re_raised_without_allow_disconnect(self):
        """SerialException is re-raised when allow_disconnect=False."""
        client = self._client_that_raises_on_read(SerialException("disconnected"))
        with self.assertRaises(SerialException):
            client.send_command("status", timeout=0.5, allow_disconnect=False)

    def test_timeout_error_re_raised_without_allow_disconnect(self):
        """TimeoutError is re-raised when allow_disconnect=False."""
        client, mock_serial = _make_client_with_mock_serial(read_return=b"")
        mock_serial.reset_input_buffer = MagicMock()
        mock_serial.write = MagicMock()
        mock_serial.flush = MagicMock()
        with self.assertRaises(TimeoutError):
            client.send_command("status", timeout=0.3, allow_disconnect=False)

    def test_normal_status_command_returns_response(self):
        """Normal command with response works correctly."""
        client, mock_serial = _make_client_with_mock_serial(
            read_side_effect=[b"status\r\nOK: CDC\r\nmsh />", b""]
        )
        mock_serial.reset_input_buffer = MagicMock()
        mock_serial.write = MagicMock()
        mock_serial.flush = MagicMock()
        result = client.send_command("status", timeout=2.0)
        self.assertEqual(result, "OK: CDC")

    def test_command_sent_with_crlf(self):
        """Command must be sent with \\r\\n terminator for RTOS shell."""
        client, mock_serial = _make_client_with_mock_serial(
            read_side_effect=[b"status\r\nmsh />", b""]
        )
        mock_serial.reset_input_buffer = MagicMock()
        mock_serial.write = MagicMock()
        mock_serial.flush = MagicMock()
        client.send_command("status", timeout=2.0)
        mock_serial.write.assert_called_once_with(b"status\r\n")


if __name__ == "__main__":
    unittest.main(verbosity=2)
