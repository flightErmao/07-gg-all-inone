from __future__ import annotations

import threading
import time
from dataclasses import dataclass

try:
    import serial
    import serial.tools.list_ports
    from serial import SerialException
    from serial import SerialTimeoutException
except ModuleNotFoundError as exc:
    if exc.name != "serial":
        raise
    raise ModuleNotFoundError(
        "Missing dependency 'pyserial'. Install it with:\n"
        "python -m pip install -r requirements.txt\n"
        "or:\n"
        "python -m pip install pyserial"
    ) from exc


@dataclass(frozen=True)
class PortInfo:
    device: str
    description: str
    manufacturer: str
    hwid: str
    vid: int | None
    pid: int | None

    @property
    def label(self) -> str:
        description = self.description or "Unknown"
        return f"{self.device} - {description}"


class DeviceClient:
    def __init__(self) -> None:
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()

    @staticmethod
    def list_port_infos() -> list[PortInfo]:
        ports: list[PortInfo] = []
        for port in serial.tools.list_ports.comports():
            ports.append(
                PortInfo(
                    device=port.device,
                    description=port.description or "",
                    manufacturer=port.manufacturer or "",
                    hwid=port.hwid or "",
                    vid=port.vid,
                    pid=port.pid,
                )
            )
        ports.sort(key=lambda item: item.device)
        return ports

    @staticmethod
    def find_preferred_port(vid: int = 0x0FFE, pid: int | tuple[int, ...] | None = 0x0001) -> PortInfo | None:
        for port in DeviceClient.list_port_infos():
            if port.vid != vid:
                continue
            if pid is None:
                return port
            if isinstance(pid, tuple):
                if port.pid in pid:
                    return port
            elif port.pid == pid:
                return port
        return None

    def open(self, port: str, baudrate: int = 115200) -> None:
        self.close()
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.2,
            write_timeout=5.0,
        )
        time.sleep(0.2)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    # Shell prompt suffixes used by common RTOS shells (RT-Thread MSH, etc.)
    _SHELL_PROMPTS: tuple[bytes, ...] = (
        b"msh />",
        b"msh>",
        b"finsh>",
        b"> ",
        b"$ ",
        b"# ",
    )

    def send_command(self, command: str, timeout: float = 2.0, allow_disconnect: bool = False) -> str:
        with self._lock:
            serial_port = self._require_serial()
            attempts = 2
            for attempt in range(attempts):
                try:
                    self._prepare_shell(serial_port)
                    # Shell requires \r\n; plain \n is ignored by most RTOS shells
                    serial_port.write((command.strip() + "\r\n").encode("ascii"))
                    serial_port.flush()
                    return self._read_shell_response(command.strip(), timeout)
                except SerialTimeoutException:
                    if attempt + 1 >= attempts:
                        if allow_disconnect:
                            return ""
                        raise
                    time.sleep(0.2)
                    continue
                except (SerialException, TimeoutError):
                    # TimeoutError can occur when the device disconnects before
                    # returning a shell prompt (e.g. enter_cdc / enter_msc).
                    if allow_disconnect:
                        return ""
                    raise
            return ""

    def _prepare_shell(self, serial_port: serial.Serial) -> None:
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()
        serial_port.write(b"\r\n")
        serial_port.flush()
        self._drain_until_idle(serial_port, 0.25)
        serial_port.reset_input_buffer()

    @staticmethod
    def _drain_until_idle(serial_port: serial.Serial, timeout: float) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = serial_port.read(256)
            if not chunk:
                break

    def _read_shell_response(self, sent_command: str, timeout: float) -> str:
        """Read until a shell prompt appears or timeout, then strip echo + prompt."""
        serial_port = self._require_serial()
        deadline = time.time() + timeout
        buf = bytearray()
        # How many consecutive empty reads we've seen after already receiving data.
        # serial timeout=0.2s, so 3 empty reads ≈ 0.6 s of silence → device is done.
        empty_streak = 0

        while time.time() < deadline:
            chunk = serial_port.read(256)
            if chunk:
                buf.extend(chunk)
                empty_streak = 0
                # Stop as soon as the buffer ends with a known prompt
                if any(buf.endswith(p) for p in self._SHELL_PROMPTS):
                    break
            else:
                # No new data
                if buf and any(p in buf for p in self._SHELL_PROMPTS):
                    break
                if buf:
                    empty_streak += 1
                    # After receiving data, 3 silent reads means the response is
                    # complete (or the device disconnected without sending a prompt)
                    if empty_streak >= 3:
                        break
                time.sleep(0.01)

        if not buf:
            raise TimeoutError("timed out waiting for device response")

        return self._strip_shell_artifacts(buf.decode("ascii", errors="replace"), sent_command)

    @staticmethod
    def _strip_shell_artifacts(text: str, sent_command: str) -> str:
        """Remove shell echo of the sent command and trailing prompt lines."""
        text = text.replace("\r\n", "\n").replace("\r", "\n")
        result_lines: list[str] = []
        echo_removed = False
        for line in text.split("\n"):
            stripped = line.strip()
            # Remove the first occurrence of the echoed command
            if not echo_removed and stripped == sent_command:
                echo_removed = True
                continue
            # Remove prompt lines (end with >, $, or #)
            if stripped.endswith(">") or stripped.endswith("$") or stripped.endswith("#"):
                continue
            result_lines.append(stripped)
        return "\n".join(line for line in result_lines if line).strip()

    def _require_serial(self) -> serial.Serial:
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("serial port is not connected")
        return self._serial
