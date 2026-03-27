from __future__ import annotations

import queue
import shutil
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from imu_usb_tool import (
    DEFAULT_BAUDRATE,
    DEFAULT_LOG_FILE,
    DEFAULT_WAIT_SECONDS,
    find_cdc_port,
    find_new_drive,
    jlink_reset,
    snapshot_volumes,
    wait_for_any_target_port,
    wait_for_mode_transition,
    wait_for_port_presence,
)
from serial_client import DeviceClient, PortInfo


APP_ROOT = Path(__file__).resolve().parent
DOWNLOAD_ROOT = APP_ROOT / "downloads"
APP_LOG_DIR = APP_ROOT / "logs"


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("IMU USB Tool")
        self.root.geometry("980x720")

        self.client = DeviceClient()
        self.message_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self.port_map: dict[str, PortInfo] = {}
        self.connected_port: PortInfo | None = None
        self.session_log_path: Path | None = None

        DOWNLOAD_ROOT.mkdir(parents=True, exist_ok=True)
        APP_LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._init_session_log()
        self._build_ui()
        self._refresh_ports()
        self._update_connection_ui()
        self._drain_messages()

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=12)
        top.pack(fill=tk.X)

        ttk.Label(top, text="串口").grid(row=0, column=0, sticky=tk.W, padx=(0, 8))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=42, state="readonly")
        self.port_box.grid(row=0, column=1, sticky=tk.W)
        ttk.Button(top, text="刷新串口", command=self._refresh_ports).grid(row=0, column=2, padx=8)
        self.connect_button = ttk.Button(top, text="连接", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=8)
        self.connection_var = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.connection_var).grid(row=0, column=4, sticky=tk.W, padx=(8, 0))

        action_panel = ttk.LabelFrame(self.root, text="设备操作", padding=12)
        action_panel.pack(fill=tk.X, padx=12, pady=(0, 8))

        ttk.Button(action_panel, text="查询状态", command=lambda: self._run_async(self._query_status)).pack(
            side=tk.LEFT
        )
        ttk.Button(action_panel, text="进入U盘模式", command=lambda: self._run_async(self._enter_msc)).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Button(action_panel, text="导出日志", command=lambda: self._run_async(self._export_log)).pack(
            side=tk.LEFT
        )
        ttk.Button(action_panel, text="退出U盘模式", command=lambda: self._run_async(self._enter_cdc)).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Button(action_panel, text="复位回CDC", command=lambda: self._run_async(self._reset_to_cdc)).pack(
            side=tk.LEFT, padx=24
        )

        test_panel = ttk.LabelFrame(self.root, text="SD 测试", padding=12)
        test_panel.pack(fill=tk.X, padx=12, pady=(0, 8))

        ttk.Label(test_panel, text="假数据行数").pack(side=tk.LEFT)
        self.fake_lines_var = tk.StringVar(value="128")
        ttk.Entry(test_panel, textvariable=self.fake_lines_var, width=10).pack(side=tk.LEFT, padx=(8, 12))
        self.fake_overwrite_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(test_panel, text="覆盖 IMU_TEST.CSV", variable=self.fake_overwrite_var).pack(side=tk.LEFT)
        ttk.Button(test_panel, text="假数据写 SD 测试", command=lambda: self._run_async(self._fake_sd_test)).pack(
            side=tk.LEFT, padx=12
        )

        path_panel = ttk.LabelFrame(self.root, text="导出目录", padding=12)
        path_panel.pack(fill=tk.X, padx=12, pady=(0, 8))
        self.output_dir_var = tk.StringVar(value=str(DOWNLOAD_ROOT))
        ttk.Entry(path_panel, textvariable=self.output_dir_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(path_panel, text="选择目录", command=self._choose_output_dir).pack(side=tk.LEFT, padx=8)

        self.progress_var = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.progress_var, padding=(12, 0)).pack(fill=tk.X)

        self.log_text = tk.Text(self.root, height=32)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)
        self.log_text.configure(state=tk.DISABLED)

    def _init_session_log(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_log_path = APP_LOG_DIR / f"imu_usb_tool_{stamp}.log"
        self.session_log_path.write_text("", encoding="utf-8")

    def _refresh_ports(self) -> None:
        ports = self.client.list_port_infos()
        labels = [port.label for port in ports]
        self.port_map = {port.label: port for port in ports}
        self.port_box["values"] = labels

        preferred = None
        try:
            preferred = wait_for_any_target_port(timeout=0.1)
        except Exception:
            preferred = None

        if preferred is not None:
            self.port_var.set(preferred.label)
        elif labels and self.port_var.get() not in self.port_map:
            self.port_var.set(labels[0])

        self._append_log(f"串口列表: {labels or ['<none>']}")

    def _toggle_connection(self) -> None:
        if self.client.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        selected = self.port_var.get().strip()
        if not selected:
            messagebox.showwarning("提示", "请先选择串口")
            return

        port_info = self.port_map.get(selected)
        if port_info is None:
            messagebox.showwarning("提示", "串口无效，请刷新后重试")
            return

        try:
            self.client.open(port_info.device, DEFAULT_BAUDRATE)
            self.connected_port = port_info
            self.progress_var.set(f"已连接: {port_info.device}")
            self._append_log(f"已连接 {port_info.device}")
            self._update_connection_ui()
        except Exception as exc:  # noqa: BLE001
            self.client.close()
            self.connected_port = None
            messagebox.showerror("连接失败", str(exc))
            self._append_log(f"连接失败: {exc}")
            self._update_connection_ui()

    def _disconnect(self) -> None:
        self.client.close()
        self.connected_port = None
        self.progress_var.set("已断开")
        self._append_log("串口已断开")
        self._update_connection_ui()

    def _drop_active_connection(self) -> None:
        if self.client.is_open:
            self.client.close()
        self.connected_port = None

    def _reconnect_preferred_port(
        self,
        timeout: float,
        action_text: str,
        preferred_port: PortInfo | None = None,
    ) -> PortInfo:
        port = preferred_port if preferred_port is not None else wait_for_any_target_port(timeout=timeout)
        self.client.close()
        self.client.open(port.device, DEFAULT_BAUDRATE)
        self.connected_port = port
        self.message_queue.put(("refresh", "ports"))
        self.message_queue.put(("refresh", "connect"))
        self.message_queue.put(("info", f"{action_text}，已重连 {port.device}"))
        return port

    def _update_connection_ui(self) -> None:
        if self.client.is_open and self.connected_port is not None:
            self.connect_button.configure(text="断开")
            self.connection_var.set(f"连接成功: {self.connected_port.device}")
        else:
            self.connect_button.configure(text="连接")
            self.connection_var.set("未连接")

    def _choose_output_dir(self) -> None:
        current = Path(self.output_dir_var.get()).resolve()
        chosen = filedialog.askdirectory(initialdir=str(current))
        if chosen:
            self.output_dir_var.set(chosen)

    def _selected_port_info(self) -> PortInfo | None:
        selected = self.port_var.get().strip()
        if not selected:
            return None

        port_info = self.port_map.get(selected)
        if port_info is not None:
            return port_info

        if " - " in selected:
            device = selected.split(" - ", 1)[0].strip()
            for port in self.port_map.values():
                if port.device.upper() == device.upper():
                    return port

        for port in self.port_map.values():
            if port.device.upper() == selected.upper():
                return port

        return None

    def _append_log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{stamp}] {message}\n"
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)
        if self.session_log_path is not None:
            with self.session_log_path.open("a", encoding="utf-8") as handle:
                handle.write(line)

    def _run_async(self, target) -> None:
        thread = threading.Thread(target=self._thread_wrapper, args=(target,), daemon=True)
        thread.start()

    def _thread_wrapper(self, target) -> None:
        try:
            target()
        except Exception as exc:  # noqa: BLE001
            self.message_queue.put(("error", str(exc)))

    def _ensure_connected_port(self) -> PortInfo:
        if self.connected_port is None:
            selected = self._selected_port_info()
            if selected is not None:
                return selected
            port_name = self.port_var.get().strip()
            if " - " in port_name:
                port_name = port_name.split(" - ", 1)[0].strip()
            return find_cdc_port(port_name if port_name else None)
        return self.connected_port

    @staticmethod
    def _format_protocol_response(command: str, response: str) -> str:
        if not response:
            return f"{command}: <no response>"

        if "command not found." in response:
            return f"{command}: command not found. MCU firmware is not updated to the latest command set."

        ack = ""
        status = ""
        result = ""
        others: list[str] = []

        for line in response.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("ACK "):
                ack = stripped
            elif stripped.startswith("STATUS "):
                status = stripped
            elif stripped.startswith("RESULT "):
                result = stripped
            else:
                others.append(stripped)

        parts = [item for item in [ack, status, result] if item]
        parts.extend(others)
        return " | ".join(parts) if parts else response

    def _send_shell_command(
        self,
        port: PortInfo,
        command: str,
        timeout: float,
        allow_disconnect: bool = False,
    ) -> str:
        active_client = self.client if (self.client.is_open and self.connected_port and self.connected_port.device == port.device) else None
        temp_client = DeviceClient()

        try:
            if active_client is None:
                temp_client.open(port.device, DEFAULT_BAUDRATE)
                active_client = temp_client

            response = active_client.send_command(command, timeout=timeout, allow_disconnect=allow_disconnect)
            return self._format_protocol_response(command, response)
        finally:
            temp_client.close()

    def _query_status(self) -> None:
        port = self._ensure_connected_port()
        response = self._send_shell_command(port, "status", timeout=2.0)
        self.message_queue.put(("info", f"{port.device} -> {response}"))

    def _enter_msc(self) -> None:
        port = self._ensure_connected_port()
        before = snapshot_volumes()
        response = self._send_shell_command(port, "enter_msc", timeout=2.0, allow_disconnect=True)
        self.message_queue.put(("info", f"{port.device} -> {response}"))
        if self.client.is_open and self.connected_port and self.connected_port.device == port.device:
            self._drop_active_connection()
        self.message_queue.put(("refresh", "disconnect"))

        new_port, drive = wait_for_mode_transition(
            before,
            wait=DEFAULT_WAIT_SECONDS,
            expected_drive_file=DEFAULT_LOG_FILE,
            expect_cdc=True,
        )
        if new_port is not None:
            self._reconnect_preferred_port(8.0, "已进入 CDC+MSC", preferred_port=new_port)
        if drive is not None:
            self.message_queue.put(("info", f"MSC 已挂载: {drive}"))

    def _export_log(self) -> None:
        port = self._ensure_connected_port()
        output_dir = Path(self.output_dir_var.get()).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)

        before = snapshot_volumes()
        response = self._send_shell_command(port, "enter_msc", timeout=2.0, allow_disconnect=True)
        self.message_queue.put(("info", f"{port.device} -> {response}"))
        if self.client.is_open and self.connected_port and self.connected_port.device == port.device:
            self._drop_active_connection()
        self.message_queue.put(("refresh", "disconnect"))

        new_port, drive = wait_for_mode_transition(
            before,
            wait=DEFAULT_WAIT_SECONDS,
            expected_drive_file=DEFAULT_LOG_FILE,
            expect_cdc=True,
        )
        if drive is None:
            raise FileNotFoundError("未等到 MSC 盘符或日志文件")
        source = Path(f"{drive}\\{DEFAULT_LOG_FILE}")
        if not source.exists():
            raise FileNotFoundError(f"log file not found: {source}")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        destination = output_dir / f"imu_log_{timestamp}.csv"
        shutil.copy2(source, destination)
        if new_port is not None:
            self._reconnect_preferred_port(8.0, "导出模式已就绪", preferred_port=new_port)
        self.message_queue.put(("info", f"日志已导出: {destination}"))

    def _enter_cdc(self) -> None:
        port = self._ensure_connected_port()
        response = self._send_shell_command(port, "enter_cdc", timeout=2.0, allow_disconnect=True)
        self.message_queue.put(("info", f"{port.device} -> {response}"))
        if self.client.is_open and self.connected_port and self.connected_port.device == port.device:
            self._drop_active_connection()
        self.message_queue.put(("refresh", "disconnect"))

        restored = self._reconnect_preferred_port(DEFAULT_WAIT_SECONDS, "已回到 CDC-only")
        self.message_queue.put(("info", f"CDC 已恢复: {restored.device}"))

    def _fake_sd_test(self) -> None:
        port = self._ensure_connected_port()

        try:
            lines = int(self.fake_lines_var.get().strip())
        except ValueError as exc:
            raise ValueError("假数据行数必须是整数") from exc

        if lines <= 0:
            raise ValueError("假数据行数必须大于 0")

        overwrite = 1 if self.fake_overwrite_var.get() else 0
        command = f"fake_imu_sd_test {lines} {overwrite}"
        response = self._send_shell_command(port, command, timeout=5.0)
        self.message_queue.put(("info", f"{port.device} -> {response}"))

    def _reset_to_cdc(self) -> None:
        selected = self._selected_port_info()
        port_name = selected.device if selected is not None else (self.port_var.get().strip() or None)
        self.message_queue.put(("info", "开始通过 J-Link 复位目标板"))
        jlink_reset()
        if port_name:
            wait_for_port_presence(port_name, present=True, timeout=DEFAULT_WAIT_SECONDS)
        self.message_queue.put(("refresh", "ports"))
        self.message_queue.put(("info", "CDC 已恢复"))

    def _drain_messages(self) -> None:
        try:
            while True:
                level, payload = self.message_queue.get_nowait()
                if level == "error":
                    self.progress_var.set("失败")
                    self._append_log(f"错误: {payload}")
                    messagebox.showerror("执行失败", payload)
                elif level == "info":
                    self.progress_var.set(payload)
                    self._append_log(payload)
                elif level == "refresh":
                    if payload == "ports":
                        self._refresh_ports()
                    elif payload == "disconnect":
                        self._update_connection_ui()
                    elif payload == "connect":
                        self._update_connection_ui()
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self._drain_messages)


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
