from __future__ import annotations

import queue
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

from imu_usb_tool import (
    DEFAULT_BAUDRATE,
    DEFAULT_WAIT_SECONDS,
    TEST_OPTIONS,
    analyze_real_imu_bin_file,
    find_cdc_port,
    parse_imu_probe_response,
    parse_noise_prepare_response,
    parse_noise_status_response,
    parse_status_response,
    snapshot_volumes,
    wait_for_any_target_port,
    wait_for_mode_transition,
)
from serial_client import DeviceClient, PortInfo


APP_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = APP_ROOT.parent
APP_LOG_DIR = APP_ROOT / "logs"
APP_DATA_DIR = PROJECT_ROOT / "data"
NOISE_DURATION_MAX_MINUTES = 120
TEST_LABELS = {key: label for key, label in TEST_OPTIONS}
ANALYSIS_OUTPUT_DIRS = {
    "bias": APP_DATA_DIR / "01_bias",
    "temp": APP_DATA_DIR / "02_temp",
    "arw": APP_DATA_DIR / "03_arw",
    "vibe": APP_DATA_DIR / "04_vibration",
    "shock": APP_DATA_DIR / "05_shock",
}
IMU_SLOT_LABELS = {
    1: "42688_A",
    2: "42688_B",
    3: "45686_A",
    4: "45686_B",
}


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("IMU USB Tool")
        self.root.geometry("1080x760")

        self.client = DeviceClient()
        self.message_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.port_map: dict[str, PortInfo] = {}
        self.connected_port: PortInfo | None = None
        self.session_log_path: Path | None = None
        self.active_test_key: str | None = None
        self.test_stop_event = threading.Event()
        self.device_mode: str = "unknown"

        APP_LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._init_session_log()
        self._build_ui()
        self._refresh_ports()
        self._auto_connect_preferred_port()
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

        ttk.Button(action_panel, text="查询状态", command=lambda: self._run_async(self._query_status)).pack(side=tk.LEFT)
        self.mode_toggle_button = ttk.Button(
            action_panel,
            text="进入U盘模式",
            command=lambda: self._run_async(self._toggle_usb_mode),
        )
        self.mode_toggle_button.pack(side=tk.LEFT, padx=8)

        probe_panel = ttk.LabelFrame(self.root, text="IMU 探测", padding=12)
        probe_panel.pack(fill=tk.X, padx=12, pady=(0, 8))
        for imu_index in range(1, 5):
            ttk.Button(
                probe_panel,
                text=f"{IMU_SLOT_LABELS[imu_index]} 探测",
                command=lambda idx=imu_index: self._run_async(lambda: self._probe_imu(idx)),
            ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(
            probe_panel,
            text="探测全部",
            command=lambda: self._run_async(self._probe_all_imus),
        ).pack(side=tk.LEFT, padx=(0, 8))

        test_panel = ttk.LabelFrame(self.root, text="测试", padding=12)
        test_panel.pack(fill=tk.X, padx=12, pady=(0, 8))
        ttk.Label(test_panel, text="测试项").pack(side=tk.LEFT)
        self.test_option_var = tk.StringVar(value=TEST_OPTIONS[0][1])
        self.test_box = ttk.Combobox(
            test_panel,
            textvariable=self.test_option_var,
            values=[label for _, label in TEST_OPTIONS],
            width=38,
            state="readonly",
        )
        self.test_box.pack(side=tk.LEFT, padx=(8, 12))
        ttk.Button(test_panel, text="开始测试", command=lambda: self._run_async(self._start_test)).pack(side=tk.LEFT)
        ttk.Button(test_panel, text="中止测试", command=lambda: self._run_async(self._stop_test)).pack(side=tk.LEFT, padx=(8, 0))

        analysis_panel = ttk.LabelFrame(self.root, text="分析", padding=12)
        analysis_panel.pack(fill=tk.X, padx=12, pady=(0, 8))
        ttk.Label(analysis_panel, text="分析项").pack(side=tk.LEFT)
        self.analysis_option_var = tk.StringVar(value=TEST_OPTIONS[0][1])
        self.analysis_box = ttk.Combobox(
            analysis_panel,
            textvariable=self.analysis_option_var,
            values=[label for _, label in TEST_OPTIONS],
            width=38,
            state="readonly",
        )
        self.analysis_box.pack(side=tk.LEFT, padx=(8, 12))
        ttk.Button(analysis_panel, text="选择 BIN 分析", command=self._choose_analysis_bin).pack(side=tk.LEFT, padx=(0, 8))

        log_panel = ttk.LabelFrame(self.root, text="日志框", padding=12)
        log_panel.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)

        self.log_text = tk.Text(log_panel, height=30)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def _init_session_log(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_log_path = APP_LOG_DIR / f"imu_usb_tool_{stamp}.log"
        self.session_log_path.write_text("", encoding="utf-8")

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

        active_port = self._find_port_by_device(self.connected_port.device) if self.connected_port is not None else None
        if active_port is not None:
            self.connected_port = active_port
            self.port_var.set(active_port.label)
        elif self.connected_port is not None and not self.client.is_open:
            self.connected_port = None

        if self.connected_port is None and preferred is not None:
            self.port_var.set(preferred.label)
        elif labels and self.port_var.get() not in self.port_map:
            self.port_var.set(labels[0])

        self._update_connection_ui()

    def _find_port_by_device(self, device: str | None) -> PortInfo | None:
        if not device:
            return None

        target = device.upper()
        for port in self.port_map.values():
            if port.device.upper() == target:
                return port
        return None

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
            self._append_log(f"已连接 {port_info.device}")
            self._run_async(lambda: self._query_status(suppress_errors=True))
        except Exception as exc:  # noqa: BLE001
            self.client.close()
            self.connected_port = None
            messagebox.showerror("连接失败", str(exc))
            self._append_log(f"连接失败: {exc}")
        finally:
            self._update_connection_ui()

    def _auto_connect_preferred_port(self) -> None:
        selected = self.port_var.get().strip()
        port_info = self._selected_port_info()

        if port_info is None:
            try:
                port_info = wait_for_any_target_port(timeout=0.3)
            except Exception:
                return

        self.port_var.set(port_info.label)

        try:
            self.client.open(port_info.device, DEFAULT_BAUDRATE)
            self.connected_port = port_info
            self._append_log(f"串口自动连接成功: {port_info.device}")
            self._run_async(lambda: self._query_status(suppress_errors=True))
        except Exception as exc:  # noqa: BLE001
            self.client.close()
            self.connected_port = None
            self._append_log(f"串口自动连接失败: {exc}")
        finally:
            self._update_connection_ui()

    def _disconnect(self) -> None:
        self.client.close()
        self.connected_port = None
        self._append_log("串口已断开")
        self._update_connection_ui()

    def _update_connection_ui(self) -> None:
        if self.client.is_open and self.connected_port is not None:
            self.connect_button.configure(text="断开")
            self.connection_var.set(f"连接成功: {self.connected_port.device}")
        else:
            self.connect_button.configure(text="连接")
            self.connection_var.set("未连接")
            self.device_mode = "unknown"
        self._update_usb_mode_button()

    def _update_usb_mode_button(self) -> None:
        if self.device_mode == "cdc+msc":
            self.mode_toggle_button.configure(text="退出U盘模式")
        else:
            self.mode_toggle_button.configure(text="进入U盘模式")

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

    def _ensure_connected_port(self) -> PortInfo:
        if self.connected_port is not None:
            return self.connected_port

        selected = self._selected_port_info()
        if selected is not None:
            return selected

        port_name = self.port_var.get().strip()
        if " - " in port_name:
            port_name = port_name.split(" - ", 1)[0].strip()
        return find_cdc_port(port_name if port_name else None)

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
        self.message_queue.put(("info", f"{action_text}，已连接 {port.device}"))
        return port

    def _run_async(self, target) -> None:
        thread = threading.Thread(target=self._thread_wrapper, args=(target,), daemon=True)
        thread.start()

    def _thread_wrapper(self, target) -> None:
        try:
            target()
        except Exception as exc:  # noqa: BLE001
            self.message_queue.put(("error", str(exc)))

    def _ask_yes_no(self, title: str, message: str) -> bool:
        result: list[bool] = []
        done = threading.Event()

        def _show() -> None:
            result.append(messagebox.askyesno(title, message))
            done.set()

        self.root.after(0, _show)
        done.wait()
        return result[0] if result else False

    def _ask_noise_duration(self, default_minutes: int = 0, default_seconds: int = 10) -> tuple[int, int] | None:
        result: list[tuple[int, int] | None] = []
        done = threading.Event()

        def _show() -> None:
            class NoiseDurationDialog(simpledialog.Dialog):
                def __init__(self, parent: tk.Misc, minutes: int, seconds: int) -> None:
                    self.minutes_var = tk.IntVar(value=minutes)
                    self.seconds_var = tk.IntVar(value=seconds)
                    self.duration_value: tuple[int, int] | None = None
                    super().__init__(parent, "噪声测试时长")

                def body(self, master: tk.Misc) -> tk.Widget:
                    ttk.Label(master, text="请一次性设置噪声测试时长").grid(
                        row=0, column=0, columnspan=2, sticky="w", padx=6, pady=(6, 10)
                    )
                    ttk.Label(master, text="分钟").grid(row=1, column=0, sticky="w", padx=6, pady=4)
                    minute_box = tk.Spinbox(
                        master,
                        from_=0,
                        to=NOISE_DURATION_MAX_MINUTES,
                        textvariable=self.minutes_var,
                        width=8,
                    )
                    minute_box.grid(row=1, column=1, sticky="ew", padx=6, pady=4)
                    ttk.Label(master, text="秒钟").grid(row=2, column=0, sticky="w", padx=6, pady=4)
                    second_box = tk.Spinbox(master, from_=0, to=59, textvariable=self.seconds_var, width=8)
                    second_box.grid(row=2, column=1, sticky="ew", padx=6, pady=4)
                    master.grid_columnconfigure(1, weight=1)
                    return minute_box

                def validate(self) -> bool:
                    try:
                        minutes = int(self.minutes_var.get())
                        seconds = int(self.seconds_var.get())
                    except (TypeError, ValueError):
                        messagebox.showwarning("提示", "请输入有效的分钟和秒钟")
                        return False

                    if minutes < 0 or minutes > NOISE_DURATION_MAX_MINUTES:
                        messagebox.showwarning("提示", f"分钟范围应为 0 到 {NOISE_DURATION_MAX_MINUTES}")
                        return False
                    if seconds < 0 or seconds > 59:
                        messagebox.showwarning("提示", "秒钟范围应为 0 到 59")
                        return False
                    if minutes == NOISE_DURATION_MAX_MINUTES and seconds > 0:
                        messagebox.showwarning(
                            "提示",
                            f"当前版本单次最长支持 {NOISE_DURATION_MAX_MINUTES} 分钟，请将秒钟设为 0",
                        )
                        return False
                    if (minutes == 0) and (seconds == 0):
                        messagebox.showwarning("提示", "测试时长不能为 0 秒")
                        return False

                    self.duration_value = (minutes, seconds)
                    return True

                def apply(self) -> None:
                    self.result = self.duration_value

            dialog = NoiseDurationDialog(self.root, default_minutes, default_seconds)
            result.append(dialog.result if isinstance(dialog.result, tuple) else None)
            done.set()

        self.root.after(0, _show)
        done.wait()
        return result[0] if result else None

    @staticmethod
    def _format_simple_response(command: str, response: str) -> str:
        if not response:
            if command in {"enter_msc", "enter_cdc"}:
                return f"{command}: 命令已发送，设备正在切换模式"
            return f"{command}: 未收到响应"
        if "command not found." in response:
            return f"{command}: 固件未实现该命令"
        return response.strip().replace("0:/", "")

    @staticmethod
    def _format_device_path(path_text: str) -> str:
        if not path_text:
            return "03_arw/000.BIN"
        if path_text.startswith("0:/"):
            return path_text[3:]
        return path_text

    def _send_shell_command(
        self,
        port: PortInfo,
        command: str,
        timeout: float,
        allow_disconnect: bool = False,
        force_reopen: bool = False,
    ) -> str:
        active_client = None
        temp_client = DeviceClient()
        last_open_error: Exception | None = None

        try:
            if (
                force_reopen
                and self.client.is_open
                and self.connected_port is not None
                and self.connected_port.device == port.device
            ):
                self.client.close()
                self.connected_port = None
                self.message_queue.put(("refresh", "disconnect"))

            if (
                not force_reopen
                and self.client.is_open
                and self.connected_port
                and self.connected_port.device == port.device
            ):
                active_client = self.client

            if active_client is None:
                for _ in range(3):
                    try:
                        temp_client.open(port.device, DEFAULT_BAUDRATE)
                        last_open_error = None
                        break
                    except Exception as exc:  # noqa: BLE001
                        last_open_error = exc
                        threading.Event().wait(0.35)
                if last_open_error is not None:
                    raise last_open_error
                active_client = temp_client

            return active_client.send_command(command, timeout=timeout, allow_disconnect=allow_disconnect)
        finally:
            temp_client.close()

    @staticmethod
    def _status_text_from_response(response: str) -> str:
        try:
            status = parse_status_response(response)
        except Exception:
            compact = response.strip().replace("0:/", "")
            return compact if compact else "状态查询失败：未收到可解析响应"
        mode_map = {
            "cdc": "仅 CDC",
            "cdc+msc": "CDC + MSC",
            "switching": "切换中",
        }
        ready_map = {
            "ready": "正常",
            "not_ready": "未就绪",
            "unknown": "查询不到",
        }
        return (
            f"当前模式: {mode_map.get(status.mode, '查询不到')} | "
            f"SD卡: {ready_map.get(status.sd, '查询不到')} | "
            f"测试: {ready_map.get(status.test, '查询不到')}"
        )

    def _sync_mode_from_response(self, response: str) -> None:
        try:
            status = parse_status_response(response)
        except Exception:
            return

        self.device_mode = status.mode
        self.message_queue.put(("refresh", "usb_mode"))

    def _query_status(self, suppress_errors: bool = False) -> None:
        last_error: Exception | None = None

        for _ in range(3):
            try:
                port = self._ensure_connected_port()
                response = self._send_shell_command(port, "status", timeout=2.0, force_reopen=True)
                self._sync_mode_from_response(response)
                message = self._status_text_from_response(response)
                self.message_queue.put(("info", message))
                return
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                threading.Event().wait(0.4)

        if suppress_errors:
            if last_error is not None:
                self.message_queue.put(("info", f"状态查询暂不可用: {last_error}"))
            return

        if last_error is not None:
            raise last_error

    def _toggle_usb_mode(self) -> None:
        if self.device_mode == "cdc+msc":
            self._enter_cdc()
        else:
            self._enter_msc()

    def _enter_msc(self) -> None:
        port = self._ensure_connected_port()
        before = snapshot_volumes()
        response = self._send_shell_command(port, "enter_msc", timeout=2.0, allow_disconnect=True)
        self.message_queue.put(("info", self._format_simple_response("enter_msc", response)))

        if self.client.is_open and self.connected_port and self.connected_port.device == port.device:
            self._drop_active_connection()
        self.message_queue.put(("refresh", "disconnect"))

        new_port, drive = wait_for_mode_transition(
            before,
            wait=DEFAULT_WAIT_SECONDS,
            expected_drive_file=None,
            expect_cdc=True,
        )
        if new_port is not None:
            self._reconnect_preferred_port(8.0, "已进入 CDC + MSC", preferred_port=new_port)
        self.device_mode = "cdc+msc"
        self.message_queue.put(("refresh", "usb_mode"))
        if drive is not None:
            self.message_queue.put(("info", f"U盘已就绪: {drive}"))

    def _enter_cdc(self) -> None:
        port = self._ensure_connected_port()
        response = self._send_shell_command(port, "enter_cdc", timeout=2.0, allow_disconnect=True)
        self.message_queue.put(("info", self._format_simple_response("enter_cdc", response)))

        if self.client.is_open and self.connected_port and self.connected_port.device == port.device:
            self._drop_active_connection()
        self.message_queue.put(("refresh", "disconnect"))

        restored = self._reconnect_preferred_port(DEFAULT_WAIT_SECONDS, "已回到仅 CDC")
        self.device_mode = "cdc"
        self.message_queue.put(("refresh", "usb_mode"))
        self.message_queue.put(("info", f"CDC 已恢复: {restored.device}"))

    def _probe_imu(self, imu_index: int) -> None:
        port = self._ensure_connected_port()
        response = self._send_shell_command(port, f"imu_probe{imu_index}", timeout=2.0)
        probe_results = parse_imu_probe_response(response)
        result = next((item for item in probe_results if item.imu == imu_index), None)
        if result is None:
            raise ValueError(f"IMU{imu_index} probe result not found")

        status_map = {
            "ready": "探测正常",
            "not_found": "未探测到",
            "unknown": "查询不到",
        }
        slot_label = IMU_SLOT_LABELS.get(imu_index, f"IMU{imu_index}")
        self.message_queue.put(("info", f"{slot_label}: {status_map.get(result.status, result.status)} ({result.name})"))

    def _probe_all_imus(self) -> None:
        port = self._ensure_connected_port()
        response = self._send_shell_command(port, "imu_probe_all", timeout=2.0)
        probe_results = parse_imu_probe_response(response)
        status_map = {
            "ready": "探测正常",
            "not_found": "未探测到",
            "unknown": "查询不到",
        }
        summary = " | ".join(
            f"{IMU_SLOT_LABELS.get(item.imu, f'IMU{item.imu}')}: {status_map.get(item.status, item.status)} ({item.name})"
            for item in probe_results
        )
        self.message_queue.put(("info", summary))

    def _selected_test_key(self, label: str) -> str:
        for key, item_label in TEST_OPTIONS:
            if item_label == label:
                return key
        raise ValueError(f"unknown test option: {label}")

    def _start_test(self) -> None:
        test_key = self._selected_test_key(self.test_option_var.get())
        if test_key != "arw":
            self.message_queue.put(("info", f"{TEST_LABELS[test_key]}：测试命令入口已预留，等待固件命令补齐"))
            return

        port = self._ensure_connected_port()
        prepare_response = self._send_shell_command(port, "noise_test_prepare", timeout=2.0)
        prepare = parse_noise_prepare_response(prepare_response)

        if prepare.detected <= 0 or prepare.status != "ok":
            self.message_queue.put(("info", "噪声测试准备失败：未探测到可用 IMU"))
            return

        confirm = self._ask_yes_no(
            "噪声测试确认",
            f"已探测到 {prepare.detected} 个 IMU。\n将开始 {prepare.duration_s} 秒噪声采集并写入 BIN。\n是否继续？",
        )
        if not confirm:
            self.message_queue.put(("info", "已取消噪声测试"))
            return

        duration = self._ask_noise_duration(default_minutes=0, default_seconds=prepare.duration_s)
        if duration is None:
            self.message_queue.put(("info", "已取消噪声测试时长设置"))
            return

        minutes, seconds = duration
        duration_s = (minutes * 60) + seconds

        start_response = self._send_shell_command(
            port,
            f"noise_test_start {minutes} {seconds}",
            timeout=2.0,
        )
        self.message_queue.put(("info", self._format_simple_response("noise_test_start", start_response)))
        if "status=accepted" not in start_response:
            self.active_test_key = None
            self.test_stop_event.set()
            self.message_queue.put(("info", "噪声测试启动失败"))
            return
        self.active_test_key = "arw"
        self.test_stop_event.clear()
        self.message_queue.put(("info", f"噪声测试已启动，固件将记录 {duration_s} 秒 IMU 数据到测试目录下的新 BIN 文件"))
        self._monitor_noise_test(port, duration_s)

    def _monitor_noise_test(self, port: PortInfo, duration_s: int) -> None:
        if duration_s >= 1800:
            update_interval = 60.0
        elif duration_s >= 300:
            update_interval = 15.0
        else:
            update_interval = 3.0

        elapsed = 0
        while elapsed < duration_s:
            wait_seconds = min(update_interval, float(duration_s - elapsed))
            if self.test_stop_event.wait(timeout=wait_seconds):
                return
            elapsed = min(elapsed + int(wait_seconds), duration_s)
            self.message_queue.put(("info", f"噪声测试进度: {elapsed}/{duration_s}s"))

        self._finalize_noise_test(port)

    def _finalize_noise_test(self, port: PortInfo) -> None:
        last_error: Exception | None = None
        threading.Event().wait(0.6)
        for _ in range(3):
            try:
                status_response = self._send_shell_command(
                    port,
                    "noise_test_status",
                    timeout=5.0,
                    force_reopen=True,
                )
                status = parse_noise_status_response(status_response)
                self.active_test_key = None
                self.message_queue.put((
                    "info",
                    f"噪声测试完成: frames={status.frames} duration={status.duration_s}s last_error={status.last_error}",
                ))
                self.message_queue.put(("info", f"bin 文件路径: {self._format_device_path(status.file)}"))
                self._reconnect_preferred_port(3.0, "测试结束后已恢复串口连接", preferred_port=port)
                return
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                threading.Event().wait(0.5)

        self.active_test_key = None
        self.message_queue.put(("info", "噪声测试已结束，但状态查询失败"))
        self.message_queue.put(("info", "bin 文件路径: 03_arw/000.BIN"))
        if last_error is not None:
            self.message_queue.put(("info", f"状态查询异常: {last_error}"))

    def _stop_test(self) -> None:
        if self.active_test_key != "arw":
            self.message_queue.put(("info", "当前没有可中止的实际测试"))
            return

        port = self._ensure_connected_port()
        response = self._send_shell_command(port, "noise_test_stop", timeout=2.0)
        self.test_stop_event.set()
        self.active_test_key = None
        self.message_queue.put(("info", self._format_simple_response("noise_test_stop", response)))
        self.message_queue.put(("info", "测试已中止，bin 文件路径请以状态查询返回为准"))

    def _choose_analysis_bin(self) -> None:
        chosen = filedialog.askopenfilename(
            title="选择 BIN 文件",
            filetypes=[("BIN files", "*.bin"), ("All files", "*.*")],
        )
        if chosen:
            self._run_async(lambda: self._start_analysis(Path(chosen)))

    def _start_analysis(self, source: Path) -> None:
        test_key = self._selected_test_key(self.analysis_option_var.get())
        output_dir = ANALYSIS_OUTPUT_DIRS[test_key] / source.stem
        output_dir.mkdir(parents=True, exist_ok=True)

        source = source.resolve()
        csv_path, packet_csv_path, md_path, summary = analyze_real_imu_bin_file(test_key, source, output_dir=output_dir)
        self.message_queue.put(("info", f"{TEST_LABELS[test_key]} 分析完成"))
        self.message_queue.put(("info", f"源文件: {source}"))
        self.message_queue.put(("info", f"输出目录: {output_dir}"))
        self.message_queue.put(("info", f"CSV: {csv_path}"))
        if packet_csv_path != csv_path:
            self.message_queue.put(("info", f"逐包 CSV: {packet_csv_path}"))
        self.message_queue.put(("info", f"报告: {md_path}"))

    def _drain_messages(self) -> None:
        try:
            while True:
                level, payload = self.message_queue.get_nowait()
                if level == "error":
                    self._append_log(f"错误: {payload}")
                    messagebox.showerror("执行失败", str(payload))
                elif level == "info":
                    self._append_log(str(payload))
                elif level == "refresh":
                    if payload == "ports":
                        self._refresh_ports()
                    elif payload in ("disconnect", "connect"):
                        self._update_connection_ui()
                    elif payload == "usb_mode":
                        self._update_usb_mode_button()
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
