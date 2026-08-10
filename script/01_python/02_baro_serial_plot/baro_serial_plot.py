#!/usr/bin/env python3
"""RT-Thread BARO serial monitor and real-time plotter."""

from __future__ import annotations

import argparse
import csv
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TextIO


BARO_PATTERN = re.compile(
    r"BARO\[(?P<device>[^\]]+)\]:\s*"
    r"P=(?P<pressure>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)Pa\s*"
    r"T=(?P<temperature>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)deg\s*"
    r"timestamp=(?P<timestamp>\d+)ms"
)


@dataclass(frozen=True)
class BaroSample:
    device: str
    pressure_pa: float
    temperature_deg: float
    timestamp_ms: int


def parse_baro_line(line: str) -> BaroSample | None:
    """Parse one firmware log line, ignoring unrelated RT-Thread output."""
    match = BARO_PATTERN.search(line)
    if match is None:
        return None
    return BaroSample(
        device=match.group("device"),
        pressure_pa=float(match.group("pressure")),
        temperature_deg=float(match.group("temperature")),
        timestamp_ms=int(match.group("timestamp")),
    )


def run_self_test() -> int:
    cases = [
        (
            "BARO[a06_03_1]: P=101325.00Pa T=25.31deg timestamp=1234ms",
            BaroSample("a06_03_1", 101325.0, 25.31, 1234),
        ),
        (
            "[I/app] BARO[spa003]: P=9.8765e4Pa T=-2.5deg timestamp=42ms\r\n",
            BaroSample("spa003", 98765.0, -2.5, 42),
        ),
    ]
    for line, expected in cases:
        actual = parse_baro_line(line)
        if actual != expected:
            print(f"SELF-TEST FAILED: {line!r}\nexpected={expected}\nactual={actual}")
            return 1
    if parse_baro_line("baro device 'a06_03_1' not found!") is not None:
        print("SELF-TEST FAILED: unrelated log was parsed as BARO data")
        return 1
    print("BARO parser self-test passed")
    return 0


def import_gui_dependencies():
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        import serial
        from serial.tools import list_ports
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
    except ImportError as exc:
        print(
            "缺少 GUI 运行依赖。请使用项目自带的 run_baro_plot.bat，或执行：\n"
            "  py -3.12 -m pip install -r requirements.txt\n"
            f"原始错误：{exc}",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc
    return tk, ttk, filedialog, messagebox, serial, list_ports, Figure, FigureCanvasTkAgg


class BaroPlotApp:
    MAX_POINTS = 3000
    UI_INTERVAL_MS = 50

    def __init__(self, root, deps) -> None:
        (
            self.tk,
            self.ttk,
            self.filedialog,
            self.messagebox,
            self.serial_module,
            self.list_ports,
            Figure,
            FigureCanvasTkAgg,
        ) = deps
        self.root = root
        self.root.title("BARO 串口实时曲线")
        self.root.geometry("1180x760")
        self.root.minsize(900, 620)

        self.serial_port = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.csv_file: TextIO | None = None
        self.csv_writer = None
        self.csv_path: Path | None = None

        self.host_times: deque[float] = deque(maxlen=self.MAX_POINTS)
        self.pressures: deque[float] = deque(maxlen=self.MAX_POINTS)
        self.temperatures: deque[float] = deque(maxlen=self.MAX_POINTS)
        self.sample_arrivals: deque[float] = deque(maxlen=100)
        self.plot_dirty = False
        self.connection_started = 0.0

        self.port_var = self.tk.StringVar()
        self.baud_var = self.tk.StringVar(value="115200")
        self.window_var = self.tk.StringVar(value="30")
        self.status_var = self.tk.StringVar(value="未连接")
        self.device_var = self.tk.StringVar(value="--")
        self.pressure_var = self.tk.StringVar(value="-- Pa")
        self.temperature_var = self.tk.StringVar(value="-- °C")
        self.timestamp_var = self.tk.StringVar(value="-- ms")
        self.rate_var = self.tk.StringVar(value="-- Hz")
        self.csv_var = self.tk.StringVar(value="CSV：未保存")

        self._build_controls()
        self._build_plot(Figure, FigureCanvasTkAgg)
        self._build_log()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(self.UI_INTERVAL_MS, self._process_events)

    def _build_controls(self) -> None:
        frame = self.ttk.Frame(self.root, padding=(10, 8))
        frame.pack(fill="x")

        self.ttk.Label(frame, text="串口").grid(row=0, column=0, padx=(0, 5))
        self.port_combo = self.ttk.Combobox(frame, textvariable=self.port_var, width=22)
        self.port_combo.grid(row=0, column=1, padx=(0, 5))
        self.ttk.Button(frame, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 12))

        self.ttk.Label(frame, text="波特率").grid(row=0, column=3, padx=(0, 5))
        self.baud_combo = self.ttk.Combobox(
            frame,
            textvariable=self.baud_var,
            values=("9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"),
            width=10,
        )
        self.baud_combo.grid(row=0, column=4, padx=(0, 12))

        self.connect_button = self.ttk.Button(frame, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=(0, 12))

        self.ttk.Label(frame, text="显示窗口").grid(row=0, column=6, padx=(0, 5))
        self.ttk.Combobox(
            frame, textvariable=self.window_var, values=("10", "20", "30", "60", "120"), width=6, state="readonly"
        ).grid(row=0, column=7)
        self.ttk.Label(frame, text="秒").grid(row=0, column=8, padx=(3, 12))

        self.ttk.Button(frame, text="清空曲线", command=self.clear_data).grid(row=0, column=9, padx=(0, 6))
        self.csv_button = self.ttk.Button(frame, text="开始保存 CSV", command=self.toggle_csv)
        self.csv_button.grid(row=0, column=10)

        info = self.ttk.LabelFrame(self.root, text="实时状态", padding=(10, 6))
        info.pack(fill="x", padx=10, pady=(0, 6))
        entries = (
            ("连接", self.status_var),
            ("设备", self.device_var),
            ("气压", self.pressure_var),
            ("温度", self.temperature_var),
            ("传感器时间", self.timestamp_var),
            ("采样率", self.rate_var),
        )
        for index, (label, variable) in enumerate(entries):
            self.ttk.Label(info, text=f"{label}：").grid(row=0, column=index * 2, sticky="e", padx=(4, 0))
            self.ttk.Label(info, textvariable=variable, width=16).grid(row=0, column=index * 2 + 1, sticky="w")
        self.ttk.Label(info, textvariable=self.csv_var).grid(row=1, column=0, columnspan=12, sticky="w", pady=(5, 0))

    def _build_plot(self, Figure, FigureCanvasTkAgg) -> None:
        plot_frame = self.ttk.Frame(self.root)
        plot_frame.pack(fill="both", expand=True, padx=10)
        figure = Figure(figsize=(10, 5), dpi=100, constrained_layout=True)
        self.pressure_axis = figure.add_subplot(211)
        self.temperature_axis = figure.add_subplot(212, sharex=self.pressure_axis)
        (self.pressure_line,) = self.pressure_axis.plot([], [], color="#1f77b4", linewidth=1.4)
        (self.temperature_line,) = self.temperature_axis.plot([], [], color="#d62728", linewidth=1.4)
        self.pressure_axis.set_ylabel("Pressure / Pa")
        self.temperature_axis.set_ylabel("Temperature / °C")
        self.temperature_axis.set_xlabel("Host elapsed time / s")
        self.pressure_axis.grid(True, alpha=0.3)
        self.temperature_axis.grid(True, alpha=0.3)
        self.canvas = FigureCanvasTkAgg(figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_log(self) -> None:
        frame = self.ttk.LabelFrame(self.root, text="串口日志（最近 300 行）", padding=5)
        frame.pack(fill="both", padx=10, pady=(6, 10))
        self.log_text = self.tk.Text(frame, height=8, wrap="none", state="disabled", font=("Consolas", 9))
        scrollbar = self.ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def refresh_ports(self) -> None:
        ports = list(self.list_ports.comports())
        display_values = [f"{port.device} — {port.description}" for port in ports]
        self.port_combo["values"] = display_values
        selected_device = self._selected_device()
        if ports and selected_device not in {port.device for port in ports}:
            self.port_var.set(display_values[0])
        elif not ports and self.serial_port is None:
            self.port_var.set("")
            self.status_var.set("未找到串口")

    def _selected_device(self) -> str:
        return self.port_var.get().split(" — ", 1)[0].strip()

    def toggle_connection(self) -> None:
        if self.serial_port is None:
            self.connect_serial()
        else:
            self.disconnect_serial("已断开")

    def connect_serial(self) -> None:
        port = self._selected_device()
        if not port:
            self.messagebox.showwarning("未选择串口", "请先刷新并选择一个串口。")
            return
        try:
            baudrate = int(self.baud_var.get())
            if baudrate <= 0:
                raise ValueError
        except ValueError:
            self.messagebox.showerror("波特率错误", "请输入有效的正整数波特率。")
            return
        try:
            self.serial_port = self.serial_module.Serial(port, baudrate, timeout=0.2)
            self.serial_port.reset_input_buffer()
        except Exception as exc:
            self.serial_port = None
            self.status_var.set("连接失败")
            self.messagebox.showerror("串口连接失败", f"无法打开 {port}：\n\n{exc}\n\n请检查串口是否被其他程序占用。")
            return

        self.stop_event.clear()
        self.connection_started = time.monotonic()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True, name="baro-serial-reader")
        self.reader_thread.start()
        self.status_var.set(f"已连接 {port}")
        self.connect_button.configure(text="断开")
        self.port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self._append_log(f"--- 已连接 {port} @ {baudrate} ---")

    def disconnect_serial(self, reason: str = "已断开") -> None:
        self.stop_event.set()
        serial_port = self.serial_port
        self.serial_port = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        self.status_var.set(reason)
        self.connect_button.configure(text="连接")
        self.port_combo.configure(state="normal")
        self.baud_combo.configure(state="normal")
        self._append_log(f"--- {reason} ---")

    def _reader_loop(self) -> None:
        serial_port = self.serial_port
        if serial_port is None:
            return
        while not self.stop_event.is_set():
            try:
                raw = serial_port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                self.events.put(("line", line))
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.events.put(("error", str(exc)))
                break

    def _process_events(self) -> None:
        try:
            processed = 0
            while processed < 500:
                event_type, payload = self.events.get_nowait()
                processed += 1
                if event_type == "line":
                    self._handle_line(str(payload))
                elif event_type == "error":
                    self.disconnect_serial("串口异常断开")
                    self.messagebox.showerror("串口读取失败", str(payload))
                    break
            if self.plot_dirty:
                self._update_plot()
                self.plot_dirty = False
        except queue.Empty:
            pass
        finally:
            self.root.after(self.UI_INTERVAL_MS, self._process_events)

    def _handle_line(self, line: str) -> None:
        self._append_log(line)
        sample = parse_baro_line(line)
        if sample is None:
            if "not found" in line.lower() and "baro" in line.lower():
                self.status_var.set("已连接串口，未检测到 BARO")
            return

        now = time.monotonic()
        elapsed = now - self.connection_started
        self.host_times.append(elapsed)
        self.pressures.append(sample.pressure_pa)
        self.temperatures.append(sample.temperature_deg)
        self.sample_arrivals.append(now)
        self.device_var.set(sample.device)
        self.pressure_var.set(f"{sample.pressure_pa:.2f} Pa")
        self.temperature_var.set(f"{sample.temperature_deg:.2f} °C")
        self.timestamp_var.set(f"{sample.timestamp_ms} ms")
        self.status_var.set("BARO 数据正常")
        if len(self.sample_arrivals) >= 2:
            duration = self.sample_arrivals[-1] - self.sample_arrivals[0]
            if duration > 0:
                self.rate_var.set(f"{(len(self.sample_arrivals) - 1) / duration:.1f} Hz")
        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [datetime.now().isoformat(timespec="milliseconds"), sample.timestamp_ms, sample.device,
                 f"{sample.pressure_pa:.6f}", f"{sample.temperature_deg:.6f}"]
            )
            self.csv_file.flush()
        self.plot_dirty = True

    def _update_plot(self) -> None:
        if not self.host_times:
            return
        x_values = list(self.host_times)
        window_seconds = float(self.window_var.get())
        right = x_values[-1]
        left = max(0.0, right - window_seconds)
        start = 0
        while start < len(x_values) - 1 and x_values[start] < left:
            start += 1
        x_visible = x_values[start:]
        p_visible = list(self.pressures)[start:]
        t_visible = list(self.temperatures)[start:]
        self.pressure_line.set_data(x_visible, p_visible)
        self.temperature_line.set_data(x_visible, t_visible)
        self.pressure_axis.set_xlim(left, max(window_seconds, right) if right <= window_seconds else right)
        self._set_y_limits(self.pressure_axis, p_visible)
        self._set_y_limits(self.temperature_axis, t_visible)
        self.canvas.draw_idle()

    @staticmethod
    def _set_y_limits(axis, values: list[float]) -> None:
        if not values:
            return
        low, high = min(values), max(values)
        span = high - low
        padding = max(span * 0.12, abs(high) * 0.00005, 0.1)
        axis.set_ylim(low - padding, high + padding)

    def _append_log(self, line: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 300:
            self.log_text.delete("1.0", f"{line_count - 300}.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_data(self) -> None:
        self.host_times.clear()
        self.pressures.clear()
        self.temperatures.clear()
        self.sample_arrivals.clear()
        self.pressure_line.set_data([], [])
        self.temperature_line.set_data([], [])
        self.pressure_axis.set_xlim(0, float(self.window_var.get()))
        self.temperature_axis.set_xlim(0, float(self.window_var.get()))
        self.canvas.draw_idle()
        self.rate_var.set("-- Hz")

    def toggle_csv(self) -> None:
        if self.csv_file is not None:
            self._close_csv()
            return
        default_name = f"baro_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = self.filedialog.asksaveasfilename(
            title="保存 BARO 数据",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if not path:
            return
        try:
            self.csv_file = open(path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["host_time", "sensor_timestamp_ms", "device", "pressure_pa", "temperature_deg"])
            self.csv_file.flush()
            self.csv_path = Path(path)
        except OSError as exc:
            self.csv_file = None
            self.csv_writer = None
            self.messagebox.showerror("CSV 创建失败", str(exc))
            return
        self.csv_button.configure(text="停止保存 CSV")
        self.csv_var.set(f"CSV：正在保存到 {self.csv_path}")

    def _close_csv(self) -> None:
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except OSError:
                pass
        saved_path = self.csv_path
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.csv_button.configure(text="开始保存 CSV")
        self.csv_var.set(f"CSV：已保存 {saved_path}" if saved_path else "CSV：未保存")

    def on_close(self) -> None:
        self.stop_event.set()
        serial_port = self.serial_port
        self.serial_port = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        self._close_csv()
        self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot RT-Thread BARO data received from a serial port")
    parser.add_argument("--self-test", action="store_true", help="test only the firmware log parser")
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    deps = import_gui_dependencies()
    tk = deps[0]
    root = tk.Tk()
    BaroPlotApp(root, deps)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
