"""Tkinter hardware-debug UI for the standalone Muweidu arm SDK."""

from __future__ import annotations

from datetime import datetime
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import shutil
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import TYPE_CHECKING, Any, Callable

from .arm_sdk import ArmSdk, MoveResult, PositionSnapshot
from .can_setup import enable_can, validate_interface
from .pi_panel import PiPanel
from .demo_cycle import DemoCycle

if TYPE_CHECKING:
    from .cartesian_motion import CartesianMotionResult, CartesianSample
    from .kinematics import CartesianPose


APP_CONFIG_DIR = Path.home() / ".config" / "muweidu_arm_sdk_ui"
HARDWARE_CONFIG_PATH = APP_CONFIG_DIR / "hardware.json"
DEMO_CONFIG_PATH = APP_CONFIG_DIR / "demo_positions.json"
DEFAULT_HARDWARE_CONFIG = Path(__file__).with_name("default_hardware.json")
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
LIVE_READ_INTERVAL_MS = 250


def _read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    temporary.replace(path)


def _load_hardware_config() -> dict[str, Any]:
    APP_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    if not HARDWARE_CONFIG_PATH.exists():
        shutil.copyfile(DEFAULT_HARDWARE_CONFIG, HARDWARE_CONFIG_PATH)
    return _read_json(HARDWARE_CONFIG_PATH)


def _default_demos() -> dict[str, list[float]]:
    return {f"demo{index}": [0.0] * 6 for index in range(6)}


def _load_demos() -> dict[str, list[float]]:
    demos = _default_demos()
    if DEMO_CONFIG_PATH.exists():
        loaded = _read_json(DEMO_CONFIG_PATH)
        for index in range(6):
            key = f"demo{index}"
            if key in loaded and len(loaded[key]) == 6:
                demos[key] = [float(value) for value in loaded[key]]
    return demos


class ArmSdkUi:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Muweidu 六轴硬件 SDK 调试工具")
        self.root.geometry("1280x920")
        self.root.minsize(1100, 780)
        self.hardware_config = _load_hardware_config()
        self.demos = _load_demos()
        self.sdk: ArmSdk | None = None
        self.pi_panel: PiPanel | None = None
        self.busy = False
        self.live_read_enabled = False
        self._live_read_inflight = False
        self._live_generation = 0
        self._live_after_id: str | None = None
        self._pending_operation: tuple[str, Callable[[], Any], Callable[[Any], None] | None, bool] | None = None
        self._demo_cycle: DemoCycle | None = None
        self._cycle_key: str | None = None
        self._cycle_generation = 0
        self._cycle_needs_leg = False
        self._cycle_started_live_read = False
        self._close_after_cycle_stop = False
        self._cartesian_cancel: threading.Event | None = None
        self._close_after_cartesian_stop = False
        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _build_widgets(self) -> None:
        viewport = ttk.Frame(self.root)
        viewport.pack(fill="both", expand=True)
        canvas = tk.Canvas(viewport, highlightthickness=0)
        vertical = ttk.Scrollbar(viewport, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vertical.set)
        vertical.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        root_frame = ttk.Frame(canvas, padding=10)
        content = canvas.create_window((0, 0), window=root_frame, anchor="nw")
        root_frame.bind("<Configure>", lambda _: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(content, width=event.width))
        self._content_canvas = canvas

        def scroll(event: Any) -> None:
            if isinstance(event.widget, tk.Text):
                return
            steps = -int(event.delta / 120) if event.delta else 0
            if steps:
                canvas.yview_scroll(steps, "units")

        self.root.bind("<MouseWheel>", scroll)
        self.root.bind("<Button-4>", lambda _: canvas.yview_scroll(-1, "units"))
        self.root.bind("<Button-5>", lambda _: canvas.yview_scroll(1, "units"))

        connection = ttk.LabelFrame(root_frame, text="真实硬件连接", padding=8)
        connection.pack(fill="x")
        ttk.Label(connection, text="SocketCAN:").grid(row=0, column=0, sticky="w")
        self.can_var = tk.StringVar(value=self.hardware_config["can_interface"])
        ttk.Entry(connection, textvariable=self.can_var, width=12).grid(
            row=0, column=1, padx=4, sticky="w")
        ttk.Label(connection, text="Motor IDs:").grid(row=0, column=2, padx=(12, 2), sticky="w")
        self.motor_id_vars: list[tk.StringVar] = []
        for index, motor_id in enumerate(self.hardware_config["motor_ids"]):
            variable = tk.StringVar(value=str(motor_id))
            self.motor_id_vars.append(variable)
            ttk.Label(connection, text=f"M{index + 1}").grid(row=0, column=3 + index * 2)
            ttk.Entry(connection, textvariable=variable, width=5).grid(
                row=0, column=4 + index * 2, padx=(2, 6))
        ttk.Button(connection, text="连接 / 应用", command=self._connect).grid(
            row=0, column=15, padx=(8, 2))
        self.connection_label = ttk.Label(connection, text="未连接")
        self.connection_label.grid(row=0, column=16, padx=8, sticky="w")
        ttk.Button(connection, text="启用 CAN（1 Mbps）", command=self._enable_can).grid(
            row=1, column=0, columnspan=4, pady=(8, 0), sticky="w")
        ttk.Label(connection, text="操作会重启所选 CAN 接口；不等同于电机上电。").grid(
            row=1, column=4, columnspan=11, pady=(8, 0), sticky="w")
        self.tendon_enabled_var = tk.BooleanVar(
            value=self.hardware_config.get("static_tendon_compensation", False))
        self.tendon_profile_var = tk.StringVar(value=self.hardware_config.get("tendon_profile", ""))
        ttk.Checkbutton(connection, text="静态绳驱补偿（应用后生效）", variable=self.tendon_enabled_var).grid(
            row=2, column=0, columnspan=4, pady=(8, 0), sticky="w")
        ttk.Label(connection, text="参数文件（留空用默认）").grid(
            row=2, column=4, columnspan=5, pady=(8, 0), sticky="w")
        ttk.Entry(connection, textvariable=self.tendon_profile_var, width=38).grid(
            row=2, column=9, columnspan=6, pady=(8, 0), sticky="ew")
        ttk.Button(connection, text="浏览…", command=self._select_tendon_profile).grid(
            row=2, column=15, padx=(8, 2), pady=(8, 0), sticky="w")
        self.j2_reversed_var = tk.BooleanVar(
            value=self.hardware_config.get("j2_urdf_direction", 1) == -1)
        ttk.Checkbutton(connection, text="J2 方向与 URDF 相反（应用后生效）",
                        variable=self.j2_reversed_var).grid(
            row=3, column=0, columnspan=12, pady=(8, 0), sticky="w")

        control = ttk.LabelFrame(root_frame, text="运行与位置命令（关节角单位：度）", padding=8)
        control.pack(fill="x", pady=(8, 0))
        ttk.Label(control, text="统一关节速度上限 (°/s):").grid(row=0, column=0, sticky="w")
        self.speed_var = tk.StringVar(value="10")
        ttk.Entry(control, textvariable=self.speed_var, width=10).grid(
            row=0, column=1, padx=4, sticky="w")
        ttk.Button(control, text="运行", command=lambda: self._run_simple("发送电机运行命令", lambda sdk: sdk.motor_on())).grid(
            row=0, column=2, padx=(12, 4))
        ttk.Button(control, text="关断", command=lambda: self._run_simple("发送电机关断命令", lambda sdk: sdk.motor_off(), priority=True)).grid(
            row=0, column=3, padx=4)
        ttk.Button(control, text="急停", command=lambda: self._run_simple("发送急停命令", lambda sdk: sdk.emergency_stop(), priority=True)).grid(
            row=0, column=4, padx=4)
        ttk.Button(control, text="读取状态", command=self._read_status).grid(
            row=0, column=5, padx=(12, 4))
        ttk.Button(control, text="读取绝对位置并保存 Demo0", command=self._read_positions).grid(
            row=0, column=6, padx=4)
        ttk.Button(control, text="移动到逻辑零位", command=self._move_zero).grid(
            row=0, column=7, padx=(12, 4))
        ttk.Button(control, text="电机编码器设零 (0x19)", command=self._set_motor_zero).grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Button(control, text="六电机三环 PI 参数", command=self._open_pi_panel).grid(
            row=1, column=3, columnspan=3, sticky="w", pady=(8, 0))

        joint_control = ttk.LabelFrame(root_frame, text="按关节运行 / 关断", padding=8)
        joint_control.pack(fill="x", pady=(8, 0))
        for index, name in enumerate(JOINT_NAMES):
            group = ttk.Frame(joint_control)
            group.grid(row=0, column=index, padx=(0, 12), sticky="w")
            ttk.Label(group, text=name).pack(side="left", padx=(0, 3))
            ttk.Button(group, text="运行", width=5, command=lambda joint=index: self._run_joint(joint, True)).pack(
                side="left", padx=2)
            ttk.Button(group, text="关断", width=5, command=lambda joint=index: self._run_joint(joint, False)).pack(
                side="left", padx=2)
        ttk.Label(
            joint_control,
            text="按映射矩阵控制该关节涉及的所有电机；耦合关节可能共享电机，运行或关断会同时影响其他关节。",
        ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(6, 0))

        self.current_labels: list[ttk.Label] = []
        current = ttk.LabelFrame(root_frame, text="当前关节位置（°）", padding=8)
        current.pack(fill="x", pady=(8, 0))
        for index, name in enumerate(JOINT_NAMES):
            ttk.Label(current, text=f"{name}:").grid(row=0, column=index * 2, padx=(4, 1))
            label = ttk.Label(current, text="—", width=12, relief="sunken", anchor="e")
            label.grid(row=0, column=index * 2 + 1, padx=(1, 8))
            self.current_labels.append(label)
        self.motor_position_label = ttk.Label(current, text="电机角度：尚未读取")
        self.motor_position_label.grid(row=1, column=0, columnspan=12, sticky="w", pady=(8, 0))
        self.encoder_position_label = ttk.Label(current, text="")
        self.encoder_position_label.grid(row=2, column=0, columnspan=12, sticky="w")
        self.live_read_button = ttk.Button(current, text="开始持续读取", command=self._toggle_live_read)
        self.live_read_button.grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self.live_read_status = tk.StringVar(value="持续读取：已停止")
        ttk.Label(current, textvariable=self.live_read_status).grid(
            row=3, column=3, columnspan=9, sticky="w", pady=(8, 0))

        commands = ttk.LabelFrame(root_frame, text="手动设置", padding=8)
        commands.pack(fill="x", pady=(8, 0))
        self.absolute_vars = self._joint_entry_row(
            commands, row=0, label="绝对目标 q (°):", values=[0.0] * 6)
        ttk.Button(commands, text="发送绝对位置", command=self._send_absolute).grid(
            row=0, column=13, padx=8)
        self.increment_vars = self._joint_entry_row(
            commands, row=1, label="增量 Δq (°):", values=[0.0] * 6)
        ttk.Button(commands, text="发送增量位置", command=self._send_increment).grid(
            row=1, column=13, padx=8)

        self._build_cartesian_widgets(root_frame)

        demo_frame = ttk.LabelFrame(root_frame, text="Demo0…Demo5 绝对位置", padding=8)
        demo_frame.pack(fill="x", pady=(8, 0))
        ttk.Label(demo_frame, text="名称").grid(row=0, column=0, padx=4)
        for index, name in enumerate(JOINT_NAMES):
            ttk.Label(demo_frame, text=f"{name} (°)").grid(row=0, column=index + 1, padx=4)
        ttk.Label(demo_frame, text="动作").grid(row=0, column=7, padx=8)
        self.demo_vars: dict[str, list[tk.StringVar]] = {}
        self.cycle_buttons: dict[str, ttk.Button] = {}
        for demo_index in range(6):
            key = f"demo{demo_index}"
            row = demo_index + 1
            ttk.Label(demo_frame, text=key.upper()).grid(row=row, column=0, padx=4, sticky="w")
            variables: list[tk.StringVar] = []
            for joint_index, value in enumerate(self.demos[key]):
                variable = tk.StringVar(value=f"{value:.4f}")
                variables.append(variable)
                entry = ttk.Entry(demo_frame, textvariable=variable, width=11)
                entry.grid(row=row, column=joint_index + 1, padx=3, pady=2)
                if demo_index == 0:
                    entry.state(["readonly"])
            self.demo_vars[key] = variables
            button_frame = ttk.Frame(demo_frame)
            button_frame.grid(row=row, column=7, padx=4)
            if demo_index > 0:
                ttk.Button(
                    button_frame, text="保存设定",
                    command=lambda demo_key=key: self._save_demo(demo_key),
                ).pack(side="left", padx=2)
                ttk.Button(
                    button_frame, text="读取并写入当前位置",
                    command=lambda demo_key=key: self._capture_demo(demo_key),
                ).pack(side="left", padx=2)
            ttk.Button(
                button_frame, text="移动到此位置",
                command=lambda demo_key=key: self._move_demo(demo_key),
            ).pack(side="left", padx=2)
            cycle_button = ttk.Button(
                button_frame, text="零点往返循环",
                command=lambda demo_key=key: self._toggle_demo_cycle(demo_key),
            )
            cycle_button.pack(side="left", padx=2)
            self.cycle_buttons[key] = cycle_button

        self.cycle_status = tk.StringVar(value="循环运动：已停止")
        ttk.Label(demo_frame, textvariable=self.cycle_status).grid(
            row=7, column=0, columnspan=8, sticky="w", pady=(6, 0))
        options = ttk.Frame(demo_frame)
        options.grid(row=8, column=0, columnspan=8, sticky="w", pady=(4, 0))
        ttk.Label(options, text="循环到位误差 (°):").pack(side="left")
        self.cycle_tolerance_var = tk.StringVar(value="0.1")
        ttk.Entry(options, textvariable=self.cycle_tolerance_var, width=8).pack(side="left", padx=4)
        ttk.Label(options, text="连续两次读取到位后换向；循环中自动读取位置，再次点击该行停止。").pack(side="left")

        status_frame = ttk.LabelFrame(root_frame, text="反馈与执行信息", padding=8)
        status_frame.pack(fill="both", expand=True, pady=(8, 0))
        self.message_var = tk.StringVar(value="请确认 CAN、六个 ID、方向和零偏后连接真实硬件。")
        ttk.Label(status_frame, textvariable=self.message_var, anchor="w").pack(fill="x")
        self.status_text = tk.Text(status_frame, height=9, wrap="none", state="disabled")
        self.status_text.pack(fill="both", expand=True, pady=(6, 0))

        ttk.Label(
            root_frame,
            text="提示：此界面直接操作实体电机，不做碰撞检测。先用低速、短位移验证方向和标定。",
            foreground="#9a3e00",
        ).pack(anchor="w", pady=(6, 0))

    def _joint_entry_row(
        self, parent: ttk.LabelFrame, row: int, label: str, values: list[float]
    ) -> list[tk.StringVar]:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4)
        variables: list[tk.StringVar] = []
        for index, (name, value) in enumerate(zip(JOINT_NAMES, values)):
            variable = tk.StringVar(value=f"{value:.4f}")
            variables.append(variable)
            ttk.Label(parent, text=name).grid(row=row, column=1 + index * 2, padx=(2, 0))
            ttk.Entry(parent, textvariable=variable, width=10).grid(
                row=row, column=2 + index * 2, padx=(2, 5))
        return variables

    def _build_cartesian_widgets(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="笛卡尔直线运动 · base_link", padding=8)
        frame.pack(fill="x", pady=(8, 0))
        target = ttk.Frame(frame)
        target.pack(fill="x")
        ttk.Label(target, text="Arm6 目标位置 (m)").pack(side="left", padx=(0, 8))
        self.cartesian_xyz_vars = [tk.StringVar() for _ in range(3)]
        for axis, variable in zip("XYZ", self.cartesian_xyz_vars):
            ttk.Label(target, text=axis).pack(side="left", padx=(2, 3))
            ttk.Entry(target, textvariable=variable, width=12).pack(side="left", padx=(0, 6))
        ttk.Label(target, text="直线规划速度 (mm/s)").pack(side="left", padx=(10, 4))
        self.cartesian_speed_var = tk.StringVar(value="10")
        ttk.Entry(target, textvariable=self.cartesian_speed_var, width=7).pack(side="left")
        ttk.Button(target, text="读取 FK 并填入 XYZ", command=self._read_cartesian_pose).pack(
            side="left", padx=(12, 4))
        self.cartesian_start_button = ttk.Button(
            target, text="执行直线", command=self._start_cartesian)
        self.cartesian_start_button.pack(side="left", padx=4)
        self.cartesian_stop_button = ttk.Button(
            target, text="停止直线", command=self._cancel_cartesian, state="disabled")
        self.cartesian_stop_button.pack(side="left", padx=4)

        storage = ttk.Frame(frame)
        storage.pack(fill="x", pady=(6, 0))
        ttk.Label(storage, text="CSV 保存目录").pack(side="left", padx=(0, 8))
        self.cartesian_directory_var = tk.StringVar(value=str(APP_CONFIG_DIR / "cartesian_traces"))
        ttk.Entry(storage, textvariable=self.cartesian_directory_var).pack(
            side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Button(storage, text="浏览…", command=self._select_cartesian_directory).pack(side="left")
        ttk.Label(frame, text="直线运动保持起始姿态，沿用上方的关节速度上限。").pack(
            anchor="w", pady=(6, 0))
        self.cartesian_pose_var = tk.StringVar(value="末端位姿：尚未读取")
        ttk.Label(frame, textvariable=self.cartesian_pose_var).pack(anchor="w", pady=(4, 0))
        self.cartesian_status_var = tk.StringVar(value="直线运动：待命")
        ttk.Label(frame, textvariable=self.cartesian_status_var, wraplength=1030).pack(
            anchor="w", pady=(4, 0))

    def _select_cartesian_directory(self) -> None:
        path = filedialog.askdirectory(parent=self.root, title="选择轨迹 CSV 保存目录")
        if path:
            self.cartesian_directory_var.set(path)

    def _read_cartesian_pose(self) -> None:
        def read_pose() -> CartesianPose:
            from .kinematics import ArmKinematics

            sdk = self._ensure_sdk()
            model = ArmKinematics(j2_urdf_direction=sdk.config["j2_urdf_direction"])
            return model.forward(sdk.read_positions().joint_deg)

        def show_pose(pose: CartesianPose) -> None:
            for variable, value in zip(self.cartesian_xyz_vars, pose.position_m):
                variable.set(f"{value:.7f}")
            self._show_cartesian_pose(pose.position_m, pose.rpy_deg)

        self._start_worker("读取末端 FK", read_pose, show_pose)

    def _start_cartesian(self) -> None:
        if self._cycle_blocks_action():
            return
        if self.busy or self._pending_operation is not None:
            messagebox.showinfo("操作进行中", "请等当前硬件请求完成后再执行直线。", parent=self.root)
            return
        try:
            sdk = self._ensure_sdk()
            target = tuple(float(variable.get()) for variable in self.cartesian_xyz_vars)
            speed_m_s = float(self.cartesian_speed_var.get()) / 1000.0
            joint_speed = self._current_speed()
            directory_text = self.cartesian_directory_var.get().strip()
            if not all(math.isfinite(value) for value in target):
                raise ValueError("XYZ 必须是有限数值，单位 m。")
            if not math.isfinite(speed_m_s) or speed_m_s <= 0.0:
                raise ValueError("直线规划速度必须大于零，单位 mm/s。")
            if not directory_text:
                raise ValueError("请选择 CSV 保存目录。")
            directory = Path(directory_text).expanduser().resolve()
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            trace = directory / f"cartesian_line_{stamp}.csv"
        except (ValueError, RuntimeError, OSError) as error:
            messagebox.showerror("无法开始直线运动", str(error), parent=self.root)
            return

        cancel = threading.Event()
        samples: SimpleQueue[CartesianSample] = SimpleQueue()
        self._cartesian_cancel = cancel
        self.cartesian_start_button.configure(state="disabled")
        self.cartesian_stop_button.configure(state="normal")
        self.cartesian_status_var.set("正在读取起点并校验整条直线…")

        def run() -> CartesianMotionResult | Exception:
            try:
                from .cartesian_motion import execute_cartesian_line
                from .kinematics import ArmKinematics

                model = ArmKinematics(j2_urdf_direction=sdk.config["j2_urdf_direction"])
                return execute_cartesian_line(
                    sdk, model, target, speed_m_s, joint_speed, trace,
                    cancel_event=cancel, on_sample=samples.put,
                )
            except Exception as error:
                return error

        def finish(result: CartesianMotionResult | Exception) -> None:
            self._cartesian_cancel = None
            self.cartesian_start_button.configure(state="normal")
            self.cartesian_stop_button.configure(state="disabled")
            self._show_latest_cartesian_sample(samples)
            if isinstance(result, Exception):
                message = f"直线运动失败：{result}"
            else:
                status = "已到达目标" if result.status == "completed" else "已停止"
                message = f"直线运动{status}，采样 {result.samples} 次。\n轨迹 CSV：{result.trace_path}"
            if not self.busy:
                self.message_var.set(message.splitlines()[0])
            self.cartesian_status_var.set(message)
            self._append_status(message)
            if self._close_after_cartesian_stop:
                self.root.after(0, self._close_when_idle)

        self._start_worker("执行笛卡尔直线并保存轨迹", run, finish)
        self._poll_cartesian_samples(cancel, samples)

    def _show_cartesian_pose(
        self, position_m: tuple[float, ...], rpy_deg: tuple[float, ...], feedback: str = ""
    ) -> None:
        xyz = ", ".join(f"{value:.5f}" for value in position_m)
        rpy = ", ".join(f"{value:.2f}" for value in rpy_deg)
        self.cartesian_pose_var.set(f"末端 XYZ (m) [{xyz}]　RPY (°) [{rpy}]{feedback}")

    def _show_latest_cartesian_sample(self, samples: SimpleQueue[CartesianSample]) -> None:
        latest = None
        while True:
            try:
                latest = samples.get_nowait()
            except Empty:
                break
        if latest is not None:
            speed = "—" if latest.speed_m_s is None else f"{latest.speed_m_s * 1000:.2f}"
            self._show_cartesian_pose(
                latest.position_m, latest.rpy_deg,
                f"　线速度 (mm/s) {speed}　偏离直线 (mm) {latest.cross_track_error_m * 1000:.2f}",
            )
            cancel = self._cartesian_cancel
            if cancel is not None and not cancel.is_set() and latest.waypoint_index > 0:
                self.cartesian_status_var.set(f"正在跟踪第 {latest.waypoint_index} 个直线路点并记录反馈。")

    def _poll_cartesian_samples(
        self, cancel: threading.Event, samples: SimpleQueue[CartesianSample]
    ) -> None:
        if self._cartesian_cancel is cancel:
            self._show_latest_cartesian_sample(samples)
            self.root.after(100, lambda: self._poll_cartesian_samples(cancel, samples))

    def _cancel_cartesian(self) -> None:
        cancel = getattr(self, "_cartesian_cancel", None)
        if cancel is not None and not cancel.is_set():
            cancel.set()
            self.cartesian_stop_button.configure(state="disabled")
            self.cartesian_status_var.set("正在停止直线运动并保存已采集的轨迹…")

    def _close_when_idle(self) -> None:
        if self.busy or self._pending_operation is not None:
            self.root.after(50, self._close_when_idle)
        else:
            self._close()

    def _read_joint_values(self, variables: list[tk.StringVar]) -> list[float]:
        try:
            values = [float(variable.get()) for variable in variables]
        except ValueError as error:
            raise ValueError("位置输入必须是数字") from error
        if len(values) != 6:
            raise ValueError("需要输入六个关节位置")
        if any(not math.isfinite(value) for value in values):
            raise ValueError("位置输入必须是有限数值")
        return values

    def _current_speed(self) -> float:
        try:
            value = float(self.speed_var.get())
        except ValueError as error:
            raise ValueError("统一速度上限必须是数字，单位 °/s") from error
        if not math.isfinite(value) or value <= 0:
            raise ValueError("统一速度上限必须大于零，单位 °/s")
        return value

    def _config_from_widgets(self) -> dict[str, Any]:
        config = dict(self.hardware_config)
        config["can_interface"] = self.can_var.get().strip()
        config["static_tendon_compensation"] = self.tendon_enabled_var.get()
        config["tendon_profile"] = self.tendon_profile_var.get().strip()
        config["j2_urdf_direction"] = -1 if self.j2_reversed_var.get() else 1
        try:
            config["motor_ids"] = [int(variable.get()) for variable in self.motor_id_vars]
        except ValueError as error:
            raise ValueError("六个 Motor ID 必须是整数") from error
        return ArmSdk._validate_config(config)

    def _select_tendon_profile(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self.root, title="选择 arm_hardware 绳驱参数", filetypes=[("JSON", "*.json")])
        if selected:
            self.tendon_profile_var.set(selected)

    def _open_pi_panel(self) -> None:
        if self._cycle_blocks_action():
            return
        if self.pi_panel is None or not self.pi_panel.window.winfo_exists():
            self.pi_panel = PiPanel(self)
        else:
            self.pi_panel.window.lift()
            self.pi_panel.window.focus_set()

    def _invalidate_pi_values(self) -> None:
        if self.pi_panel is not None and self.pi_panel.window.winfo_exists():
            self.pi_panel.invalidate_all()

    def _enable_can(self) -> None:
        if self._cycle_blocks_action():
            return
        if self.busy:
            messagebox.showinfo("操作进行中", "请等当前硬件请求完成。", parent=self.root)
            return
        try:
            interface = validate_interface(self.can_var.get().strip())
        except ValueError as error:
            messagebox.showerror("CAN 接口错误", str(error), parent=self.root)
            return
        if not messagebox.askyesno(
            "启用 CAN 接口",
            f"将重启 {interface} 并设置为 1000000 bit/s。\n"
            "操作会中断正在使用该接口的通信；请先确认电机已停止并安全保持。\n"
            "可能需要多次管理员认证。是否继续？",
            parent=self.root, icon="warning",
        ):
            return
        self._stop_live_read()
        self._invalidate_pi_values()
        if self.sdk is not None:
            try:
                self.sdk.close()
            except Exception as error:
                messagebox.showerror("关闭 SDK 连接失败", str(error), parent=self.root)
                return
            self.sdk = None
            self.connection_label.configure(text="未连接", foreground="black")
        for label in self.current_labels:
            label.configure(text="—")
        self.motor_position_label.configure(text="电机角度：CAN 重启后请重新读取")
        self.encoder_position_label.configure(text="")

        def configured(message: str) -> None:
            self.message_var.set(message)
            self._append_status(message)

        self._start_worker(
            f"启用 CAN 接口 {interface}", lambda: enable_can(interface), configured)

    def _connect(self) -> None:
        if self._cycle_blocks_action():
            return
        if self.busy:
            messagebox.showinfo("操作进行中", "请等当前硬件请求完成后再重连。", parent=self.root)
            return
        try:
            config = self._config_from_widgets()
            self._stop_live_read()
            self._invalidate_pi_values()
            if self.sdk is None:
                self.sdk = ArmSdk(config)
            else:
                self.sdk.reconfigure(config)
            self.hardware_config = config
            self.connection_label.configure(text=f"已连接 {config['can_interface']}", foreground="green")
        except Exception as error:
            if self.sdk is None:
                self.connection_label.configure(text="连接失败", foreground="red")
            else:
                self.connection_label.configure(text="应用失败", foreground="red")
                self.message_var.set("当前连接仍使用上次成功应用的配置。")
            messagebox.showerror("SDK_UI 连接失败", str(error), parent=self.root)
            return
        for label in self.current_labels:
            label.configure(text="—")
        self.motor_position_label.configure(text="电机角度：尚未读取")
        enabled = config["static_tendon_compensation"]
        self.encoder_position_label.configure(text=(
            "静态绳驱补偿已启用，读取后显示静态反解位置。" if enabled else ""
        ))
        self.message_var.set("SocketCAN 已连接；点击“读取绝对位置”检查 0x92 反馈。")
        try:
            _write_json(HARDWARE_CONFIG_PATH, config)
        except OSError as error:
            self._append_status(f"配置已应用，本机设置文件写入失败：{error}")

    def _ensure_sdk(self) -> ArmSdk:
        if self.sdk is None:
            raise RuntimeError("请先设置 CAN 接口和电机 ID，然后点击“连接 / 应用”")
        return self.sdk

    def _start_worker(
        self,
        description: str,
        operation: Callable[[], Any],
        on_success: Callable[[Any], None] | None = None,
        *,
        priority: bool = False,
        cycle_operation: bool = False,
    ) -> None:
        if priority:
            self._cancel_cartesian()
        if getattr(self, "_demo_cycle", None) is not None and not cycle_operation:
            if priority:
                self._stop_demo_cycle(send_stop=False, reason="收到停止或关断命令")
            else:
                self._cycle_blocks_action()
                return
        if self.busy:
            # A position poll is short-lived but still a sequence of CAN
            # transactions. Queue user actions instead of silently losing them.
            if self._live_read_inflight or priority:
                pending = self._pending_operation
                if pending is None or (priority and not pending[3]):
                    if pending is not None:
                        self._append_status(f"急停优先，取消尚未执行的操作：{pending[0]}")
                    self._pending_operation = (description, operation, on_success, priority)
                    self.message_var.set(f"等待当前硬件请求完成后执行：{description}")
                else:
                    messagebox.showinfo("操作已排队", "请等待已排队的硬件操作完成。", parent=self.root)
            else:
                messagebox.showinfo("操作进行中", "请等待当前硬件请求完成。", parent=self.root)
            return
        self.busy = True
        self.message_var.set(f"执行中：{description}")

        def worker() -> None:
            try:
                result = operation()
            except Exception as error:
                self.root.after(0, lambda current_error=error: self._worker_failed(description, current_error))
            else:
                self.root.after(0, lambda current_result=result: self._worker_succeeded(
                    description, current_result, on_success))

        threading.Thread(target=worker, daemon=True).start()

    def _run_pending_operation(self) -> None:
        pending = self._pending_operation
        self._pending_operation = None
        if pending is not None:
            description, operation, on_success, priority = pending
            self._start_worker(description, operation, on_success, priority=priority)

    def _worker_failed(self, description: str, error: Exception) -> None:
        self.busy = False
        self.message_var.set(f"{description}失败：{error}")
        self._append_status(str(error))
        # Dispatch a queued emergency before presenting a modal error dialog.
        if getattr(self, "_demo_cycle", None) is not None:
            self._stop_demo_cycle(send_stop=True, reason=f"{description}失败：{error}")
        else:
            self._run_pending_operation()
        messagebox.showerror("硬件操作失败", str(error), parent=self.root)

    def _worker_succeeded(
        self,
        description: str,
        result: Any,
        on_success: Callable[[Any], None] | None,
    ) -> None:
        self.busy = False
        self.message_var.set(f"完成：{description}")
        # PI and other success handlers can display modal dialogs. Dispatch a
        # queued emergency before entering one; it must not wait for dismissal.
        priority_dispatched = bool(self._pending_operation and self._pending_operation[3])
        if priority_dispatched:
            self._run_pending_operation()
        if on_success is not None:
            try:
                on_success(result)
            except Exception as error:
                if priority_dispatched:
                    # A newly dispatched emergency still owns busy. Do not
                    # clear it through _worker_failed while handling UI errors.
                    self._append_status(f"{description}结果处理失败：{error}")
                    messagebox.showerror("操作结果处理失败", str(error), parent=self.root)
                else:
                    self._worker_failed(description, error)
                return
        if not priority_dispatched:
            self._run_pending_operation()
        self._maybe_send_cycle_leg()

    def _run_simple(
        self, description: str, operation: Callable[[ArmSdk], None], *, priority: bool = False
    ) -> None:
        self._start_worker(description, lambda: operation(self._ensure_sdk()), priority=priority)

    def _toggle_live_read(self) -> None:
        if self.live_read_enabled:
            if self._cycle_blocks_action():
                return
            self._stop_live_read()
            return
        try:
            self._ensure_sdk()
        except Exception as error:
            messagebox.showerror("持续读取失败", str(error), parent=self.root)
            return
        self.live_read_enabled = True
        self._live_generation += 1
        self.live_read_button.configure(text="停止持续读取")
        self.live_read_status.set("持续读取：运行中（完成一次读取后间隔 250 ms）")
        self._schedule_live_read(0)

    def _stop_live_read(self) -> None:
        self.live_read_enabled = False
        self._live_generation += 1
        if self._live_after_id is not None:
            try:
                self.root.after_cancel(self._live_after_id)
            except tk.TclError:
                pass
            self._live_after_id = None
        self.live_read_button.configure(text="开始持续读取")
        self.live_read_status.set("持续读取：已停止")

    def _schedule_live_read(self, delay_ms: int = LIVE_READ_INTERVAL_MS) -> None:
        if self.live_read_enabled and self._live_after_id is None:
            self._live_after_id = self.root.after(delay_ms, self._live_read_tick)

    def _live_read_tick(self) -> None:
        self._live_after_id = None
        if not self.live_read_enabled:
            return
        if self.busy:
            self._schedule_live_read()
            return
        try:
            sdk = self._ensure_sdk()
        except Exception as error:
            self._stop_live_read()
            self.live_read_status.set(f"持续读取已停止：{error}")
            if getattr(self, "_demo_cycle", None) is not None:
                self._stop_demo_cycle(send_stop=True, reason=f"位置连接失效：{error}")
            messagebox.showerror("持续读取失败", str(error), parent=self.root)
            return
        generation = self._live_generation
        self.busy = True
        self._live_read_inflight = True

        def worker() -> None:
            try:
                snapshot = sdk.read_positions()
            except Exception as error:
                self.root.after(0, lambda current_error=error: self._live_read_finished(
                    generation, None, current_error))
            else:
                self.root.after(0, lambda: self._live_read_finished(generation, snapshot, None))

        threading.Thread(target=worker, daemon=True).start()

    def _live_read_finished(
        self, generation: int, snapshot: PositionSnapshot | None, error: Exception | None
    ) -> None:
        self.busy = False
        self._live_read_inflight = False
        current = self.live_read_enabled and generation == self._live_generation
        if current and error is not None:
            self._stop_live_read()
            self.live_read_status.set(f"持续读取已停止：{error}")
            for label in self.current_labels:
                label.configure(text="—")
            self.motor_position_label.configure(text="电机角度：读取失败，位置未知")
            self._append_status(f"持续读取失败：{error}")
        elif current and snapshot is not None:
            # Continuous reads update only the display; Demo0 is saved solely
            # by the existing explicit read-and-save action.
            self._show_snapshot(snapshot)
            self._observe_demo_cycle(snapshot)
        # A user command (especially emergency stop) precedes the next poll.
        self._run_pending_operation()
        if current and error is not None and getattr(self, "_demo_cycle", None) is not None:
            self._stop_demo_cycle(send_stop=True, reason=f"位置读取失败：{error}")
        self._maybe_send_cycle_leg()
        if current and error is not None:
            messagebox.showerror("持续读取失败", str(error), parent=self.root)
        self._schedule_live_read()

    def _run_joint(self, index: int, enabled: bool) -> None:
        action = "运行" if enabled else "关断"
        stop_cycle_motors = not enabled and getattr(self, "_demo_cycle", None) is not None

        def operation() -> tuple[int, ...]:
            sdk = self._ensure_sdk()
            if stop_cycle_motors:
                sdk.emergency_stop()
            return sdk.joint_motor_on(index) if enabled else sdk.joint_motor_off(index)

        def show_result(motor_ids: tuple[int, ...]) -> None:
            self._append_status(
                f"{JOINT_NAMES[index]} {action}完成，涉及电机 ID: {list(motor_ids)}。"
                "这些电机可能与其他关节共享。"
            )

        self._start_worker(f"{JOINT_NAMES[index]} {action}", operation, show_result, priority=not enabled)

    def _set_motor_zero(self) -> None:
        if self._cycle_blocks_action():
            return
        if self.busy:
            return
        try:
            self._ensure_sdk()
        except Exception as error:
            messagebox.showerror("电机编码器设零失败", str(error), parent=self.root)
            return
        if not messagebox.askyesno(
            "确认电机编码器设零",
            "将六个电机的当前位置写入电机编码器零位（0x19），并持久写入电机 ROM。\n\n"
            "这会改变硬件标定及其他应用使用的位置坐标；已保存的 Demo 不会自动更新。"
            "软件电机零偏不会自动修改，0x92 多圈位置不一定清零；必须重新核验并校准零偏。"
            "请确认机械臂当前处于正确的标定位置。\n\n"
            "确定对六个电机执行设零吗？",
            parent=self.root,
            icon="warning",
        ):
            return
        # A timeout can still mean that the command reached a motor.
        self._stop_live_read()
        for label in self.current_labels:
            label.configure(text="—")
        self.motor_position_label.configure(text="电机角度：设零后请重新读取")
        self._start_worker(
            "电机编码器设零 (0x19)",
            lambda: self._ensure_sdk().set_current_positions_as_zero(),
            self._motor_zero_finished,
        )

    def _motor_zero_finished(self, results: tuple[Any, ...]) -> None:
        lines = ["电机编码器设零结果："]
        errors = False
        for result in results:
            if result.error is None:
                lines.append(f"ID {result.motor_id}: 已确认，编码器零偏 {result.encoder_zero_offset}")
            else:
                errors = True
                lines.append(f"ID {result.motor_id}: 未确认，{result.error}")
        self._append_status("\n".join(lines))
        if errors:
            self.message_var.set("设零结果部分或全部未确认，请检查反馈后重新读取位置。")
            messagebox.showwarning("电机编码器设零未全部确认", "\n".join(lines), parent=self.root)
        else:
            self.message_var.set("六电机设零已确认；请重新读取位置，核验软件零偏、ROS 坐标和 Demo。")

    def _read_positions(self) -> None:
        self._start_worker(
            "读取 0x92 绝对位置",
            lambda: self._ensure_sdk().read_positions(),
            self._positions_read,
        )

    def _positions_read(self, snapshot: PositionSnapshot) -> None:
        self.demos["demo0"] = list(snapshot.joint_deg)
        self._update_demo_fields("demo0")
        _write_json(DEMO_CONFIG_PATH, self.demos)
        self._show_snapshot(snapshot)
        self._append_status("Demo0 已更新为本次读取到的当前关节位置。")

    def _capture_demo(self, key: str) -> None:
        self._start_worker(
            f"读取当前位置并写入 {key.upper()}",
            lambda: self._ensure_sdk().read_positions(),
            lambda snapshot: self._capture_demo_finished(key, snapshot),
        )

    def _capture_demo_finished(self, key: str, snapshot: PositionSnapshot) -> None:
        updated_demos = dict(self.demos)
        updated_demos[key] = list(snapshot.joint_deg)
        _write_json(DEMO_CONFIG_PATH, updated_demos)
        self.demos = updated_demos
        self._update_demo_fields(key)
        self._show_snapshot(snapshot)
        self._append_status(f"{key.upper()} 已保存本次读取的六个关节位置。")

    def _show_snapshot(self, snapshot: PositionSnapshot) -> None:
        for label, value in zip(self.current_labels, snapshot.joint_deg):
            label.configure(text=f"{value:.4f}")
        raw = ", ".join(f"{value:.2f}" for value in snapshot.raw_motor_deg)
        self.motor_position_label.configure(text=f"电机 0x92 原始角度 (°)：[{raw}]")
        encoder = snapshot.encoder_equivalent_joint_deg
        self.encoder_position_label.configure(text=(
            f"上方为静态反解位置。电机等效关节角 (°)：{[round(value, 6) for value in encoder]}"
            if encoder is not None else ""
        ))

    def _read_status(self) -> None:
        self._start_worker("读取六电机状态", lambda: self._ensure_sdk().read_status(), self._show_status)

    def _show_status(self, statuses: list[tuple[Any, Any]]) -> None:
        lines = ["ID  STATE TEMP(°C) VOLT(V) CURR(A) FAULT IQ(raw) SPEED(°/s) ENCODER"]
        ids = self.hardware_config["motor_ids"]
        for motor_id, (status1, status2) in zip(ids, statuses):
            state = "ON" if status1.motor_state == 0 else f"0x{status1.motor_state:02X}"
            lines.append(
                f"{motor_id:<3} {state:<5} {status1.temperature_c:>4} "
                f"{status1.voltage_v:>7.2f} {status1.current_a:>7.2f} "
                f"0x{status1.fault_flags:02X} {status2.iq_raw:>7} "
                f"{status2.speed_dps:>10} {status2.encoder_raw:>7}"
            )
        self._set_status_text("\n".join(lines))

    def _send_absolute(self) -> None:
        try:
            target = self._read_joint_values(self.absolute_vars)
            speed = self._current_speed()
        except ValueError as error:
            messagebox.showerror("输入错误", str(error), parent=self.root)
            return
        self._start_worker(
            "发送逻辑关节绝对位置",
            lambda: self._ensure_sdk().move_absolute(target, speed),
            self._show_move_result,
        )

    def _send_increment(self) -> None:
        try:
            delta = self._read_joint_values(self.increment_vars)
            speed = self._current_speed()
        except ValueError as error:
            messagebox.showerror("输入错误", str(error), parent=self.root)
            return
        self._start_worker(
            "读取当前位置并发送逻辑关节增量",
            lambda: self._ensure_sdk().move_increment(delta, speed),
            self._show_move_result,
        )

    def _move_zero(self) -> None:
        try:
            speed = self._current_speed()
        except ValueError as error:
            messagebox.showerror("输入错误", str(error), parent=self.root)
            return
        self._start_worker(
            "快速回逻辑零位",
            lambda: self._ensure_sdk().move_absolute([0.0] * 6, speed),
            self._show_move_result,
        )

    def _move_demo(self, key: str) -> None:
        try:
            target = [float(variable.get()) for variable in self.demo_vars[key]]
            speed = self._current_speed()
        except ValueError as error:
            messagebox.showerror("Demo 位置错误", str(error), parent=self.root)
            return
        self._start_worker(
            f"绝对移动到 {key.upper()}",
            lambda: self._ensure_sdk().move_absolute(target, speed),
            self._show_move_result,
        )

    def _cycle_blocks_action(self) -> bool:
        if getattr(self, "_demo_cycle", None) is None:
            return False
        messagebox.showinfo("循环运动中", "请先点击当前 Demo 的“停止循环”再执行此操作。", parent=self.root)
        return True

    def _toggle_demo_cycle(self, key: str) -> None:
        if getattr(self, "_demo_cycle", None) is not None:
            if key == self._cycle_key:
                self._stop_demo_cycle(send_stop=True, reason="用户停止")
            else:
                self._cycle_blocks_action()
            return
        if self.busy or self._pending_operation is not None:
            messagebox.showinfo("操作进行中", "请等当前请求完成后开始循环。", parent=self.root)
            return
        try:
            sdk = self._ensure_sdk()
            cycle = DemoCycle(
                self._read_joint_values(self.demo_vars[key]), self._current_speed(),
                float(self.cycle_tolerance_var.get()),
            )
            # Validate BOTH endpoints before enabling or moving any motor.
            sdk._check_joint_position_limits(cycle.target, "循环逻辑零位")
            sdk._check_joint_position_limits(cycle.demo_target, "循环 Demo 位置")
        except Exception as error:
            messagebox.showerror("无法开始循环", str(error), parent=self.root)
            return
        self._demo_cycle = cycle
        self._cycle_key = key
        self._cycle_generation += 1
        self._cycle_needs_leg = True
        self._cycle_started_live_read = not self.live_read_enabled
        for name, button in self.cycle_buttons.items():
            button.configure(text="停止循环" if name == key else "零点往返循环",
                             state="normal" if name == key else "disabled")
        self.cycle_status.set(f"{key.upper()}：准备前往逻辑零位")
        self._append_status(f"开始 {key.upper()} 循环：逻辑零位 ↔ Demo；速度与目标已固定为本次输入。")
        self._maybe_send_cycle_leg(first=True)

    def _maybe_send_cycle_leg(self, *, first: bool = False) -> None:
        cycle = getattr(self, "_demo_cycle", None)
        if cycle is None or not self._cycle_needs_leg or self.busy:
            return
        self._cycle_needs_leg = False
        generation = self._cycle_generation
        sdk = self._ensure_sdk()
        target = cycle.target
        name = self._cycle_key.upper()
        destination = name if cycle.to_demo else "逻辑零位"

        def operation() -> MoveResult | None:
            if self._demo_cycle is not cycle or generation != self._cycle_generation:
                return None
            if first:
                sdk.motor_on()
            if self._demo_cycle is not cycle or generation != self._cycle_generation:
                return None
            return sdk.move_absolute(target, cycle.speed_dps)

        def sent(result: MoveResult | None) -> None:
            if self._demo_cycle is not cycle or generation != self._cycle_generation or result is None:
                return
            cycle.begin_leg(result.duration_s, time.monotonic())
            self.cycle_status.set(f"{name}：前往{destination}；已完成往返 {cycle.round_trips} 次")
            self._show_move_result(result)
            if not self.live_read_enabled:
                self.live_read_enabled = True
                self._live_generation += 1
                self.live_read_button.configure(text="停止持续读取")
                self.live_read_status.set("持续读取：循环运动自动读取中")
            self._schedule_live_read(0)

        self._start_worker(f"{name} 循环：前往{destination}", operation, sent, cycle_operation=True)

    def _observe_demo_cycle(self, snapshot: PositionSnapshot) -> None:
        cycle = getattr(self, "_demo_cycle", None)
        if cycle is None:
            return
        destination = self._cycle_key.upper() if cycle.to_demo else "逻辑零位"
        try:
            arrived = cycle.observe(snapshot.joint_deg, time.monotonic())
        except Exception as error:
            self._stop_demo_cycle(send_stop=True, reason=str(error))
            self._append_status(str(error))
            return
        self.cycle_status.set(
            f"{self._cycle_key.upper()}：目标{destination}；最大误差 {cycle.error_deg:.4f}°；"
            f"已完成往返 {cycle.round_trips} 次")
        if arrived:
            self._cycle_needs_leg = True

    def _stop_demo_cycle(self, *, send_stop: bool, reason: str) -> None:
        if getattr(self, "_demo_cycle", None) is None:
            return
        key = self._cycle_key
        self._demo_cycle = None
        self._cycle_key = None
        self._cycle_generation += 1
        self._cycle_needs_leg = False
        if self._cycle_started_live_read:
            self._stop_live_read()
        for button in self.cycle_buttons.values():
            button.configure(text="零点往返循环", state="normal")
        self.cycle_status.set(f"{key.upper()} 循环已停止：{reason}")
        self._append_status(f"{key.upper()} 循环已停止：{reason}")
        if send_stop:
            def stopped(_: Any) -> None:
                if self._close_after_cycle_stop:
                    self._close_after_cycle_stop = False
                    self._close()
            self._start_worker("停止循环并发送急停", lambda: self._ensure_sdk().emergency_stop(),
                               stopped, priority=True)

    def _show_move_result(self, result: MoveResult) -> None:
        speeds = ", ".join(str(value) for value in result.applied_motor_speed_dps)
        requested = ", ".join(f"{value:.1f}" for value in result.requested_motor_speed_dps)
        self._append_status(
            "本次目标已发送。\n"
            f"目标关节角 (°): {[round(v, 4) for v in result.target_joint_deg]}\n"
            f"按统一速度映射的电机速度 (°/s): [{requested}]\n"
            f"受电机速度上限约束后发送 (°/s): [{speeds}]\n"
            f"公共规划时长估算: {result.duration_s:.3f} s"
        )
        if result.tendon is not None:
            self._append_status(
                f"关节补偿 (°) {[round(v, 8) for v in result.tendon.compensation_deg]}\n"
                f"补偿后关节等效角 (°) {[round(v, 8) for v in result.tendon.effective_deg]}\n"
                f"电机补偿 (°) {[round(v, 8) for v in result.motor_compensation_deg]}\n"
                f"A4 目标原始电机角 (°) {list(result.target_raw_motor_deg)}\n"
                f"到位静态推算 (°) {[round(v, 8) for v in result.predicted_joint_deg]}"
            )

    def _save_demo(self, key: str) -> None:
        try:
            values = [float(variable.get()) for variable in self.demo_vars[key]]
        except ValueError:
            messagebox.showerror("Demo 位置错误", "六个关节位置必须全部是数字。", parent=self.root)
            return
        if any(not math.isfinite(value) for value in values):
            messagebox.showerror("Demo 位置错误", "六个关节位置必须是有限数值。", parent=self.root)
            return
        self.demos[key] = values
        _write_json(DEMO_CONFIG_PATH, self.demos)
        self.message_var.set(f"{key.upper()} 已保存到本机配置文件。")

    def _update_demo_fields(self, key: str) -> None:
        for variable, value in zip(self.demo_vars[key], self.demos[key]):
            variable.set(f"{value:.4f}")

    def _append_status(self, text: str) -> None:
        old = self.status_text.get("1.0", "end-1c")
        combined = (old + "\n\n" + text).strip()
        self._set_status_text(combined)

    def _set_status_text(self, value: str) -> None:
        self.status_text.configure(state="normal")
        self.status_text.delete("1.0", "end")
        self.status_text.insert("1.0", value)
        self.status_text.configure(state="disabled")

    def _close(self) -> None:
        if getattr(self, "_cartesian_cancel", None) is not None:
            self._close_after_cartesian_stop = True
            self._stop_live_read()
            self._cancel_cartesian()
            return
        if getattr(self, "_demo_cycle", None) is not None:
            self._close_after_cycle_stop = True
            self._stop_demo_cycle(send_stop=True, reason="关闭窗口")
            return
        self._stop_live_read()
        if self.busy:
            messagebox.showinfo("操作进行中", "请等当前硬件请求完成后再关闭窗口。", parent=self.root)
            return
        if self.sdk is not None:
            self.sdk.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ArmSdkUi(root)
    root.mainloop()


if __name__ == "__main__":
    main()
