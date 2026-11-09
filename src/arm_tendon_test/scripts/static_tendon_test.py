#!/usr/bin/env python3
"""Offline UI for the hardware package's static tendon model."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
import tkinter as tk
from tkinter import filedialog, ttk
from typing import Sequence
import xml.etree.ElementTree as ET


def add_sdk_path() -> None:
    locations = [parent for parent in Path(__file__).resolve().parents]
    locations.extend(Path(prefix) / "share" for prefix in
                     os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if prefix)
    for location in locations:
        sdk = location / "arm_hardware" / "SDK_UI"
        if (sdk / "sdk_ui" / "tendon_compensation.py").is_file():
            sys.path.insert(0, str(sdk))
            return


add_sdk_path()
from sdk_ui.app import DEFAULT_HARDWARE_CONFIG, HARDWARE_CONFIG_PATH
from sdk_ui.arm_sdk import ArmSdk
from sdk_ui.motor_coordinates import check_position_limits, joint_to_motor, motor_to_joint, quantize_a4
from sdk_ui.tendon_compensation import ENCODER_LIMIT_TOLERANCE_DEG, TendonModel, package_file


@dataclass(frozen=True)
class MotorMapping:
    matrix: Sequence[Sequence[float]]
    directions: Sequence[float]
    offsets_deg: Sequence[float]
    limits_deg: Sequence[Sequence[float]]
    source: Path
    ros_rounding: bool

    def raw(self, joint_deg: Sequence[float]) -> list[float]:
        return [direction * motor + offset for direction, motor, offset in zip(
            self.directions, joint_to_motor(self.matrix, joint_deg), self.offsets_deg
        )]

    def decode(self, raw_deg: Sequence[float]) -> list[float]:
        return motor_to_joint(self.matrix, [
            direction * (motor - offset) for direction, motor, offset in zip(
                self.directions, raw_deg, self.offsets_deg
            )
        ])


def load_mapping(use_ros: bool, profile: str) -> tuple[TendonModel, MotorMapping]:
    if use_ros:
        model = TendonModel(profile)
        path = package_file("arm_moveit_config", "config/arm_R.ros2_control.xacro")
        params = {node.attrib["name"]: node.text
                  for node in ET.parse(path).findall(".//ros2_control/hardware/param")}
        return model, MotorMapping(
            model.coupled_motor_matrix(),
            [float(params[f"motor{i}_direction"]) for i in range(1, 7)],
            [math.degrees(float(params[f"motor{i}_zero_offset_rad"])) for i in range(1, 7)],
            model.position_limits_deg, path, True,
        )
    path = HARDWARE_CONFIG_PATH if HARDWARE_CONFIG_PATH.is_file() else DEFAULT_HARDWARE_CONFIG
    config = ArmSdk._validate_config(json.loads(path.read_text(encoding="utf-8")))
    model = TendonModel(
        profile or config["tendon_profile"], j2_urdf_direction=config["j2_urdf_direction"]
    )
    return model, MotorMapping(
        config["joint_to_motor"], config["motor_directions"], config["motor_zero_offset_deg"],
        config["joint_position_limits_deg"], path, False,
    )


def check_command(
    model: TendonModel, mapping: MotorMapping, nominal: Sequence[float],
    effective: Sequence[float], encoded_joint: Sequence[float],
) -> None:
    tolerance = ENCODER_LIMIT_TOLERANCE_DEG
    model.check_limits(nominal, "目标角")
    model.check_limits(effective, "补偿后等效角")
    model.check_limits(encoded_joint, "量化后等效角", tolerance)
    predicted = model.estimate(encoded_joint, nominal)
    if mapping.ros_rounding:
        return
    for angles, label in ((nominal, "目标角"), (effective, "补偿后等效角"),
                          (encoded_joint, "量化后等效角"), (predicted, "静态推算角")):
        check_position_limits(angles, mapping.limits_deg, label)


def fmt(value: float) -> str:
    return f"{value:.9g}" if value else "0"


class OfflineUi:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("六轴机械臂 · 静态绳驱补偿离线测试")
        root.geometry("1160x790")
        root.minsize(990, 730)
        panel = ttk.Frame(root, padding=14)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="静态绳驱补偿离线测试", font=("TkDefaultFont", 16, "bold")).pack(anchor="w")
        ttk.Label(panel, text="按所选映射的关节角约定输入。A4 指令和 0x92 读数在本机模拟。").pack(
            anchor="w", pady=(2, 8))

        settings = ttk.Frame(panel)
        settings.pack(fill="x")
        ttk.Label(settings, text="电机映射").pack(side="left")
        self.mapping = tk.StringVar(value="ROS 整机")
        ttk.Combobox(settings, textvariable=self.mapping, values=("ROS 整机", "SDK_UI"),
                     state="readonly", width=12).pack(side="left", padx=(5, 14))
        ttk.Label(settings, text="共享参数文件（留空自动选择）").pack(side="left")
        self.profile = tk.StringVar()
        ttk.Entry(settings, textvariable=self.profile).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(settings, text="浏览…", command=self.select_profile).pack(side="right")
        self.source = tk.StringVar()
        ttk.Label(panel, textvariable=self.source, wraplength=940).pack(anchor="w", pady=(4, 4))

        self.targets = self.input_row(panel, "目标关节角（°）", "J", "0", "计算补偿并模拟反馈", self.calculate)
        self.joints = self.table(panel, "关节侧与对拉绳", (
            "关节", "目标 (°)", "保持力矩 (N·m)", "补偿 Δq (°)", "补偿后等效角 (°)",
            "左绳变化 (mm)", "右绳变化 (mm)", "左绳张力增量 (N)", "右绳张力增量 (N)",
        ))
        self.motors = self.table(panel, "电机侧原始角度（°）", (
            "电机轴", "方向", "无补偿目标", "电机补偿量", "补偿后目标", "A4 量化目标",
        ))
        self.raw_feedback = self.input_row(
            panel, "电机 0x92 反馈（原始 °）", "M", "", "反解输入的反馈", self.invert
        )
        self.feedback = self.table(panel, "静态位置反馈（°）", (
            "读数类型", *(f"J{i}" for i in range(1, 7))
        ), rows=3, first_width=190)
        self.command_status = tk.StringVar()
        self.feedback_status = tk.StringVar()
        for status in (self.command_status, self.feedback_status):
            ttk.Label(panel, textvariable=status, wraplength=940).pack(anchor="w", pady=(3, 0))
        self.calculate()

    def select_profile(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root, title="选择 arm_hardware 静态绳驱参数", filetypes=[("JSON", "*.json")])
        if path:
            self.profile.set(path)

    @staticmethod
    def input_row(panel, title, prefix, initial, button, callback) -> list[tk.StringVar]:
        frame = ttk.LabelFrame(panel, text=title, padding=(8, 4))
        frame.pack(fill="x", pady=(4, 2))
        values = []
        for i in range(6):
            ttk.Label(frame, text=f"{prefix}{i + 1}").pack(side="left", padx=(4, 2))
            variable = tk.StringVar(value=initial)
            ttk.Entry(frame, textvariable=variable, width=13).pack(side="left", padx=(0, 8))
            values.append(variable)
        ttk.Button(frame, text=button, command=callback).pack(side="right", padx=4)
        return values

    @staticmethod
    def table(panel, title, columns, rows=6, first_width=60) -> ttk.Treeview:
        frame = ttk.LabelFrame(panel, text=title, padding=(4, 3))
        frame.pack(fill="x", pady=(4, 2))
        tree = ttk.Treeview(frame, columns=columns, show="headings", height=rows, selectmode="none")
        for index, column in enumerate(columns):
            tree.heading(column, text=column)
            tree.column(column, width=first_width if index == 0 else 108, anchor="e")
        tree.pack(fill="x")
        return tree

    @staticmethod
    def fill(table: ttk.Treeview, rows: Sequence[Sequence[float | str]]) -> None:
        table.delete(*table.get_children())
        for row in rows:
            table.insert("", "end", values=[value if isinstance(value, str) else fmt(value) for value in row])

    @staticmethod
    def values(inputs: list[tk.StringVar], name: str) -> list[float]:
        try:
            result = [float(variable.get()) for variable in inputs]
        except ValueError as error:
            raise ValueError(f"请为 {name} 输入六个数值。") from error
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} 必须是有限数值。")
        return result

    def context(self) -> tuple[TendonModel, MotorMapping]:
        model, mapping = load_mapping(self.mapping.get() == "ROS 整机", self.profile.get().strip())
        self.source.set(f"参数  {model.profile}\n映射  {mapping.source}")
        return model, mapping

    def calculate(self) -> None:
        for table in (self.joints, self.motors, self.feedback):
            self.fill(table, [])
        for variable in self.raw_feedback:
            variable.set("")
        self.command_status.set("")
        self.feedback_status.set("")
        self.source.set("")
        try:
            model, mapping = self.context()
            nominal = self.values(self.targets, "目标关节角")
            result = model.evaluate(nominal)
            self.fill(self.joints, [
                (f"J{i + 1}", nominal[i], result.hold_torque_nm[i], result.compensation_deg[i],
                 result.effective_deg[i], result.rope_a_delta_mm[i], -result.rope_a_delta_mm[i],
                 result.rope_a_delta_n[i], result.rope_b_delta_n[i]) for i in range(6)
            ])
            raw_base = mapping.raw(nominal)
            raw_target = mapping.raw(result.effective_deg)
            encoded = quantize_a4(raw_target, ros_rounding=mapping.ros_rounding)
            self.fill(self.motors, [
                (f"M{i + 1}", mapping.directions[i], raw_base[i], raw_target[i] - raw_base[i],
                 raw_target[i], encoded[i]) for i in range(6)
            ])
            for variable, raw in zip(self.raw_feedback, encoded):
                variable.set(f"{raw:.2f}")
        except (OSError, ValueError, RuntimeError, KeyError, TypeError, ET.ParseError) as error:
            self.command_status.set(f"计算失败  {error}")
            return
        try:
            check_command(model, mapping, nominal, result.effective_deg, mapping.decode(encoded))
            self.command_status.set("目标通过限位和静态终点检查。")
        except ValueError as error:
            self.command_status.set(f"实机将拒绝此目标  {error}")
        self.show_feedback(model, mapping, encoded, simulated=True)

    def show_feedback(
        self, model: TendonModel, mapping: MotorMapping, raw: Sequence[float], *, simulated: bool = False
    ) -> None:
        encoder = None
        try:
            encoder = mapping.decode(raw)
            estimated = model.estimate(encoder)
        except ValueError as error:
            if encoder is not None:
                self.fill(self.feedback, [("编码器等效角", *encoder)])
            self.feedback_status.set(str(error))
            return
        self.fill(self.feedback, [
            ("编码器等效角", *encoder), ("静态反解关节角", *estimated),
            ("反解扣除的补偿", *(a - b for a, b in zip(encoder, estimated))),
        ])
        self.feedback_status.set(
            "反馈来源  模拟电机到达 A4 量化目标" if simulated else "反馈来源  手动输入的六台电机读数"
        )

    def invert(self) -> None:
        self.fill(self.feedback, [])
        try:
            model, mapping = self.context()
            self.show_feedback(model, mapping, self.values(self.raw_feedback, "电机反馈"))
        except (OSError, ValueError, RuntimeError, KeyError, TypeError, ET.ParseError) as error:
            self.feedback_status.set(f"反解失败  {error}")


def main() -> None:
    root = tk.Tk()
    OfflineUi(root)
    root.mainloop()


if __name__ == "__main__":
    main()
