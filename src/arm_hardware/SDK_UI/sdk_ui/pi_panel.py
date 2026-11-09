"""Separate six-motor PID parameter window for the standalone SDK UI."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

from .arm_sdk import PI_HISTORY_PATH
from .protocol import PidGains


_FIELDS = (
    "position_kp", "position_ki", "speed_kp", "speed_ki", "torque_kp", "torque_ki",
)
_HEADERS = ("位置 Kp", "位置 Ki", "速度 Kp", "速度 Ki", "转矩 Kp", "转矩 Ki")


class PiPanel:
    """Edit one motor at a time, always reading before a persistent write."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner
        self.window = tk.Toplevel(owner.root)
        self.window.title("六电机三环 PI 参数")
        self.window.geometry("1120x700")
        self.window.minsize(950, 620)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.baselines: list[PidGains | None] = [None] * 6
        self.baseline_devices: list[tuple[str, int] | None] = [None] * 6
        # Keep the two displays independent: only the editable values are sent.
        self.read_values: list[list[tk.StringVar]] = []
        self.values: list[list[tk.StringVar]] = []
        self.motor_labels: list[ttk.Label] = []
        self.status_labels: list[ttk.Label] = []

        viewport = ttk.Frame(self.window)
        viewport.pack(fill="both", expand=True)
        canvas = tk.Canvas(viewport, highlightthickness=0)
        vertical = ttk.Scrollbar(viewport, orient="vertical", command=canvas.yview)
        horizontal = ttk.Scrollbar(viewport, orient="horizontal", command=canvas.xview)
        canvas.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        vertical.pack(side="right", fill="y")
        horizontal.pack(side="bottom", fill="x")
        canvas.pack(side="left", fill="both", expand=True)
        frame = ttk.Frame(canvas, padding=12)
        content = canvas.create_window((0, 0), window=frame, anchor="nw")
        frame.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(content, width=max(event.width, frame.winfo_reqwidth())),
        )
        ttk.Label(frame, text="电机 / ID").grid(row=1, column=0, padx=3)
        ttk.Label(frame, text="区域").grid(row=1, column=1, padx=3)
        for index, header in enumerate(_HEADERS):
            ttk.Label(frame, text=header).grid(row=1, column=index + 2, padx=3)
        ttk.Label(frame, text="操作").grid(row=1, column=8, padx=3)
        ttk.Label(frame, text="状态").grid(row=1, column=9, padx=3)
        for index in range(6):
            read_row = 2 + index * 2
            edit_row = read_row + 1
            label = ttk.Label(frame, text=f"M{index + 1} / —")
            label.grid(row=read_row, column=0, rowspan=2, sticky="w", padx=3, pady=3)
            self.motor_labels.append(label)
            ttk.Label(frame, text="读取值（只读）", foreground="#666666").grid(
                row=read_row, column=1, sticky="w", padx=3)
            ttk.Label(frame, text="待写入（可编辑）").grid(
                row=edit_row, column=1, sticky="w", padx=3)
            read_row_values: list[tk.StringVar] = []
            edit_row_values: list[tk.StringVar] = []
            for column in range(6):
                read_variable = tk.StringVar(value="—")
                read_row_values.append(read_variable)
                # Tk's readonlybackground is explicit and not theme-dependent.
                tk.Entry(
                    frame, textvariable=read_variable, width=8, state="readonly",
                    readonlybackground="#e5e7eb", fg="#555555", relief="sunken",
                ).grid(row=read_row, column=column + 2, padx=3, pady=(3, 1))
                edit_variable = tk.StringVar(value="")
                edit_row_values.append(edit_variable)
                ttk.Entry(frame, textvariable=edit_variable, width=8).grid(
                    row=edit_row, column=column + 2, padx=3, pady=(1, 3))
            self.read_values.append(read_row_values)
            self.values.append(edit_row_values)
            ttk.Button(frame, text="读取", command=lambda i=index: self.read_one(i)).grid(
                row=read_row, column=8, padx=3)
            ttk.Button(frame, text="永久写入", command=lambda i=index: self.write_one(i)).grid(
                row=edit_row, column=8, padx=3)
            state = ttk.Label(frame, text="未读取", width=16)
            state.grid(row=read_row, column=9, rowspan=2, sticky="w", padx=3)
            self.status_labels.append(state)
        ttk.Button(frame, text="读取全部六台", command=self.read_all).grid(
            row=14, column=0, columnspan=3, sticky="w", pady=(8, 0))

    def close(self) -> None:
        if self.owner.busy:
            messagebox.showinfo("操作进行中", "请等当前请求完成后关闭窗口。", parent=self.window)
            return
        self.window.destroy()

    @staticmethod
    def _identity(sdk: Any, index: int) -> tuple[str, int]:
        return str(sdk.config["can_interface"]), int(sdk.config["motor_ids"][index])

    def _mark_unread(self, index: int, status: str) -> None:
        self.baselines[index] = None
        self.baseline_devices[index] = None
        for variable in self.read_values[index]:
            variable.set("—")
        self.status_labels[index].configure(text=status)

    def invalidate_all(self) -> None:
        """Discard readings on reconnect; keep unsent edits as drafts."""
        for index in range(6):
            self._mark_unread(index, "连接变更，请重读")
            self.motor_labels[index].configure(text=f"M{index + 1} / —")

    def _apply_read(self, index: int, identity: tuple[str, int], gains: PidGains) -> None:
        self.baselines[index] = gains
        self.baseline_devices[index] = identity
        self.motor_labels[index].configure(text=f"M{index + 1} / ID {identity[1]}")
        for read_variable, edit_variable, field in zip(
            self.read_values[index], self.values[index], _FIELDS
        ):
            value = str(getattr(gains, field))
            read_variable.set(value)
            edit_variable.set(value)
        self.status_labels[index].configure(text="读取成功")

    def _record_read_results(self, results: list[tuple[int, tuple[str, int], PidGains | None, str | None]]) -> None:
        failures = []
        for index, identity, gains, error in results:
            if error is None and gains is not None:
                self._apply_read(index, identity, gains)
            else:
                self._mark_unread(index, "读取失败")
                self.motor_labels[index].configure(text=f"M{index + 1} / ID {identity[1]}")
                failures.append(f"ID {identity[1]}: {error}")
        if failures:
            self.owner.message_var.set("PI 参数部分或全部读取失败；失败电机不可写入。")
            self.owner._append_status("PI 参数读取失败：\n" + "\n".join(failures))
            messagebox.showwarning("PI 读取未全部成功", "\n".join(failures), parent=self.window)
        else:
            self.owner._append_status(f"已读取 {len(results)} 台电机 PI 参数。")

    def _read(self, indices: tuple[int, ...]) -> None:
        if self.owner.busy:
            return
        try:
            sdk = self.owner._ensure_sdk()
        except Exception as error:
            messagebox.showerror("PI 读取失败", str(error), parent=self.window)
            return
        # Invalidate each baseline before starting: a failed read must never
        # leave a previously read value eligible for a persistent write.
        for index in indices:
            self._mark_unread(index, "读取中")

        def operation() -> list[tuple[int, tuple[str, int], PidGains | None, str | None]]:
            results = []
            for index in indices:
                identity = self._identity(sdk, index)
                try:
                    gains = sdk.read_pid(index)
                except Exception as error:
                    results.append((index, identity, None, str(error)))
                else:
                    results.append((index, identity, gains, None))
            return results

        self.owner._start_worker("读取电机 PI 参数", operation, self._record_read_results)

    def read_one(self, index: int) -> None:
        self._read((index,))

    def read_all(self) -> None:
        self._read(tuple(range(6)))

    def _input_gains(self, index: int) -> PidGains:
        try:
            values = [int(variable.get().strip(), 10) for variable in self.values[index]]
        except ValueError as error:
            raise ValueError("六个 PI 参数必须全部为 0～255 的整数。") from error
        if any(value < 0 or value > 255 for value in values):
            raise ValueError("六个 PI 参数必须全部为 0～255 的整数。")
        return PidGains(*values)

    def write_one(self, index: int) -> None:
        if self.owner.busy:
            return
        try:
            sdk = self.owner._ensure_sdk()
            identity = self._identity(sdk, index)
            baseline = self.baselines[index]
            if baseline is None or self.baseline_devices[index] != identity:
                self._mark_unread(index, "请先读取")
                raise ValueError("请先读取当前连接的对应电机 PI 参数，再进行永久写入。")
            desired = self._input_gains(index)
        except Exception as error:
            messagebox.showerror("PI 参数错误", str(error), parent=self.window)
            return

        changes = "\n".join(
            f"{label}: {getattr(baseline, name)} → {getattr(desired, name)}"
            for label, name in zip(_HEADERS, _FIELDS)
        )
        if not messagebox.askyesno(
            "确认永久写入 PI",
            f"CAN {identity[0]}，电机 ID {identity[1]}：\n{changes}\n\n"
            "将发送 0x32 写入电机 ROM；电机控制参数改变可能影响稳定性。\n"
            "写入前会重新读取并核对当前值，写后再读取核验。确定写入吗？",
            icon="warning", parent=self.window,
        ):
            return
        # A missing reply can still mean the ROM write occurred. Invalidate
        # the cached baseline before any I/O and require a fresh read to retry.
        self._mark_unread(index, "写入中")
        self.owner._start_worker(
            f"永久写入电机 ID {identity[1]} PI",
            lambda: sdk.write_pid_persistent(index, desired, baseline),
            lambda result: self._write_finished(index, identity, result),
        )

    def _write_finished(self, index: int, identity: tuple[str, int], result: Any) -> None:
        if result.status == "verified" and result.observed is not None:
            self._apply_read(index, identity, result.observed)
            self.status_labels[index].configure(text="已确认，回读一致")
            self.owner._append_status(
                f"电机 ID {result.motor_id} ROM 写入应答已确认、当前参数回读一致；"
                f"断电持久性需重启后核验。历史：{PI_HISTORY_PATH}"
            )
        else:
            self._mark_unread(index, "未确认，请重读")
            detail = result.error or result.status
            self.owner.message_var.set(f"电机 ID {result.motor_id} 写入结果未确认，请重新读取。")
            self.owner._append_status(f"电机 ID {result.motor_id} PI 写入结果 {result.status}：{detail}")
            messagebox.showwarning(
                "PI 写入结果未确认",
                f"电机 ID {result.motor_id}：{result.status}\n{detail}\n\n"
                "不能仅凭超时判断 ROM 未写入，请先重新读取并检查历史记录。",
                parent=self.window,
            )
