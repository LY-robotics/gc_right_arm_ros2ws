"""Configure one SocketCAN interface using fixed ip commands without a shell."""
from __future__ import annotations

import os
from pathlib import Path
import re
import subprocess
from typing import Callable

BITRATE = 1_000_000
_INTERFACE = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,14}\Z", re.ASCII)


def validate_interface(name: str) -> str:
    if not isinstance(name, str) or not _INTERFACE.fullmatch(name):
        raise ValueError("CAN 接口名应为 1–15 个英文字母、数字、下划线、点或连字符，且不能以点或连字符开头")
    return name


def _find_ip() -> str:
    # Fixed system locations; do not elevate a user-selected executable.
    for candidate in ("/usr/sbin/ip", "/sbin/ip", "/usr/bin/ip", "/bin/ip"):
        if Path(candidate).is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise RuntimeError("未找到系统 ip 命令，请安装 iproute2")


def _check_can_interface(interface: str) -> None:
    interface = validate_interface(interface)
    try:
        link_type = (Path("/sys/class/net") / interface / "type").read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as error:
        raise RuntimeError(f"无法读取接口 {interface} 的类型：接口不存在或无法访问") from error
    if link_type != "280":  # Linux ARPHRD_CAN
        raise RuntimeError(f"{interface} 不是 CAN 接口（类型 {link_type}），未执行任何操作")


def enable_can(
    interface: str, runner: Callable[..., subprocess.CompletedProcess[str]] | None = None
) -> str:
    """Run the three operations on just the validated interface, reporting partial failure.

    pkexec may show authentication for each step; sudo -n is used only when
    pkexec is unavailable. No shell or writable Python helper is elevated.
    """
    interface = validate_interface(interface)
    _check_can_interface(interface)
    ip = _find_ip()
    if runner is None:
        runner = subprocess.run
    if os.geteuid() == 0:
        prefix: list[str] = []
    else:
        pkexec = next((path for path in ("/usr/bin/pkexec", "/bin/pkexec")
                       if Path(path).is_file() and os.access(path, os.X_OK)), None)
        sudo = next((path for path in ("/usr/bin/sudo", "/bin/sudo")
                     if Path(path).is_file() and os.access(path, os.X_OK)), None)
        if pkexec:
            prefix = [pkexec]
        elif sudo:
            prefix = [sudo, "-n"]
        else:
            raise RuntimeError("未找到 pkexec 或 sudo，无法获得配置 CAN 所需的管理员权限")
    steps = (
        ("关闭接口", [ip, "link", "set", "dev", interface, "down"]),
        ("设置 CAN 波特率 1000000", [ip, "link", "set", "dev", interface, "type", "can", "bitrate", str(BITRATE)]),
        ("启用接口", [ip, "link", "set", "dev", interface, "up"]),
    )
    completed: list[str] = []
    for label, command in steps:
        try:
            result = runner(
                prefix + command, capture_output=True, text=True, check=False,
                timeout=120, stdin=subprocess.DEVNULL,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            state = "接口可能仍处于关闭状态。" if completed else "接口状态未知，请检查。"
            raise RuntimeError(f"{label}失败：{error}；已完成步骤：{', '.join(completed) or '无'}。{state}") from error
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or f"退出码 {result.returncode}").strip()
            state = "接口可能仍处于关闭状态。" if completed else "接口状态未知，请检查。"
            raise RuntimeError(
                f"{label}失败：{detail}；已完成步骤：{', '.join(completed) or '无'}。{state}\n"
                "请检查授权和接口状态；如认证未弹出，请在终端手动配置。"
            )
        completed.append(label)
    return f"{interface} 已配置为 CAN 1000000 bit/s 并启用。请重新连接 SDK。"
