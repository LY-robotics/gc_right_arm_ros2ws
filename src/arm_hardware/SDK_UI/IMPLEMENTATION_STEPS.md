# SDK_UI 实现步骤说明

## 1. 本次范围

新增 `src/arm_hardware/SDK_UI` 作为独立的真实硬件调试 SDK 与 GUI。本工具直接通过 Linux SocketCAN 与六个电机通信，不加载 ROS、MoveIt、URDF、Gazebo 或其他仿真文件。所有新增代码、依赖说明和默认参数都位于 hardware 包内。

本次同时将 hardware 插件中 Joint1 的绳驱比从 3.0 校正为用户最新给出的 2.8，使 SDK_UI 和 `ArmHardwareInterface` 使用同一套换算矩阵。其他五个电机映射关系不变。

## 2. 分步骤实现

### 第 1 步：建立独立 Python SDK 包

文件：

- `pyproject.toml`
- `sdk_ui/__init__.py`
- `sdk_ui/protocol.py`
- `sdk_ui/arm_sdk.py`

包不依赖 ROS，也不需要安装 `python-can`；通过 Python 标准库 `socket`、`struct` 访问 Linux PF_CAN/SocketCAN。`pyproject.toml` 提供 `muweidu-arm-sdk-ui` 命令行入口，也支持直接 `python3 -m sdk_ui.app` 运行。

`protocol.py` 实现标准 11 位 CAN ID、单次请求/匹配回复、0x92 多圈位置、0x9A 状态1、0x9C 状态2、0x88 运行、0x80 关断、0x81 急停和 0xA4 带速度的多圈绝对位置。每次请求按 CAN ID、命令码、8 字节 DLC 检查回复；请求超时会抛出包含 CAN 口、电机 ID 和命令码的错误。

### 第 2 步：定义逻辑关节和电机映射

文件：`sdk_ui/default_hardware.json`、`sdk_ui/arm_sdk.py`。

所有 UI 位置输入和显示使用逻辑关节角度“度”；电机角也用“度”表达，比例矩阵在度或弧度下相同。当前矩阵为：

```text
      q1     q2     q3     q4    q5   q6
m1  [-100.8,   0,     0,     0,    0,   0]  # -36 × 2.8
m2  [   0,   36,  -43.2,    0,    0,   0]
m3  [   0,  -36,  -43.2,    0,    0,   0]
m4  [   0,    0,     0,  -64.8,   0,   0]
m5  [   0,    0,     0,    16,   13,   0]
m6  [   0,    0,     0,     0,    0,  15]
```

读取 0x92 后，将电机原始多圈角度、方向和电机零偏换算为映射空间位置：

```text
m_state_i = direction_i × (raw_angle_i - zero_offset_i)
q_state = inverse(B) × m_state
```

发命令时先计算 `m_target = B × q_target`，再换到电机原始角度：

```text
raw_target_i = direction_i × m_target_i + zero_offset_i
```

UI `hardware.json` 中的 `motor_zero_offset_deg` 用度表示；ros2_control xacro 的对应零偏以弧度表示。电机方向只允许 `+1` 或 `-1`。

### 第 3 步：实现增量、绝对位置和速度上限

`ArmSdk.move_absolute()` 和 `move_increment()` 都先读取六台电机的 0x92 当前位置。绝对位置直接使用六个目标关节角；增量操作将六个输入增量加到刚读出的当前关节角，形成目标关节角。原实现将增量目标转换为电机相对增量并发送 0xA8；现已改为将读取快照与增量相加、经过限位校验后发送 0xA4 绝对目标，详见 [POSITION_LIMITS_LIVE_READ_STEPS.md](POSITION_LIMITS_LIVE_READ_STEPS.md)。

设当前关节向量为 `q0`、目标为 `q1`、共同速度上限为 `V`（度/秒）：

```text
Δq = q1 - q0
Δm = B × Δq
T_joint = max(abs(Δq_j)) / V
```

六个电机沿同一个逻辑关节路径运动，各自理想电机速度为：

```text
V_motor_i = abs(Δm_i) / T
```

`V_motor_i` 因此与该电机本次映射位移成比例。配置中的每电机安全速度上限 `C_i` 会放大共同时间：

```text
T = max(T_joint, max_i(abs(Δm_i) / C_i))
```

随后将 `abs(Δm_i)/T` 转成 0xA4 的整数度/秒速度字段，并按 `C_i` 限幅。公共时间被放大时六台电机仍按位移比例运行。Joint1 单轴动作的未限幅速度为 `V × 36 × 2.8`；例如关节速度输入 100°/s，Joint1 电机1 的未限幅需求为 10080°/s。默认硬件速度上限会进一步限制实际发送值，避免直接把这个换算值无条件写入电机。

增量速度和电机速度上限使用与绝对移动相同的共同时间规划；每个电机的速度上限按映射后的电机位移比例分配。现根据完整关节目标计算原始电机绝对位置，将方向与零偏应用到 0xA4 绝对目标。

### 第 4 步：加入运行、关断、状态读取和位置读取

`CanMotor` 暴露运行/关断/急停，读取状态1/状态2和读取多圈绝对位置。SDK 将每个操作串行应用于六个电机；即使一个电机通信失败，运行/关断类广播也会继续尝试其他 ID，并汇总错误。

- 运行：0x88。
- 关断：0x80。
- 急停：0x81。
- 读取状态：0x9A + 0x9C，展示温度、电压、电流、状态、故障位、原始 Iq、转速和编码器字段。
- 读取绝对位置：六次 0x92，展示原始电机角和反算的逻辑关节角。
- 快速回 0：发送逻辑目标 `[0,0,0,0,0,0]`，速度策略与其他绝对位置命令相同。

### 第 5 步：加入 Demo0…Demo5 与 Tkinter UI

文件：`sdk_ui/app.py`。

- Demo1…Demo5 有六关节绝对角度输入、保存按钮和“移动到此位置”按钮。
- 点击“读取绝对位置并保存 Demo0”后，六个实测逻辑关节位置自动保存到 Demo0。
- Demo0…Demo5 每一行均有绝对移动按钮。
- 单一速度输入作用于整个逻辑关节运动；界面会同时显示位移比例计算出的电机速度、速度上限约束后的实际发送值和估算公共时长。
- CAN 操作在后台线程执行，避免等待电机回复时冻结窗口。
- 通信错误通过状态区和错误对话框显示。

### 第 6 步：配置持久化与安装

首次启动时，默认硬件参数会复制到：

```text
~/.config/muweidu_arm_sdk_ui/hardware.json
```

CAN 口和六个 ID 可以在窗口中修改并应用；电机方向、零偏、每轴速度上限及映射矩阵可以编辑该 JSON。Demo 数据存放在：

```text
~/.config/muweidu_arm_sdk_ui/demo_positions.json
```

hardware 包 `CMakeLists.txt` 将 `SDK_UI` 安装到 `share/arm_hardware/SDK_UI`，并独立提供 pip editable 安装/运行入口。

## 3. 文件清单

| 文件 | 用途 |
|---|---|
| `SDK_UI/pyproject.toml` | 独立 Python 包元数据和命令行入口 |
| `SDK_UI/sdk_ui/default_hardware.json` | CAN、ID、方向、零偏、速度限值和映射默认值 |
| `SDK_UI/sdk_ui/protocol.py` | SocketCAN 传输和单电机协议 SDK |
| `SDK_UI/sdk_ui/arm_sdk.py` | 六轴变换、位置读取、速度规划和多电机命令 |
| `SDK_UI/sdk_ui/app.py` | 实体硬件调试 GUI、Demo 保存和状态显示 |
| `SDK_UI/README.md` | 安装、运行和操作说明 |
| `SDK_UI/IMPLEMENTATION_STEPS.md` | 本步骤详细设计说明 |
| `src/arm_hardware/src/arm_hardware_interface.cpp` | 将 MoveIt Joint1 比例同步改为 2.8 |
| `src/arm_hardware/CMakeLists.txt` | 安装 SDK_UI 文件 |

## 4. 约束和实机前检查

1. 默认 Motor ID 2…7 是占位示例，需填入实际唯一 ID。
2. 默认电机速度上限由此前设备表“目标电机最大转速”换算为度/秒：rpm × 6，约为 `[123,139,139,148,503,430]`。这是待核对初值，需确认 0xA4 速度字段是否按电机编码器侧度/秒解释、以及减速比在 0x92 角度中的位置。
3. Joint1 绳驱比当前明确为 2.8。若实机最终使用其他数值，需同时更新 SDK 的 JSON 矩阵和 hardware 插件矩阵。
4. 增量值与绝对目标都以关节角度度输入；所有命令直接操作实体硬件，不做 MoveIt 碰撞检查。
5. 按工作区约束，本次未运行构建或测试；此说明记录代码实现和待实际 CAN/电机验证的参数。
