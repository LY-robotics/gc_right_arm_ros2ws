# 六轴电机耦合与 MoveIt 定点执行修改说明

## 1. 修改目标和边界

机械臂在 URDF 中有六个逻辑关节 `Joint1`…`Joint6`，但六个实体电机经过减速器和耦合机构，不是一关节一电机的直连关系。本次保留 URDF、MoveIt 和轨迹控制器中的六个关节名字，在 `ArmHardwareInterface` 中实现关节空间与电机空间的换算。MoveIt 仍然规划关节角或末端位姿；硬件插件负责将轨迹控制器下发的六关节位置转成六个电机目标。

位置命令使用用户给出的映射，角度内部统一使用弧度：

```text
m1 = -36 * 2.8 * q1
m2 =  36 * (q2 - 1.2 * q3)
m3 = -36 * (q2 + 1.2 * q3)
m4 = -36 * 1.8 * q4
m5 =  10 * (1.3 * q5 + 1.6 * q4)
m6 =  10 * 1.5 * q6
```

其中 `q1`…`q6` 是 URDF 逻辑关节角，`m1`…`m6` 是电机侧角。写入时从 `q` 计算 `m`；读回时先将编码器角度修正方向和零偏，再从 `m` 反算 `q`。映射矩阵为：

```text
      q1    q2     q3    q4    q5   q6
m1  [-100.8, 0,     0,    0,    0,   0]
m2  [   0,  36, -43.2,    0,    0,   0]
m3  [   0, -36, -43.2,    0,    0,   0]
m4  [   0,   0,     0, -64.8,   0,   0]
m5  [   0,   0,     0,   16,   13,   0]
m6  [   0,   0,     0,    0,    0,  15]
```

对应的逆变换为：

```text
q1 = -m1 / 100.8
q2 = (m2 - m3) / 72
q3 = -(m2 + m3) / 86.4
q4 = -m4 / 64.8
q5 = (m5 - 16*q4) / 13
q6 = m6 / 15
```

## 2. 分步骤实现

### 第 1 步：支持可靠的电机多圈位置反馈

修改 `src/arm_hardware/include/muweidu_ts/motor.hpp` 和 `src/arm_hardware/src/motor.cpp`：

- 新增 `readMultiTurnAngleCentiDegrees()`，发送 0x92 并解析有符号 56 位多圈角度；单位是厘度，即 0.01°/bit。
- 0x9C 状态帧的 Iq 和转速按有符号 16 位字段解析，以保留双向反馈。
- 转速按协议的 °/s 处理。Iq 保留原始比例值；协议资料没有给出从 Iq 换算 N·m 的电机常数。

机械臂激活和周期反馈均通过 0x92 获得多圈位置，不通过 0x94 单圈角度累计圈数。

### 第 2 步：在硬件插件中加入两种映射模式

修改 `src/arm_hardware/include/arm_hardware/arm_hardware_interface.hpp` 和 `src/arm_hardware/src/arm_hardware_interface.cpp`：

- 保留 `direct` 模式，供单电机滑块调试配置使用。
- 增加 `coupled` 模式，在一个 hardware component 下创建六个电机对象，并只对外暴露六个逻辑关节的接口。
- 耦合模式要求关节顺序严格为 `Joint1`…`Joint6`，要求六个电机 ID 唯一，并从硬件参数读取 CAN 口、方向、零偏和速度上限。
- 方向修正使用 `m_state = direction * (raw_angle - zero_offset)`；命令侧使用 `raw_target = direction * m_target + zero_offset`。`direction` 只能是 `+1` 或 `-1`。
- 每次六关节位置命令变化时，使用同一组 `q` 计算完整的六电机目标，并逐个发送 0xA4 多圈位置命令。不会对差动电机分别套用某个单关节角。
- 激活耦合硬件时，默认向六个电机各发送一次 0xA4 绝对多圈位置目标 `0`。这是电机编码器坐标的原始零目标，不会调用编码器置零命令，也不会写入电机非易失存储。参数 `initialize_motors_to_zero` 可关闭这项动作。
- 位置反馈使用逆矩阵还原 `q`；速度反馈对 0x9C 的六个电机速度应用同一逆矩阵。
- effort 状态按位置矩阵的转置将六个电机 Iq 原始量映射到广义关节反馈。其量纲仍是原始 Iq 派生值，不能称为 N·m，也不用于 MoveIt 的位置规划和轨迹控制。

六个电机目标通过 CAN 顺序发送，CAN 协议没有在本插件中实现六轴同步触发。因此电机目标更新之间可能存在很短的总线时差；如果设备固件有同步组命令，后续可以再换成同步发送。

### 第 3 步：让 ros2_control 描述与机械臂逻辑关节匹配

修改 `src/arm_moveit_config/config/arm_R.ros2_control.xacro`：

- `mapping_mode` 设为 `coupled`。
- 在 `<hardware>` 下配置共用 CAN 口、六个电机 ID、每个电机方向和多圈零偏、0xA4 电机侧速度上限。
- 保留 `Joint1`…`Joint6` 六个 `<joint>`，每个关节声明 `position` 命令接口以及 `position`、`velocity`、`effort` 状态接口。
- 插件实现电机耦合，URDF 不新增电机虚拟关节，也不使用 mimic 关节，避免 MoveIt 将电机轴当成额外自由度。

`src/arm_moveit_config/config/arm_R.urdf.xacro` 传入上述六个 ID、速度和初始化置零开关。ID 默认 2…7 只是唯一 ID 的示例，必须根据实际总线改成六个电机的地址；本仓库此前只验证过 ID 2，不能据此推断其余五个 ID。方向默认 `+1`、零偏默认 `0` 也只是占位标定值。若启动时不应发送绝对位置 0，将 `initialize_motors_to_zero` 默认值改成 `false`。

**启动动作影响：**如果电机在硬件激活时已使能，0xA4 目标 0 可能让电机运动到原始多圈编码器坐标 0。若电机未使能，固件是否保存并在之后执行该目标取决于电机固件；随后 JTC 下发的位置命令也可能覆盖它。该动作不是回零传感器流程，也不代表机械臂各关节的安全零位。上电前确认运动范围无人、急停可用；不希望启动时发零目标时关闭该参数。

### 第 4 步：保持 MoveIt 和 JointTrajectoryController 在关节空间工作

修改 `src/arm_moveit_config/config/ros2_controllers.yaml`，轨迹控制器仍接收六关节 `position` 命令，并读取 `position`/`velocity` 状态。MoveIt 的 `moveit_controllers.yaml` 已将 `arm_R_controller` 配置为包含这六个关节的 `FollowJointTrajectory` 控制器，因此不需要把六个电机暴露给 MoveIt。

执行链路：

```text
MoveIt 目标（末端位姿或六个 q 关节角）
        ↓ 规划 Joint1…Joint6 的 JointTrajectory
arm_R_controller / FollowJointTrajectory
        ↓ 六个逻辑 position 命令
ArmHardwareInterface（q→m 耦合映射）
        ↓ 六条 0xA4 多圈绝对位置命令
六个电机

六个电机 0x92/0x9C 反馈
        ↓ 方向、零偏修正及 m→q 逆映射
六个逻辑 position/velocity/effort 状态
        ↓
joint_state_broadcaster、MoveIt 当前状态
```

在 MoveIt RViz 中可选择机械臂规划组，设置末端位姿或六个关节目标，点击 Plan 检查路径，再点击 Execute 执行。规划组必须仍包含 `Joint1`…`Joint6`，状态来源应与 ros2_control 的 `joint_state_broadcaster` 一致。先在机械臂安全、低速条件下验证零点和方向，再执行远离当前位姿的目标。

### 第 5 步：速度参数与规划限速

`max_speed_dps` 是硬件插件放入每个 0xA4 命令的电机侧最大转速字段，单位为度/秒。它不是 ROS 关节速度，也不会把关节速度换算成某一个电机速度。关节的规划速度仍由 MoveIt 的 `joint_limits.yaml` 及请求中的 velocity scaling factor 决定。

Joint1 绳驱比在后续校正为 2.8，因此 `m1=-36*2.8*q1`，Joint1 的比例为 100.8。该 hardware 参数会影响 MoveIt 直连真实硬件的角度映射；SDK_UI 与此保持一致。

本配置暂将 `max_speed_dps` 默认设为 1200。这只是配置初值，不代表已由电机铭牌或固件验证为安全值。图片中的“目标电机转速”与含减速比位置公式之间，需要按电机 0x92/0xA4 的实际角度定义核对；在确认协议角度位于减速器前还是后之前，应降低 MoveIt velocity scaling，并在无负载、可急停的条件下逐轴验证。

由于电机速度是关节速度的线性组合，六个独立的 URDF 关节速度上限不能精确表达每个电机的组合速度约束。当前 `joint_limits.yaml` 的关节上限是规划器使用的每关节边界，不能替代电机空间的速度校核。已知电机侧安全速度后，建议用这些约束求保守的关节速度边界，或在轨迹限速层实现基于矩阵的组合限幅。

### 第 6 步：零点、方向和 ID 标定

修改 `arm_R.urdf.xacro` 或 `arm_R.ros2_control.xacro` 的硬件参数：

1. 将 `motor1_id`…`motor6_id` 填成总线扫描得到的实际唯一 ID。
2. 手动把机械臂置于 URDF 规定的零位，并记录每个电机的 0x92 多圈角度。
3. 将角度转换为弧度，填入对应的 `motorN_zero_offset_rad`。
4. 低速小范围单轴试动，核对正方向；若编码器符号与公式约定不同，将对应 `motorN_direction` 设为 `-1`。
5. 对 Joint2/Joint3 差动、Joint4/Joint5 耦合分别做小角度组合验证，确认反馈反解后的逻辑关节方向、比例正确。

方向修正和零偏按电机轴单独配置；不能把关节零位直接复制到六个关节的每个电机参数中。配置完成后，通过 `/joint_states` 对照实际关节角，并确保 MoveIt 显示的当前状态与真实机械臂一致，再开始定点规划执行。

## 3. 修改文件清单

| 文件 | 修改内容 |
|---|---|
| `src/arm_hardware/include/muweidu_ts/motor.hpp` | 多圈位置读取接口和有符号状态字段 |
| `src/arm_hardware/src/motor.cpp` | 0x92 多圈角度与 0x9C 有符号字段解析 |
| `src/arm_hardware/include/arm_hardware/arm_hardware_interface.hpp` | direct/coupled 数据结构、映射接口和六轴电机状态 |
| `src/arm_hardware/src/arm_hardware_interface.cpp` | 0x92 状态、六轴映射、命令发送、速度和 effort 反馈换算 |
| `src/arm_moveit_config/config/arm_R.ros2_control.xacro` | 六个逻辑关节与六个物理电机的耦合硬件参数 |
| `src/arm_moveit_config/config/arm_R.urdf.xacro` | 机械臂电机 ID、CAN 口、电机速度参数和激活时原始位置置零开关 |
| `src/arm_moveit_config/config/ros2_controllers.yaml` | 六关节轨迹控制器的状态接口配置 |
| `src/arm_hardware/README.md` | 两种硬件模式、耦合配置和单位说明 |
| `src/arm_hardware/CMakeLists.txt` | 安装本步骤说明文档 |

## 4. 当前实现的限制

- 本次只修改 hardware 和 MoveIt/ros2_control 描述，没有修改机器人几何模型、SRDF 规划组和关节位置范围。
- effort 反馈尚未标定为 N·m；MoveIt 位置规划和 JTC 不依赖该状态。
- 六轴命令顺序写入 CAN，无硬件同步触发。
- 默认激活动作向六个电机发送原始位置目标 0，若设备已使能可能发生运动；这不是寻零传感器或记录当前位置的操作。
- 电机 ID、方向、零偏和电机速度上限均需按实机核对。尤其是 0x92/0xA4 角度相对减速器的位置定义，需要先确认，再确定 1200°/s 的默认速度上限是否可用。
- 按仓库维护要求，本次没有执行构建或硬件联机验证；文档描述的是代码实现和待实机校准项。
