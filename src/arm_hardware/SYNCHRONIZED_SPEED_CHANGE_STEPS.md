# 六关节统一速度上限：逐步修改说明

本文记录本次修改的目的、每个文件的修改位置、修改前后的行为、计算过程和验证方法。以下路径均相对于 ROS 2 工作空间根目录 `ros2_ws`。

## 1. 明确修改目标和适用范围

机械臂共有六个逻辑关节 `Joint1`～`Joint6`，使用 `mapping_mode=coupled`。以前六个电机的 0xA4 位置命令都填写同一个**电机侧**速度 `max_speed_dps`，并不根据每台电机要移动的距离来分配速度。

现在提供一个统一的**关节侧**速度上限 `joint_max_speed_dps`。每次收到新的六关节位置目标时，程序使用最近一次成功读取的六台电机位置，计算从当前位置到目标位置的增量，然后按电机增量计算六个速度。

位置单位在 ROS 接口和内部映射中为弧度；配置的速度单位是关节 **°/s**；0xA4 发送的速度单位是电机 **°/s**。单电机 `direct` 模式、滑块和独立 SDK/UI 没有改用这个六轴参数。

## 2. 修改六轴的统一速度配置

### 2.1 在机械臂 xacro 中声明参数

文件：`src/arm_moveit_config/config/arm_R.urdf.xacro`。

原来声明的是电机侧参数：

```xml
<xacro:arg name="max_speed_dps" default="1200" />
```

现在改成关节侧参数：

```xml
<xacro:arg name="joint_max_speed_dps" default="10" />
```

同一文件调用 `arm_R_ros2_control` 宏时，也改为传递新参数：

```xml
joint_max_speed_dps="$(arg joint_max_speed_dps)"
```

`10` 是新的初始配置，代表所有关节共用的 10°/s；它不是由旧值 1200 自动换算得到的。若希望设置为 100°/s，可以将 `default="10"` 改为 `default="100"`。支持有限的正小数。

### 2.2 将参数传给硬件插件

文件：`src/arm_moveit_config/config/arm_R.ros2_control.xacro`。

在 `arm_R_ros2_control` 宏的参数列表中使用 `joint_max_speed_dps`，在 `<hardware>` 内传递：

```xml
<param name="joint_max_speed_dps">${joint_max_speed_dps}</param>
```

参数路径是：`arm_R.urdf.xacro` 的默认值或 xacro 参数 → `arm_R_ros2_control` 宏 → ros2_control 的硬件参数 → `ArmHardwareInterface::on_init()`。

当前启动文件直接运行 xacro，通常修改默认值后需要重新构建并加载配置包；单独展开 xacro 时也可传入 `joint_max_speed_dps:=100`。

## 3. 修改硬件类中保存速度和反馈的成员

文件：`src/arm_hardware/include/arm_hardware/arm_hardware_interface.hpp`。

1. 原来耦合模式用 `coupled_max_speed_dps_` 保存一个电机侧整数速度，并复制到每个 `MotorAxis`。
2. 现在用 `double joint_max_speed_dps_{10.0}` 保存关节侧统一速度，支持小数。
3. 删除耦合电机结构中原先固定的 `max_speed_dps`；正常位置命令和启动归零时均按当次位移计算速度。
4. 增加 `bool coupled_feedback_valid_{false}`，记录是否有一组完整成功的六电机反馈。直连模式 `Joint` 内的电机侧 `max_speed_dps` 保留。

## 4. 修改硬件初始化时的参数读取

文件：`src/arm_hardware/src/arm_hardware_interface.cpp`，函数 `ArmHardwareInterface::on_init()`。

按以下顺序处理六轴耦合模式的速度参数：

1. 检查硬件参数是否还包含旧的耦合参数 `max_speed_dps`；如果包含，返回初始化错误，日志提示改用 `joint_max_speed_dps`，防止将原电机侧数值误当成关节侧数值。
2. 读取 `joint_max_speed_dps`；若硬件描述未提供则使用类内默认值 10。
3. 通过 `parsePositionParameter()` 检查是否为有限数，再检查数值大于零；无效值使初始化失败。
4. 继续读取六台电机的 CAN ID、方向和零偏，保留原有校验。

直连模式仍按原来的方式从每个关节读取 `max_speed_dps`，范围为 1～65535 电机 °/s。

## 5. 在激活时读取六电机当前位置

文件：`src/arm_hardware/src/arm_hardware_interface.cpp`，函数 `on_activate()`。

1. 按六个电机顺序发送 0x92 多圈角度读取命令。
2. 将读取值从 0.01° 换算为弧度：`raw_angle_rad = angle_centi_degrees × π / 18000`。
3. 根据各电机配置转换到映射空间：`mapped_angle = direction × (raw_angle_rad - zero_offset_rad)`。
4. 六台全部读取成功后，将这组电机位置通过 `motorToJointAngles()` 反算为六个当前关节位置，并初始化关节状态和命令位置。
5. 标记 `coupled_feedback_valid_ = true`。如果激活过程失败则清除标志并返回错误，避免把不完整的位置当作六轴当前位置。

反馈快照来自逐台顺序读取，因此六个数值不是同一瞬间的硬件采样。

## 6. 修改周期性反馈读取，避免混用数据

文件：`src/arm_hardware/src/arm_hardware_interface.cpp`，函数 `read()` 的 `coupled` 分支。

1. 开始读取时先将 `coupled_feedback_valid_` 设为 `false`。
2. 六台电机的 0x92 角度先保存到局部数组 `motor_angles`；如果配置需要速度或力矩状态，则另外读取 0x9C 状态并暂存到局部数组。
3. 只有六台全部读取成功，才把局部数组统一写入 `motors_` 状态，再将 `coupled_feedback_valid_` 设为 `true`，并更新关节状态。
4. 如果任何一台读取失败，`read()` 返回错误；在插件已使能电机或已下发启动归零的情况下，尝试对全部六台发送急停。
5. 后续 `writeCoupledMotors()` 会检查反馈有效标志，避免用失效的反馈计算新的速度。

这种处理保证一次失败的读取不会把部分新位置与部分旧位置组合使用。

## 7. 保留原有六轴位置映射

文件：`src/arm_hardware/src/arm_hardware_interface.cpp`，函数 `jointToMotorAngles()` 和 `motorToJointAngles()`。

此次没有改变机械映射。完整关节位置向量 `q` 转成电机位置向量 `m` 的关系为：

```text
m1 = -36 × 2.8 × q1
m2 =  36 × (q2 - 1.2 × q3)
m3 = -36 × (q2 + 1.2 × q3)
m4 = -36 × 1.8 × q4
m5 =  10 × (1.3 × q5 + 1.6 × q4)
m6 =  10 × 1.5 × q6
```

关节 2 和 3 同时影响电机 2、3；关节 4 和 5 也共同影响电机 5。因此先对完整六关节向量做映射，再使用实际电机增量计算速度。电机方向和零偏在映射与原始 CAN 位置之间另行处理。

## 8. 新增可独立测试的速度计算函数

新文件：`src/arm_hardware/include/arm_hardware/synchronized_speed.hpp`。

新增函数：

```cpp
synchronizedMotorSpeeds(
  current_joint_rad, target_joint_rad,
  current_motor_rad, target_motor_rad,
  joint_speed_limit_dps)
```

前四项都是包含六个弧度值的数组；最后一项是统一关节速度上限，单位 °/s。返回六个 `uint16_t` 电机速度，单位也是 °/s。

函数依次进行：

1. 检查速度上限有限且大于零，四组位置都必须是有限数。
2. 计算六个关节位置差 `Δq` 和六个电机位置差 `Δm`，并转换成角度。
3. 根据所有关节位移，计算达到统一关节速度上限所需的最短理论时间。
4. 根据所有电机位移，计算速度字段不超过 65535°/s 所需的最短时间。
5. 取两个时间中的较大值 `T`，再用 `|Δm_i| / T` 算每个电机的理论速度。
6. 将理论速度编码为整数，并处理零位移、太小的速度和非法值。

计算式是：

```text
T_joint    = max_i(|Δq_i|) / joint_max_speed_dps
T_protocol = max_i(|Δm_i|) / 65535
T          = max(T_joint, T_protocol)
v_motor_i  = |Δm_i| / T
```

所有电机共用同一个 `T`，所以编码前的速度与各自电机位移成比例。65535 是协议字段上限，并非电机额定转速。

## 9. 示例：只运动关节 1

假设统一关节速度为 `100°/s`，当前位置 `q1=0°`，本次目标 `q1=10°`，其余关节不变，并假设电机目标无需额外位置舍入：

```text
Δq1 = 10°
Δm1 = -36 × 2.8 × 10° = -1008°
T_joint = 10° / 100°/s = 0.1 s
T_protocol = 1008° / 65535°/s ≈ 0.0154 s
T = 0.1 s
电机 1 理论速度 = 1008° / 0.1 s = 10080°/s
```

速度字段只表示大小，方向由当前位置和目标位置的关系确定。此例的 10080 即 `100 × 36 × 2.8`。其他电机没有位移，其目标保持当前位置并填写速度字段 1。

## 10. 示例：关节 2、3 同时运动

假设当前位置为零，本次目标为 `q2=10°`、`q3=5°`，其余关节不变，统一速度上限为 `10°/s`：

```text
Δm2 =  36 × (10 - 1.2 × 5) =  144°
Δm3 = -36 × (10 + 1.2 × 5) = -576°
T_joint = max(10°, 5°) / 10°/s = 1 s
电机 2 理论速度 = 144°/s
电机 3 理论速度 = 576°/s
```

两台电机的理论速度之比为 `1:4`，与电机位移绝对值之比一致。此例也说明关节 2、3 的耦合必须参与计算。

## 11. 修改正常位置输出

文件：`src/arm_hardware/src/arm_hardware_interface.cpp`，函数 `writeCoupledMotors()`。

原来位置目标经过映射后，六条命令均使用固定的 `motor.max_speed_dps`。现在依次进行：

1. 判断是否存在新的关节位置命令；非有限命令也会进入校验路径并报错。
2. 检查 `coupled_feedback_valid_`，没有有效六轴反馈则拒绝写入。
3. 检查六个关节目标都是有限数，并通过 `jointToMotorAngles()` 计算六个映射空间电机目标。
4. 从最近一次成功读取的 `motors_[i].state_position_rad` 收集当前电机位置，通过 `motorToJointAngles()` 求出当前关节位置。
5. 将每个目标按方向和零偏转换成原始电机角度，并按 0xA4 的 0.01° 分辨率四舍五入；检查目标处于有符号 32 位范围。
6. 将编码后的原始目标重新转换到映射空间，得到 `quantized_motor_angles`，并反算 `quantized_joint_angles`。这样速度计算使用的是实际将要发送的编码目标。
7. 调用 `synchronizedMotorSpeeds()` 计算六个速度。
8. 根据配置在需要时使能电机，再向每台电机发送各自的速度和对应位置目标。
9. 六条命令都成功后，更新最近发送的关节命令记录。计算、使能或发送发生异常时返回错误，并在相关条件满足时尝试对全部电机急停。

使用的是控制器本次采样目标，而不是整条 MoveIt 轨迹的最终目标。控制器给出新的位置采样时会重新计算。

## 12. 修改启动时的电机归零速度

文件：`src/arm_hardware/src/arm_hardware_interface.cpp`，函数 `on_activate()` 的 `initialize_motors_to_zero` 分支。

原来启动归零直接对六台电机发送固定电机速度和原始目标 0。现在：

1. 使用激活时读到的六台电机当前位置。
2. 对每台电机，把原始绝对目标 0 按其方向和零偏转为映射空间目标：`zero_motor_angles[i] = -direction[i] × zero_offset_rad[i]`。
3. 将六个电机归零目标反算为关节目标。
4. 用当前位置和归零目标调用同一个 `synchronizedMotorSpeeds()`，得到六个电机速度。
5. 按各自速度向六台电机发送原始绝对位置目标 0。如果部分发送失败，尝试对全部六台电机急停。

`initialize_motors_to_zero` 默认是 `true`。这里的零是原始电机绝对位置目标，不是重新设定或保存编码器零点；如果电机已经使能，启动时可能移动。不需要此行为时，在 `arm_R.urdf.xacro` 将该参数设为 `false`。

## 13. 处理速度和位置精度

0xA4 目标位置字段的分辨率是 0.01°，速度是 16 位整数 °/s，因此不能准确表示所有比例：

- 电机位移为零：位置目标保持当前编码位置，速度字段填 1，避免依赖速度字段为 0 的固件语义。
- 理论速度至少为 1°/s：按整数向下量化；实现对整数边界的浮点误差留有极小容差。
- 理论速度大于零但小于 1°/s：拒绝本次命令并尝试急停，同时输出错误日志。位移差距大、反馈微小误差和连续轨迹采样都可能触发，因此某些轨迹无法执行。
- 某个电机理论速度会超过 65535°/s：增加所有电机共用的 `T`，而不是只截断这一台。

位置和速度量化会造成比例误差；经过耦合映射后的实际关节速度也可能与理论上限略有偏差。电机速度上限、加减速及机械负载仍需根据设备实际能力核实，65535 不能作为电机额定速度。

## 14. 添加计算测试和构建配置

新增测试文件：`src/arm_hardware/test/test_synchronized_speed.cpp`。

测试包括：

1. Joint1 单轴运动时 `100°/s → 10080°/s`。
2. 关节 2、3 耦合运动时电机 2、3 的速度比例。
3. 一个耦合电机的位移相互抵消，以及六台电机均无位移。
4. 超过协议速度上限时，通过共同延长时间保留各电机速度比例。
5. 小于协议最小非零速度、无效速度上限、NaN 和无穷大输入。

文件 `src/arm_hardware/CMakeLists.txt` 在 `BUILD_TESTING` 时构建并注册 `synchronized_speed` 测试，同时安装本文档。`src/arm_hardware/README.md` 更新了新参数的含义、默认电机 ID 和本文档链接。

## 15. 构建和验证过程

本次工作区的旧构建缓存引用了另一台机器上的路径，直接使用现有 `build`/`install` 时失败。因此没有删除原目录，而是在临时路径独立构建。实际执行的命令如下：

```bash
cd /home/xlh/桌面/ros2_ws_2/ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/arm_sync_log build \
  --build-base /tmp/arm_sync_build \
  --install-base /tmp/arm_sync_install \
  --packages-up-to arm_hardware arm_moveit_config
colcon --log-base /tmp/arm_sync_test_log test \
  --build-base /tmp/arm_sync_build \
  --install-base /tmp/arm_sync_install \
  --packages-select arm_hardware --event-handlers console_direct+
colcon test-result --test-result-base /tmp/arm_sync_build --verbose
```

结果：`arm_description`、`arm_hardware`、`arm_moveit_config` 构建成功；测试 1 项，0 失败。另用临时安装目录展开 xacro，传入 `joint_max_speed_dps:=100`，生成的硬件参数值为 `100`。

上述构建产物位于 `/tmp/arm_sync_install`，并没有覆盖工作区原来的 `install`。实际使用前需要在自己的有效构建环境中重新构建并加载相应安装目录。当前环境没有 CAN/vCAN 接口，本次没有连接真实电机验证。

## 16. 使用时需要了解的行为

共同时间和比例是针对**当前反馈到本次目标**计算的理论速度字段，不保证六轴实际同时到达。CAN 命令逐台发送，反馈逐台读取；电机加减速、负载、内部控制、误差及整数精度都可能改变实际运动。使用前需按机械臂和电机实际能力核实统一上限、各电机方向与零偏、启动归零和异常停止行为。
