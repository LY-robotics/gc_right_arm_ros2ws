# arm_hardware

慕纬度 TS CAN 电机的 ROS 2 Humble 硬件包。包内包含两种用法：`motor_slider` 是单电机滑块调试入口；机械臂使用 `arm_moveit_config` 中的六轴耦合配置，由 `ArmHardwareInterface` 将六个 URDF 关节位置映射到六个电机位置。底层通过 Linux SocketCAN 发送标准帧，可使用 `can0`、`can1`、`can2`、`can3`。

另有独立真实硬件调试 SDK/UI，位于 [`SDK_UI`](SDK_UI/README.md)。它使用 Python 标准库直接访问 SocketCAN，不依赖 ROS 或仿真。

## 构建

```bash
cd ~/桌面/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 配置 CAN

启动前确认转接器已创建 SocketCAN 接口，并设置与电机一致的波特率。已确认电机在 `can2`、ID 2；CAN 速率为 1 Mbps：

```bash
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up
ip -details link show can2
```

## 配置文件

- `motor_slider.ros2_control.xacro`：定义硬件插件、虚拟关节、电机 CAN 口、ID、默认速度和使能行为。当前默认值为 `can2`、ID 2、10°/s。
- `config/arm_hardware_controllers.yaml`：定义 controller manager 更新频率、关节状态广播器、位置控制器和滑块桥接节点参数。
- `../arm_moveit_config/config/arm_R.ros2_control.xacro`：声明机械臂的六个逻辑关节和六个物理电机的耦合映射参数。

硬件与电机参数在 `.ros2_control.xacro` 中修改；控制器和话题参数在 YAML 中修改。启动文件负责加载这两处配置。

机械臂耦合模式下，URDF/MoveIt/`JointTrajectoryController` 始终使用 `Joint1`…`Joint6`。硬件插件内部将每一组完整关节位置命令换算成 6 个电机目标，并用 0x92 多圈角度反馈反算回逻辑关节状态。当前映射由机械臂控制 xacro 的 `mapping_mode=coupled` 启用；滑块仍保留一关节直连一电机的 `direct` 模式。机械臂硬件激活时默认向每台电机发送一次原始多圈位置目标 0（可将 `arm_R.urdf.xacro` 中 `initialize_motors_to_zero` 改为 `false` 关闭）。如果电机此时已使能，该绝对位置命令可能使机械臂移动。

`arm_R.urdf.xacro` 的电机 ID 默认值 1…6 是示例值，需要按实际 CAN 总线上的六个电机 ID 修改。各电机方向和多圈角度零偏也需要按机械臂标定值调整。MoveIt 六轴耦合模式通过 `joint_speed_limits_dps` 设置 J1～J6 各自的关节侧上限，`joint_speed_scale` 和 `joint_speed_offset_dps` 将每轴轨迹速度映射为本次限速。驱动结合传动耦合和反馈计算六条 A4 的电机侧速度字段。单电机模式使用电机侧 `max_speed_dps`。 [修改记录与配置示例](MOVEIT_TRAJECTORY_SPEED_CHANGE_STEPS.md)

## 打开位置滑块

```bash
ros2 launch arm_hardware motor_slider.launch.xml
```

也可以运行 Python 包装启动文件：

```bash
ros2 launch arm_hardware motor_slider.launch.py
```

启动会打开 `joint_state_publisher_gui`。虚拟关节叫 `motor_joint`，滑块范围为 -90° 到 +90°。GUI 初始状态只作为基准；移动滑块后才会使能电机并发送 `0xA4` 多圈位置命令。关闭启动文件时，硬件插件会按配置发送急停。

位置反馈由 `0x92` 多圈角度读取，按有符号 56 位值和 0.01°/LSB 换算为弧度；不会再从 `0x94` 单圈值推算圈数。滑块直连模式的软件零位由关节参数 `zero_position_rad` 提供；六轴耦合模式则为每个电机单独设置 `motorN_zero_offset_rad`。`0xA4` 位置命令按协议规定的 0.01°/LSB 转换。

六轴耦合映射的 Joint1 绳驱比当前按最新给定值 2.8 处理，即 `m1=-36*2.8*q1`；其余五轴沿用此前给定的映射。`SDK_UI` 的矩阵和本硬件插件保持一致。

硬件插件支持 `position`、`velocity` 和 `effort` 状态接口。速度由 0x9C 的有符号转速（°/s）换算为 rad/s；耦合模式对速度使用位置矩阵的逆矩阵。`effort` 使用 0x9C 的有符号 Iq 原始值，并按耦合矩阵的转置映射到逻辑关节。协议资料没有提供 Iq 到 N·m 的换算常数，因此 effort 目前是广义原始反馈值，不是物理单位 N·m。只有硬件描述声明的状态接口才会导出；机械臂控制 xacro 已声明三种状态接口，MoveIt 的轨迹控制器只使用 position command 和 position/velocity state。

硬件反馈和六轴耦合修改的逐步说明见 [COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md](COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md)。此前单独记录 0x92 反馈阶段的 [HARDWARE_FEEDBACK_CHANGE_STEPS.md](HARDWARE_FEEDBACK_CHANGE_STEPS.md) 是早期阶段记录，现已由新文档补充完整。

## 命令行单次发送

也可以不打开滑块，直接启动一次位置命令：

```bash
ros2 launch arm_hardware motor_position.launch.py \
  target_degrees:=目标角度 \
  max_speed_dps:=电机允许的速度
```

将两个参数替换为数字。目标是多圈绝对角度，单位为度；该入口默认使用 `can2`、ID 2。
