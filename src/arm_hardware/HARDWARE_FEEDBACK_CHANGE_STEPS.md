# 硬件反馈基础修改记录

这份文档记录六轴耦合实现使用的硬件反馈基础。后续的硬件映射、MoveIt 和 ros2_control 配套改动见 [COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md](COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md)。

## 第 1 步：新增 0x92 多圈角度解析

修改 `include/muweidu_ts/motor.hpp` 和 `src/motor.cpp`，新增 `Motor::readMultiTurnAngleCentiDegrees()`。该方法发送 0x92 读取多圈角度，将数据字段按有符号 56 位数解码，返回 0.01°单位的厘度值。

## 第 2 步：以多圈反馈更新位置

修改 `include/arm_hardware/arm_hardware_interface.hpp` 和 `src/arm_hardware_interface.cpp`。硬件激活时和控制周期中都读取 0x92，并将厘度换算为弧度。直连模式再按每个关节的 `direction`、`zero_position_rad` 修正状态；耦合模式按每个电机的 `motorN_direction`、`motorN_zero_offset_rad` 修正电机反馈，再用耦合矩阵的逆变换得到关节位置。

此前通过 0x94 单圈角度增量估算多圈位置的状态不再需要。直连模式保留的软件零位参数为 `zero_position_rad`。

机械臂耦合配置当前还会在激活时默认发送一次六电机原始位置目标 0。若电机已使能，这可能引起运动；可通过 `initialize_motors_to_zero` 参数关闭。此命令不是写编码器零位或机械臂回零流程。

## 第 3 步：读取速度和 Iq 原始反馈

0x9C 状态帧中的转速和 Iq 字段按有符号 16 位解析。速度从 °/s 换算为 rad/s；耦合模式下将六个电机速度用同一个逆矩阵换算为六个关节速度。Iq 保持原始值，在耦合模式中按位置矩阵的转置形成广义关节反馈。

协议资料没有提供 Iq 到 N·m 的电机常数，所以 `effort` 状态目前不能作为物理单位 N·m。机械臂 MoveIt 位置规划和 `JointTrajectoryController` 使用 position/velocity 状态，不依赖该原始 effort。

## 第 4 步：配置接口

ROS 2 control 硬件描述声明了 `velocity` 或 `effort` 状态接口时，插件才导出并读取相应反馈。当前机械臂的六轴描述已经声明 position、velocity、effort 状态接口；位置滑块仍采用直连模式。

具体六轴映射公式、硬件参数、MoveIt 执行链路、速度约束和实机标定步骤见 [COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md](COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md)。
