# MoveIt 轨迹速度映射与六轴独立限速

## 命令链路

原实现只声明六个 `position` 命令接口。硬件用同一个 `joint_max_speed_dps` 计算同步速度，MoveIt 轨迹速度没有进入硬件的 A4 限速计算。

1. MoveIt 将关节轨迹交给 `arm_R_controller` 的 `FollowJointTrajectory` action。 [MoveIt Humble 控制器句柄](https://github.com/moveit/moveit2/blob/humble/moveit_plugins/moveit_simple_controller_manager/src/follow_joint_trajectory_controller_handle.cpp#L53-L59)
2. 六轴 `JointTrajectoryController` 同时声明 `position`、`velocity` 命令接口。在 Humble 的这组接口组合下，控制器每周期写入期望关节位置和插值后的期望关节速度。位置和速度分别以 rad 和 rad/s 传递。默认 spline 插值也能从只含位置的相邻轨迹点计算速度。 [控制器配置](../arm_moveit_config/config/ros2_controllers.yaml) · [JTC Humble 写入实现](https://github.com/ros-controls/ros2_controllers/blob/humble/joint_trajectory_controller/src/joint_trajectory_controller.cpp#L270-L299) · [轨迹插值](https://github.com/ros-controls/ros2_controllers/blob/humble/joint_trajectory_controller/src/trajectory.cpp#L186-L253)
3. 轨迹速度和每轴上限先换算成关节侧本周期限速，再结合本次六轴位置目标、实时电机反馈和传动耦合计算 A4 电机侧速度。 [硬件接口](src/arm_hardware_interface.cpp) · [速度计算](include/arm_hardware/synchronized_speed.hpp)

## 参数和公式

对于关节 $i$，取 JTC 速度的绝对值并由 rad/s 换算成 °/s，再计算

$$
x_i=|v_i|\frac{180}{\pi},\qquad
y_i=\min\bigl(v_{\max,i},\,k x_i+b\bigr).
$$

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `joint_speed_limits_dps` | J1 到 J6 各自的关节侧硬上限，单位 °/s | `10,10,10,10,10,10` |
| `joint_speed_scale` | 共享斜率 $k$，要求 $0<k\le1$ | `0.8` |
| `joint_speed_offset_dps` | 共享偏置 $b$，单位 °/s，要求 $0<b\le\min_i v_{\max,i}$ | `2.0` |

六个上限分别配置。初始值沿用此前 10°/s 的硬件上限。$k<1$ 压缩速度差，$b$ 给零速度采样保留正的限速，供有剩余位置误差时继续逼近目标。默认下 $x=0,5,10$ °/s 分别映射为 $y=2,6,10$ °/s。负速度按同等幅值计算，运动方向由 A4 绝对位置目标决定。非有限命令、非有限系数、非法参数或缺失的部分速度接口会报告错误。

## 六个关节限速到六个电机字段

位置目标仍经过原有耦合映射。以 M2、M3 为例，$m_2=36q_2-43.2q_3$、$m_3=-36q_2-43.2q_3$。M5 也同时受 J4、J5 影响。驱动以本次反馈与量化后目标的差值求一个共同运动时长。令 $\Delta q_i$ 和 $\Delta m_j$ 均以度计，

$$
T=\max\left(\max_i\frac{|\Delta q_i|}{y_i},\ 
             \max_j\frac{|\Delta m_j|}{65535}\right).
$$

各电机的 A4 速度字段按 $|\Delta m_j|/T$ 向下量化，范围为 1 到 65535 电机 °/s。零位移电机也使用字段值 1。若某个非零电机速度低于 1°/s，该电机只能按最小字段发送，可能早于公共时长到达。计算的关节限速是按同步位移比例得到的名义值，实机响应还受串行发送、固件和绳驱变形影响。

例如 J2、J3 需分别移动 10°、5°，本周期关节侧限速分别为 5°/s、8°/s，且其余轴静止，则共同时间为 2 s。M2、M3 的位移分别为 144°、−576°，速度字段分别为 72、288。

## 项目修改

| 文件 | 修改 |
| --- | --- |
| [ros2_controllers.yaml](../arm_moveit_config/config/ros2_controllers.yaml) | JTC 同时请求六轴位置和速度命令接口。 |
| [arm_R.ros2_control.xacro](../arm_moveit_config/config/arm_R.ros2_control.xacro) | 为六个耦合关节声明 `velocity` 命令接口，并把三项速度映射参数交给硬件插件。 |
| [arm_R.urdf.xacro](../arm_moveit_config/config/arm_R.urdf.xacro) 和 [整机启动文件](../arm_bringup/launch/arm_moveit_start.launch.xml) | 暴露六轴上限、斜率和偏置的参数及默认值。 |
| [arm_hardware_interface.hpp](include/arm_hardware/arm_hardware_interface.hpp) 和 [arm_hardware_interface.cpp](src/arm_hardware_interface.cpp) | 导出速度命令，跟踪接口认领状态，在写入前按关节速度计算限速。保留电机零偏、方向、A4 位置量化和静态绳驱补偿。 |
| [synchronized_speed.hpp](include/arm_hardware/synchronized_speed.hpp) | 实现六轴速度映射、独立限速和 A4 速度量化。 |
| [test_synchronized_speed.cpp](test/test_synchronized_speed.cpp) | 校验正负速度、零速度偏置、每轴封顶、差动耦合、协议边界与非法输入。 |

硬件初始化时以实际反馈填充位置命令，速度命令置零。只有六轴速度接口都被控制器认领时才使用轨迹速度。控制器切换时清空速度命令和上次速度缓存。位置变化时发送一组完整的六电机 A4 指令。位置目标相同时，只有已下发的目标还有电机位置残差、且映射限速和最终 A4 速度字段发生变化时才更新速度。单独的速度采样不会使能空闲电机。A4 写入失败沿用整组急停处理。

旧式仅位置的六轴描述可使用固定的六轴 `joint_speed_limits_dps`，也可保留旧标量 `joint_max_speed_dps` 作为兼容项。整机 MoveIt 新配置使用速度接口和新参数。单电机模式继续使用其原有电机侧 `max_speed_dps`。独立 SDK_UI 走自己的 SocketCAN 链路。

## 配置与验证

从 ROS 2 Humble 工作区重新构建并加载环境后，按 J1 到 J6 的顺序传入六个上限。以下参数是配置示例。

```bash
colcon build --symlink-install --packages-up-to arm_bringup
source install/setup.bash
ros2 launch arm_bringup arm_moveit_start.launch.xml \
  joint_speed_limits_dps:=8,6,5,7,9,4 \
  joint_speed_scale:=0.8 \
  joint_speed_offset_dps:=2.0
```

MoveIt 的规划速度取自 [joint_limits.yaml](../arm_moveit_config/config/joint_limits.yaml)。当前该文件各轴 `max_velocity` 为 0.524533 rad/s，约 30.05°/s，和本次硬件侧默认上限 10°/s 分别生效。当轨迹速度大于硬件映射后的限速时，实际跟随时间可能超过规划时间。调试时可查看 `/arm_R_controller/controller_state` 的期望和实测速度，对照运行时的跟随误差。

启动后可用 `ros2 control list_hardware_interfaces` 核对 `Joint1/velocity` 至 `Joint6/velocity` 是否均由控制器认领。

可在构建环境中运行纯计算回归。

```bash
colcon test --packages-select arm_hardware --ctest-args -R '^synchronized_speed$'
colcon test-result --verbose
```

本次在 Windows 原生编译并运行了速度计算回归。独立展开 Xacro 验证了默认值、不同的六轴上限和启用绳驱补偿三种配置，生成的六轴均含位置与速度命令接口。84 项可移植 SDK 测试通过。完整 ROS 2 Humble 构建、控制器认领及实际 A4 下发仍需在目标环境验证。
