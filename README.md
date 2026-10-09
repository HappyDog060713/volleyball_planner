# volleyball_planner

排球机器人拦截运动规划与击打时序中枢功能包 (ROS 2)。

本功能包作为**视觉追踪系统（volleyball_tracker）**与**底盘及Stewart并联机构控制工程（volleyball_controller）**之间的核心纽带。

## 功能特性
- **底盘拦截运动规划**：根据排球下落落点与到达时间预测，规划全向底盘拦截运动速度，发布 `chassis_controllers/msg/ChassisControl` 到 `/chassis_command`。
- **Stewart 并联机构反弹法向定向**：基于期望出球速度向量与入射速度向量，根据镜面弹性碰撞反弹原理解算 Stewart 靶面所需的倾角姿态 $(roll, pitch)$，发布 `volleyball_controller/msg/StewartControl` 到 `/stewart_command`。
- **击打时序协同**：针对机械固有击打延迟（`strike_delay_time`），在最佳击球窗口提前向 `volleyball_hub` 发送击打触发信号 `/volleyball/strike_trigger`。

## 通信接口
- **订阅话题**:
  - `/volleyball/position_filtered` (`geometry_msgs/msg/PointStamped`): 滤波后排球位姿。
  - `/volleyball/landing_prediction` (`geometry_msgs/msg/PointStamped`): 预测落点。
- **发布话题**:
  - `/chassis_command` (`chassis_controllers/msg/ChassisControl`): 底盘全向运动控制。
  - `/stewart_command` (`volleyball_controller/msg/StewartControl`): Stewart 机构 6-DOF 姿态。
  - `/volleyball/strike_trigger` (`std_msgs/msg/Bool`): 击打触发通知。

## 编译与启动
```bash
source /path/to/26_RC_Volleyball_2/install/setup.bash
colcon build --packages-select volleyball_planner
source install/setup.bash
ros2 launch volleyball_planner planner.launch.py
```
