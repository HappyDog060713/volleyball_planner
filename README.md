# volleyball_planner

排球机器人拦截运动规划与击打时序中枢功能包 (ROS 2)。

本功能包作为**视觉追踪系统（volleyball_tracker）**与**底盘及Stewart并联机构控制工程（volleyball_controller）**之间的核心纽带。

## 功能特性
- **底盘拦截运动规划**：根据排球下落落点与到达时间预测，规划全向底盘拦截运动速度，发布 `chassis_controllers/msg/ChassisControl` 到 `/chassis_command`。在 Stewart 平台爆发顶升瞬间自动抱闸底盘以保证发射刚性。
- **Stewart 并联机构反弹法向与 Z 轴反击顶升**：
  - 拦截预备期间，平台下沉至蓄力高度 $Z=0.20\text{ m}$，并基于期望出球速度向量与入射速度向量解算镜面反弹法向倾角姿态 $(roll, pitch)$；
  - 触发击打时，平台以最大速度向上顶升冲顶至 $Z=0.36\text{ m}$ 进行反击撞击（耗时约 150ms），持续 0.20s 后回落复位。
- **击打时序协同**：针对 Stewart 平台机械顶升时延（`strike_delay_time = 0.15s`），在底盘到位且剩余落点时间进入窗口时触发爆发顶击。

## 通信接口
- **订阅话题**:
  - `/volleyball/trajectory_pred` (`volleyball_tracker/msg/TrajectoryPrediction`): EKF 滤波轨迹与落点到达时间。
- **发布话题**:
  - `/chassis_command` (`chassis_controllers/msg/ChassisControl`): 底盘全向运动控制。
  - `/stewart_command` (`volleyball_controller/msg/StewartControl`): Stewart 机构 6-DOF 姿态与高动态 $Z$ 轴顶升击打。
  - `/volleyball/strike_trigger` (`std_msgs/msg/Bool`): 击打触发通知。

## 编译与启动
```bash
source /path/to/26_RC_Volleyball_2/install/setup.bash
colcon build --packages-select volleyball_planner
source install/setup.bash
ros2 launch volleyball_planner planner.launch.py
```
