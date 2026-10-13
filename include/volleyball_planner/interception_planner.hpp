#pragma once

#include <Eigen/Dense>
#include "chassis_controllers/msg/chassis_control.hpp"
#include "volleyball_controller/msg/stewart_control.hpp"

namespace volleyball_planner {

struct PlannerConfig {
    double max_chassis_speed{2.0};         // m/s
    double chassis_kp{2.2};
    double deadband_dist{0.03};            // 3cm
    
    // Stewart 6-DOF 并联机构反击击打参数 (反击由 Stewart 平台上升顶击进行)
    double stewart_ready_z{0.20};          // 拦截预备位高度 (m, 蓄力状态)
    double stewart_strike_z{0.36};         // 顶升击打爆发高度 (m, 击球最高点)
    double default_height{0.22816};        // 机构标称复位高度 (m)
    double pitch_range{0.35};              // rad (~20 deg)
    double roll_range{0.35};               // rad
    
    // Stewart 平台上升击打时延与触发阈值 (机械顶升仅需约 120~180ms)
    double strike_delay_time{0.15};        // 平台从 0.20m 上升至 0.36m 的机械达峰时间 (s)
    double strike_duration_s{0.20};        // 击打保持时间 (s)
    double strike_distance_thresh{0.25};   // 底盘对位容差 (m)
    
    Eigen::Vector3d target_outgoing_vel{0.0, 7.0, 3.5}; // 期望回击出射速度向量
};

class InterceptionPlanner {
public:
    explicit InterceptionPlanner(const PlannerConfig& config = PlannerConfig());
    ~InterceptionPlanner() = default;

    void setConfig(const PlannerConfig& config);
    const PlannerConfig& getConfig() const { return config_; }

    // 1. Chassis movement command toward predicted interception point
    chassis_controllers::msg::ChassisControl planChassisMovement(
        const Eigen::Vector2d& rel_target_pos_in_base,
        double time_to_arrive);

    // 2. Stewart orientation and Z-strike pop-up pose for counter-attack
    volleyball_controller::msg::StewartControl planStewartPose(
        const Eigen::Vector3d& incoming_vel_in_base,
        bool is_striking_now);

    // 3. Counter-strike trigger timing verification (Stewart upward pop-up)
    bool evaluateStrikeTrigger(
        double time_to_intercept,
        double distance_to_target);

private:
    PlannerConfig config_;
};

} // namespace volleyball_planner
