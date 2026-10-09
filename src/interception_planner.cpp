#include "volleyball_planner/interception_planner.hpp"
#include <algorithm>
#include <cmath>

namespace volleyball_planner {

InterceptionPlanner::InterceptionPlanner(const PlannerConfig& config)
    : config_(config) {}

void InterceptionPlanner::setConfig(const PlannerConfig& config) {
    config_ = config;
}

chassis_controllers::msg::ChassisControl InterceptionPlanner::planChassisMovement(
    const Eigen::Vector2d& rel_target_pos_in_base,
    double time_to_arrive)
{
    chassis_controllers::msg::ChassisControl cmd;
    cmd.emergency_stop = 0;
    cmd.power_limit = 0;
    cmd.spin_speed = 0.0f; // Lock heading during ball receiving

    double dist = rel_target_pos_in_base.norm();
    if (dist < config_.deadband_dist) {
        cmd.x_speed = 0.0f;
        cmd.y_speed = 0.0f;
        return cmd;
    }

    double vx = 0.0;
    double vy = 0.0;

    if (time_to_arrive > 0.1) {
        // Time-optimal velocity to arrive exactly on time
        vx = rel_target_pos_in_base.x() / time_to_arrive;
        vy = rel_target_pos_in_base.y() / time_to_arrive;
    } else {
        // Fallback proportional speed
        vx = config_.chassis_kp * rel_target_pos_in_base.x();
        vy = config_.chassis_kp * rel_target_pos_in_base.y();
    }

    // Clamp speed to maximum allowable chassis velocity
    double current_spd = std::hypot(vx, vy);
    if (current_spd > config_.max_chassis_speed) {
        double scale = config_.max_chassis_speed / current_spd;
        vx *= scale;
        vy *= scale;
    }

    cmd.x_speed = static_cast<float>(vx);
    cmd.y_speed = static_cast<float>(vy);
    return cmd;
}

volleyball_controller::msg::StewartControl InterceptionPlanner::planStewartPose(
    const Eigen::Vector3d& incoming_vel_in_base)
{
    volleyball_controller::msg::StewartControl cmd;
    cmd.x = 0.0f;
    cmd.y = 0.0f;
    cmd.z = static_cast<float>(config_.default_height);
    cmd.yaw = 0.0f;
    cmd.emergency_stop = 0;

    // Specular collision plane normal: n = (v_out - v_in) / ||v_out - v_in||
    Eigen::Vector3d delta_v = config_.target_outgoing_vel - incoming_vel_in_base;
    if (delta_v.norm() < 1e-3) {
        cmd.roll = 0.0f;
        cmd.pitch = 0.0f;
        return cmd;
    }

    Eigen::Vector3d n = delta_v.normalized();

    // Normal vector to RPY (pitch = -asin(nx), roll = atan2(ny, nz))
    double pitch = std::asin(-n.x());
    double roll = std::atan2(n.y(), n.z());

    cmd.pitch = static_cast<float>(std::clamp(pitch, -config_.pitch_range, config_.pitch_range));
    cmd.roll = static_cast<float>(std::clamp(roll, -config_.roll_range, config_.roll_range));

    return cmd;
}

bool InterceptionPlanner::evaluateStrikeTrigger(
    double time_to_intercept,
    double distance_to_target)
{
    // Strike is triggered when:
    // 1. Robot is close enough to target (< 20cm)
    // 2. Remaining flight time matches mechanical swing delay
    if (distance_to_target > 0.25) {
        return false;
    }

    double diff = std::abs(time_to_intercept - config_.strike_delay_time);
    return (diff <= config_.strike_tolerance_s);
}

} // namespace volleyball_planner
