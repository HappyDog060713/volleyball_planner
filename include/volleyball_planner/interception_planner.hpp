#pragma once

#include <Eigen/Dense>
#include "chassis_controllers/msg/chassis_control.hpp"
#include "volleyball_controller/msg/stewart_control.hpp"

namespace volleyball_planner {

struct PlannerConfig {
    double max_chassis_speed{2.5};     // m/s
    double chassis_kp{2.0};
    double deadband_dist{0.03};        // 3cm
    double default_height{0.22816};    // Stewart nominal height (m)
    double pitch_range{0.35};          // rad (~20 deg)
    double roll_range{0.35};           // rad
    double strike_delay_time{0.41};    // Mechanical strike delay (s)
    double strike_tolerance_s{0.05};   // +/- 50ms trigger window
    Eigen::Vector3d target_outgoing_vel{0.0, 8.0, 3.5}; // Desired exit velocity towards opponent court
};

class InterceptionPlanner {
public:
    explicit InterceptionPlanner(const PlannerConfig& config = PlannerConfig());
    ~InterceptionPlanner() = default;

    void setConfig(const PlannerConfig& config);

    // 1. Chassis movement command toward predicted interception point
    chassis_controllers::msg::ChassisControl planChassisMovement(
        const Eigen::Vector2d& rel_target_pos_in_base,
        double time_to_arrive);

    // 2. Stewart orientation for desired return trajectory
    volleyball_controller::msg::StewartControl planStewartPose(
        const Eigen::Vector3d& incoming_vel_in_base);

    // 3. Strike trigger timing verification
    bool evaluateStrikeTrigger(
        double time_to_intercept,
        double distance_to_target);

private:
    PlannerConfig config_;
};

} // namespace volleyball_planner
