#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "volleyball_tracker/msg/trajectory_prediction.hpp"
#include "volleyball_planner/interception_planner.hpp"

class VolleyballPlannerNode : public rclcpp::Node {
public:
    VolleyballPlannerNode() : Node("volleyball_planner_node") {
        this->declare_parameter<double>("max_chassis_speed", 2.0);
        this->declare_parameter<double>("chassis_kp", 2.2);
        this->declare_parameter<double>("stewart_ready_z", 0.20);
        this->declare_parameter<double>("stewart_strike_z", 0.36);
        this->declare_parameter<double>("strike_delay_time", 0.15);
        this->declare_parameter<double>("strike_duration_s", 0.20);
        this->declare_parameter<double>("strike_distance_thresh", 0.25);
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("stewart_frame", "lower_platform_center");

        base_frame_ = this->get_parameter("base_frame").as_string();
        stewart_frame_ = this->get_parameter("stewart_frame").as_string();

        volleyball_planner::PlannerConfig cfg;
        cfg.max_chassis_speed = this->get_parameter("max_chassis_speed").as_double();
        cfg.chassis_kp = this->get_parameter("chassis_kp").as_double();
        cfg.stewart_ready_z = this->get_parameter("stewart_ready_z").as_double();
        cfg.stewart_strike_z = this->get_parameter("stewart_strike_z").as_double();
        cfg.strike_delay_time = this->get_parameter("strike_delay_time").as_double();
        cfg.strike_duration_s = this->get_parameter("strike_duration_s").as_double();
        cfg.strike_distance_thresh = this->get_parameter("strike_distance_thresh").as_double();
        planner_.setConfig(cfg);

        // TF2 setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to complete dynamic trajectory prediction
        traj_sub_ = this->create_subscription<volleyball_tracker::msg::TrajectoryPrediction>(
            "/volleyball/trajectory_pred", 10,
            std::bind(&VolleyballPlannerNode::trajectoryCallback, this, std::placeholders::_1));

        // Publishers directly targeting volleyball_controller & chassis_controllers
        chassis_pub_ = this->create_publisher<chassis_controllers::msg::ChassisControl>(
            "/chassis_command", 10);

        stewart_pub_ = this->create_publisher<volleyball_controller::msg::StewartControl>(
            "/stewart_command", 10);

        strike_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/volleyball/strike_trigger", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz control loop
            std::bind(&VolleyballPlannerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "VolleyballPlannerNode initialized. Subscribing to /volleyball/trajectory_pred.");
    }

private:
    void trajectoryCallback(const volleyball_tracker::msg::TrajectoryPrediction::SharedPtr msg) {
        last_traj_msg_ = *msg;
        has_traj_ = true;
        last_update_time_ = this->now();
    }

    void controlLoop() {
        if (!has_traj_ || !last_traj_msg_.is_tracking || (this->now() - last_update_time_).seconds() > 0.4) {
            // Stop robot and reset Stewart platform if ball is lost or not tracked
            is_striking_ = false;
            stopChassis();
            resetStewart();
            return;
        }

        // 检查正在执行的 Stewart 顶升击球是否超时回落
        if (is_striking_) {
            if ((this->now() - strike_start_time_).seconds() > planner_.getConfig().strike_duration_s) {
                is_striking_ = false;
                RCLCPP_INFO(this->get_logger(),
                            "Stewart upward counter-strike finished, returning to ready height %.2fm.",
                            planner_.getConfig().stewart_ready_z);
            }
        }

        double time_to_arrive = last_traj_msg_.time_to_landing;
        if (time_to_arrive <= 0.0 && !is_striking_) {
            // Ball already landed or invalid, and not currently executing strike
            stopChassis();
            resetStewart();
            return;
        }

        // 1. Compute target relative to base_link
        geometry_msgs::msg::PointStamped landing_pt;
        landing_pt.header = last_traj_msg_.header;
        landing_pt.point = last_traj_msg_.landing_position;

        geometry_msgs::msg::PointStamped target_in_base;
        if (landing_pt.header.frame_id != base_frame_) {
            try {
                if (tf_buffer_->canTransform(base_frame_, landing_pt.header.frame_id, tf2::TimePointZero)) {
                    target_in_base = tf_buffer_->transform(landing_pt, base_frame_);
                } else {
                    target_in_base = landing_pt;
                }
            } catch (const tf2::TransformException&) {
                target_in_base = landing_pt;
            }
        } else {
            target_in_base = landing_pt;
        }

        Eigen::Vector2d rel_target(target_in_base.point.x, target_in_base.point.y);

        // 2. 评估 Stewart 平台上升反击触发时机
        if (!is_striking_ && planner_.evaluateStrikeTrigger(time_to_arrive, rel_target.norm())) {
            is_striking_ = true;
            strike_start_time_ = this->now();

            std_msgs::msg::Bool trigger_msg;
            trigger_msg.data = true;
            strike_pub_->publish(trigger_msg);
            RCLCPP_INFO(this->get_logger(),
                        "Stewart upward counter-strike TRIGGERED! Remaining time: %.3fs, Dist: %.2fm, Target Z: %.2fm",
                        time_to_arrive, rel_target.norm(), planner_.getConfig().stewart_strike_z);
        }

        // 3. 底盘控制：击打爆发期间底盘急刹锁死以提供最大刚性支撑；拦截机动期间全速向落点对位
        if (is_striking_) {
            stopChassis();
        } else {
            auto chassis_cmd = planner_.planChassisMovement(rel_target, time_to_arrive);
            chassis_pub_->publish(chassis_cmd);
        }

        // 4. Stewart 姿态与高度规划：反击期间冲顶至 stewart_strike_z，平时维持 stewart_ready_z
        Eigen::Vector3d incoming_vel(
            last_traj_msg_.current_velocity.x,
            last_traj_msg_.current_velocity.y,
            last_traj_msg_.current_velocity.z
        );
        auto stewart_cmd = planner_.planStewartPose(incoming_vel, is_striking_);
        stewart_pub_->publish(stewart_cmd);
    }

    void stopChassis() {
        chassis_controllers::msg::ChassisControl stop_cmd;
        stop_cmd.x_speed = 0.0f;
        stop_cmd.y_speed = 0.0f;
        stop_cmd.spin_speed = 0.0f;
        stop_cmd.emergency_stop = 0;
        stop_cmd.power_limit = 0;
        chassis_pub_->publish(stop_cmd);
    }

    void resetStewart() {
        volleyball_controller::msg::StewartControl neutral_cmd;
        neutral_cmd.x = 0.0f;
        neutral_cmd.y = 0.0f;
        neutral_cmd.z = static_cast<float>(planner_.getConfig().default_height); // Nominal height (0.22816m)
        neutral_cmd.roll = 0.0f;
        neutral_cmd.pitch = 0.0f;
        neutral_cmd.yaw = 0.0f;
        neutral_cmd.emergency_stop = 0;
        stewart_pub_->publish(neutral_cmd);
    }

    std::string base_frame_;
    std::string stewart_frame_;
    volleyball_planner::InterceptionPlanner planner_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<volleyball_tracker::msg::TrajectoryPrediction>::SharedPtr traj_sub_;

    rclcpp::Publisher<chassis_controllers::msg::ChassisControl>::SharedPtr chassis_pub_;
    rclcpp::Publisher<volleyball_controller::msg::StewartControl>::SharedPtr stewart_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr strike_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    volleyball_tracker::msg::TrajectoryPrediction last_traj_msg_;
    bool has_traj_{false};
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

    bool is_striking_{false};
    rclcpp::Time strike_start_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VolleyballPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
