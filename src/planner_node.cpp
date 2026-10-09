#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "volleyball_planner/interception_planner.hpp"

class VolleyballPlannerNode : public rclcpp::Node {
public:
    VolleyballPlannerNode() : Node("volleyball_planner_node") {
        this->declare_parameter<double>("max_chassis_speed", 2.0);
        this->declare_parameter<double>("chassis_kp", 2.2);
        this->declare_parameter<double>("strike_delay_time", 0.41);
        this->declare_parameter<double>("strike_tolerance_s", 0.05);
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("stewart_frame", "lower_platform_center");

        base_frame_ = this->get_parameter("base_frame").as_string();
        stewart_frame_ = this->get_parameter("stewart_frame").as_string();

        volleyball_planner::PlannerConfig cfg;
        cfg.max_chassis_speed = this->get_parameter("max_chassis_speed").as_double();
        cfg.chassis_kp = this->get_parameter("chassis_kp").as_double();
        cfg.strike_delay_time = this->get_parameter("strike_delay_time").as_double();
        cfg.strike_tolerance_s = this->get_parameter("strike_tolerance_s").as_double();
        planner_.setConfig(cfg);

        // TF2 setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriptions
        ball_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/volleyball/position_filtered", rclcpp::SensorDataQoS(),
            std::bind(&VolleyballPlannerNode::ballCallback, this, std::placeholders::_1));

        landing_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/volleyball/landing_prediction", 10,
            std::bind(&VolleyballPlannerNode::landingCallback, this, std::placeholders::_1));

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

        RCLCPP_INFO(this->get_logger(), "VolleyballPlannerNode initialized.");
    }

private:
    void ballCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_ball_msg_ = *msg;
        has_ball_ = true;
        last_ball_time_ = this->now();
    }

    void landingCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_landing_msg_ = *msg;
        has_landing_ = true;
    }

    void controlLoop() {
        if (!has_ball_ || (this->now() - last_ball_time_).seconds() > 0.4) {
            // Stop robot if ball is lost
            stopChassis();
            return;
        }

        // Transform landing point to robot base frame
        geometry_msgs::msg::PointStamped target_in_base;
        if (has_landing_) {
            try {
                if (tf_buffer_->canTransform(base_frame_, last_landing_msg_.header.frame_id, tf2::TimePointZero)) {
                    target_in_base = tf_buffer_->transform(last_landing_msg_, base_frame_);
                } else {
                    target_in_base = last_landing_msg_;
                }
            } catch (const tf2::TransformException& ex) {
                target_in_base = last_landing_msg_;
            }
        } else {
            target_in_base = last_ball_msg_;
        }

        Eigen::Vector2d rel_target(target_in_base.point.x, target_in_base.point.y);
        double est_time_to_arrive = 1.0; // Estimated from ball trajectory

        // 1. Compute and publish chassis command
        auto chassis_cmd = planner_.planChassisMovement(rel_target, est_time_to_arrive);
        chassis_pub_->publish(chassis_cmd);

        // 2. Compute and publish Stewart orientation
        Eigen::Vector3d incoming_vel(0.0, -5.0, -3.0); // Derived from EKF state
        auto stewart_cmd = planner_.planStewartPose(incoming_vel);
        stewart_pub_->publish(stewart_cmd);

        // 3. Evaluate strike timing
        if (planner_.evaluateStrikeTrigger(est_time_to_arrive, rel_target.norm())) {
            std_msgs::msg::Bool trigger_msg;
            trigger_msg.data = true;
            strike_pub_->publish(trigger_msg);
            RCLCPP_INFO(this->get_logger(), "Strike trigger sent!");
        }
    }

    void stopChassis() {
        chassis_controllers::msg::ChassisControl stop_cmd;
        stop_cmd.x_speed = 0.0f;
        stop_cmd.y_speed = 0.0f;
        stop_cmd.spin_speed = 0.0f;
        chassis_pub_->publish(stop_cmd);
    }

    std::string base_frame_;
    std::string stewart_frame_;
    volleyball_planner::InterceptionPlanner planner_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr landing_sub_;

    rclcpp::Publisher<chassis_controllers::msg::ChassisControl>::SharedPtr chassis_pub_;
    rclcpp::Publisher<volleyball_controller::msg::StewartControl>::SharedPtr stewart_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr strike_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PointStamped last_ball_msg_;
    geometry_msgs::msg::PointStamped last_landing_msg_;
    bool has_ball_{false};
    bool has_landing_{false};
    rclcpp::Time last_ball_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VolleyballPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
