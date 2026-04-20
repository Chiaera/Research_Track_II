#include "rclcpp/rclcpp.hpp"
#include "pkg_cpp/move_robot_server.hpp"

namespace robot_namespace {
    MoveRobotServerNode::MoveRobotServerNode(const rclcpp::NodeOptions &options) : Node("move_robot_server", options){
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<MoveRobot>(
                this,
                "move_robot",
                std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
                std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
                std::bind(&MoveRobotServerNode::handle_accepted_callback, this, _1),
                rcl_action_server_get_default_options(),
                cb_group_
        );

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MoveRobotServerNode::odom_callback, this, _1));
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Action server has been started");

    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_namespace::MoveRobotServerNode)