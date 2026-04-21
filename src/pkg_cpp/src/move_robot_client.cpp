#include "rclcpp/rclcpp.hpp"
#include "pkg_cpp/move_robot_client.hpp"

namespace robot_namespace {
    MoveRobotClientNode::MoveRobotClientNode(const rclcpp::NodeOptions &options) : Node("move_robot_client", options){
        move_robot_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");
        cmd_subscriber_ = this->create_subscription<robot_interfaces::msg::SendUserCommand>("/send_command", 10, std::bind(&MoveRobotClientNode::cmd_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action client has been started");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_namespace::MoveRobotClientNode)