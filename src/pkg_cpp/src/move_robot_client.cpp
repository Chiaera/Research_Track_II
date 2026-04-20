#include "rclcpp/rclcpp.hpp"
#include "pkg_cpp/move_robot_client.hpp"

namespace robot_namespace {
    MoveRobotClientNode::MoveRobotClientNode(const rclcpp::NodeOptions &options) : Node("move_robot_client", options){
        move_robot_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");

        RCLCPP_INFO(this->get_logger(), "Action client has been started");

        send_goal(6, 3, 0);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_namespace::MoveRobotClientNode)