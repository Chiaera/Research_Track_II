#ifndef MOVE_ROBOT_SERVER_HPP
#define MOVE_ROBOT_SERVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"

using MoveRobot = robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using namespace std::placeholders;

namespace robot_namespace {
class MoveRobotServerNode : public rclcpp::Node {
public:
  MoveRobotServerNode(const rclcpp::NodeOptions &options);

private:
  // GOAL callback
  rclcpp_action::GoalResponse
  goal_callback(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const MoveRobot::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received a new goal");

    // Validate new goal
    if ((goal->position < 0) || (goal->position > 100)) {
      RCLCPP_INFO(this->get_logger(), "Invalid position: reject goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Accept goal
    RCLCPP_INFO(this->get_logger(), "Accept goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // CANCEL callback - accept request of canceling
  rclcpp_action::CancelResponse
  cancel_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // handle ACCEPT goal --> execute the goal
  void handle_accepted_callback(
      const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
    execute_goal(goal_handle);
  }

  // ODOM callback - take position x of the robot
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    position_ = x;
  }

  // EXECUTE goal
  void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
    double goal_position = goal_handle->get_goal()->position;

    auto result = std::make_shared<MoveRobot::Result>();
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    rclcpp::Rate loop_rate(1.0);

    RCLCPP_INFO(this->get_logger(), "Execute goal");
    while (rclcpp::ok()) {

      // Check if cancel request
      if (goal_handle->is_canceling()) { // reach the position the exact moment
                                         // the cancel is sent
        result->position = position_;
        if (goal_position == position_) {
          result->message = "Success";
          goal_handle->succeed(result);
        } else {
          result->message = "Canceled";
          goal_handle->canceled(result);
        }
        return;
      }

      // Check remains ditances
      double diff = goal_position - position_;
      if (std::abs(diff) < 0.1) {
        // stop robot
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0;
        vel_publisher_->publish(msg);

        result->position = position_;
        result->message = "Success";
        goal_handle->succeed(result);
        return;
      }

      // Compute velocity
      double Kp = 0.5; // proportional gain
      double cmd_vel = Kp * diff;
      if (cmd_vel > 0.5)
        cmd_vel = 0.5;
      if (cmd_vel < -0.5)
        cmd_vel = -0.5;
      // publish velocity
      geometry_msgs::msg::Twist msg;
      msg.linear.x = cmd_vel;
      vel_publisher_->publish(msg);

      RCLCPP_INFO(this->get_logger(), "Robot position: %f", position_);
      feedback->current_position = position_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }
  }

  double position_ = 0.0;
  double vel_ = 0.0;
  rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
};
} // namespace robot_namespace

#endif