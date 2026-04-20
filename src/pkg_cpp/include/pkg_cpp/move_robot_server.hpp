#ifndef MOVE_ROBOT_SERVER_HPP
#define MOVE_ROBOT_SERVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"


//ros2 interface show tf2_msgs/msg/TFMessage
// geometry_msgs/TransformStamped[] transforms
// 	#
// 	#
// 	std_msgs/Header header
// 		builtin_interfaces/Time stamp
// 			int32 sec
// 			uint32 nanosec
// 		string frame_id
// 	string child_frame_id
// 	Transform transform
// 		Vector3 translation
// 			float64 x
// 			float64 y
// 			float64 z
// 		Quaternion rotation
// 			float64 x 0
// 			float64 y 0
// 			float64 z 0
// 			float64 w 1


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
  goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobot::Goal> goal) {
    (void)uuid; //avoid warning erro
    RCLCPP_INFO(this->get_logger(), "Received a new goal");

    // Validate new goal
    if ((goal->position < 0) || (goal->position > 100)) {
      RCLCPP_INFO(this->get_logger(), "Invalid position: reject goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // New goal arrived --> preempt previosly one
    {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
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
  void handle_accepted_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
    execute_goal(goal_handle);
  }

  // TF callback - take position of the robot
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    const auto& t = msg->transforms[0].transform;
    double x = t.translation.x;
    position_ = x;
  }

  // EXECUTE goal
  void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
    { //active goal
      std::lock_guard<std::mutex> lock(mutex_);
      this->goal_handle_ = goal_handle;
    }

    double goal_position = goal_handle->get_goal()->position;

    auto result = std::make_shared<MoveRobot::Result>();
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    rclcpp::Rate loop_rate(1.0);

    RCLCPP_INFO(this->get_logger(), "Execute goal");
    while (rclcpp::ok()) {
      // Check if needs to preempt goal
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->position = position_;
                    result->message = "Preempted by another goal";
                    goal_handle->abort(result);
                    return;
                }
            }
            
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
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
  std::mutex mutex_;
  rclcpp_action::GoalUUID preempted_goal_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};
} //namespace robot_namespace

#endif