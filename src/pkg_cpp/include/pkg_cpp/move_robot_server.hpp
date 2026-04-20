#ifndef MOVE_ROBOT_SERVER_HPP
#define MOVE_ROBOT_SERVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
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
    if ((goal->goal_position_x < 0) || (goal->goal_position_x > 100) || (goal->goal_position_y < 0) || (goal->goal_position_y > 100)) {
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
    if (msg->transforms.empty()) return; 

    const auto& t = msg->transforms[0].transform;
    position_x_ = t.translation.x;
    position_y_ = t.translation.y;

    //transformation to Quaternion
    tf2::Quaternion q(
        t.rotation.x,
        t.rotation.y,
        t.rotation.z,
        t.rotation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    position_theta_ = yaw;
  }

  // EXECUTE goal
  void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle) {
    { //active goal
      std::lock_guard<std::mutex> lock(mutex_);
      this->goal_handle_ = goal_handle;
    }

    double goal_position_x = goal_handle->get_goal()->goal_position_x;
    double goal_position_y = goal_handle->get_goal()->goal_position_y;
    double goal_position_theta = goal_handle->get_goal()->goal_position_theta;

    auto result = std::make_shared<MoveRobot::Result>();
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    rclcpp::Rate loop_rate(1.0);

    RCLCPP_INFO(this->get_logger(), "Execute goal");
    while (rclcpp::ok()) {
      // Check if needs to preempt goal
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->final_position_x = position_x_;
                    result->final_position_y = position_y_;
                    result->final_position_theta = position_theta_;
                    result->message = "Preempted by another goal";
                    goal_handle->abort(result);
                    return;
                }
            }
            
      // Check if cancel request
      if (goal_handle->is_canceling()) { 
        result->final_position_x = position_x_;
        result->final_position_y = position_y_;
        result->final_position_theta = position_theta_;
        
        result->message = "Canceled";
        goal_handle->canceled(result);
        return;
      }

      // Check remains ditances
      double diff_x = goal_position_x - position_x_;
      double diff_y = goal_position_y - position_y_;
      double diff_theta = goal_position_theta - position_theta_;
      if ((std::abs(diff_x) < 0.1 && std::abs(diff_y) < 0.1 && std::abs(diff_theta) < 0.1)) {
        //stop robot
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.angular.z = 0;
        vel_publisher_->publish(msg);

        result->final_position_x = position_x_;
        result->final_position_y = position_y_;
        result->final_position_theta = position_theta_;
        result->message = "Success";
        goal_handle->succeed(result);
        return;
      }

      // Compute velocity
      double Kp = 0.5; // proportional gain
      double cmd_vel = Kp * diff_x;
      if (cmd_vel > 0.5)
        cmd_vel = 0.5;
      if (cmd_vel < -0.5)
        cmd_vel = -0.5;
      // publish velocity
      geometry_msgs::msg::Twist msg;
      msg.linear.x = cmd_vel;
      vel_publisher_->publish(msg);

      RCLCPP_INFO(this->get_logger(), "Robot position: (%f, %f) with rotation: %f", position_x_, position_y_, position_theta_);
      feedback->current_position_x = position_x_;
      feedback->current_position_y = position_y_;
      feedback->current_position_theta = position_theta_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }
  }

  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double position_theta_ = 0.0;
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