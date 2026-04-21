#ifndef MOVE_ROBOT_SERVER_HPP
#define MOVE_ROBOT_SERVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
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

//FRAMES
//base_footprint --> base_link --> left_wheel
//                             --> right wheel
//                             --> scan link


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

  // ODOM callback - take position x of the robot
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;

    //assign header and child frame 
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    //Get /odom pose (position and orientation) to send a broadcast
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    //publish the transformation
    tf_broadcaster_->sendTransform(t);
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
    rclcpp::Rate loop_rate(3.0);

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
      geometry_msgs::msg::Twist stop_msg;
      vel_publisher_->publish(stop_msg);

      result->final_position_x = position_x_;
      result->final_position_y = position_y_;
      result->final_position_theta = position_theta_;
      
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    //get position ---
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        position_x_ = transform.transform.translation.x;
        position_y_ = transform.transform.translation.y;
    } catch (tf2::TransformException &ex) {RCLCPP_WARN(this->get_logger(), "TF could not transform: %s", ex.what());
        loop_rate.sleep();
        continue;
    }

    //transformation to Quaternion
    tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    position_theta_ = yaw;
    //---------------------

      // Check remains ditances
      double diff_x = goal_position_x - position_x_;
      double diff_y = goal_position_y - position_y_;
      double diff_theta = goal_position_theta - position_theta_;
      //angle normalization
      while (diff_theta > M_PI) diff_theta -= 2*M_PI;
      while (diff_theta < -M_PI) diff_theta += 2*M_PI;
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

      // VELOCITY ---
      double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
      double current_theta = std::atan2(diff_y, diff_x);
      double delta_theta = current_theta - position_theta_;
      //angle normalization
      while (delta_theta > M_PI) delta_theta -= 2*M_PI;
      while (delta_theta < -M_PI) delta_theta += 2*M_PI;

      double delta_theta_final = goal_position_theta -position_theta_;
      //angle normalization
      while (delta_theta_final > M_PI) delta_theta_final -= 2*M_PI;
      while (delta_theta_final < -M_PI) delta_theta_final += 2*M_PI;

      geometry_msgs::msg::Twist msg;
      if (dist > 0.1) { //if robot distant from target
          if (std::abs(delta_theta) > 0.2) { //fix position
              msg.angular.z = 0.5*delta_theta;
              msg.linear.x = 0.0;
          } 
          else { //fix distance
              msg.linear.x = 2*dist; 
              msg.angular.z = 0.0;
          }
      } else { //reached position 
          if (std::abs(delta_theta_final) > 0.05) { //check orientation
              msg.linear.x = 0.0;
              msg.angular.z = 0.4*delta_theta_final;
          } else { //stop
              msg.linear.x = 0.0;
              msg.angular.z = 0.0;
          }
      }
      vel_publisher_->publish(msg);
      //--------

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
  rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
  std::mutex mutex_;
  rclcpp_action::GoalUUID preempted_goal_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};
} //namespace robot_namespace

#endif