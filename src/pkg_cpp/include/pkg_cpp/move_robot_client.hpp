#ifndef MOVE_ROBOT_CLIENT_HPP
#define MOVE_ROBOT_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"

using MoveRobot = robot_interfaces::action::MoveRobot;
using namespace std::placeholders;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;


namespace robot_namespace {
class MoveRobotClientNode : public rclcpp::Node {
public:
    MoveRobotClientNode(const rclcpp::NodeOptions &options);

private:
    void send_goal(double x, double y, double theta){
        move_robot_client_->wait_for_action_server();

        auto goal = MoveRobot::Goal();
        goal.goal_position_x = x;
        goal.goal_position_y = y;
        goal.goal_position_theta = theta;

        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MoveRobotClientNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&MoveRobotClientNode::goal_result_callback, this, _1);
        options.feedback_callback = std::bind(&MoveRobotClientNode::goal_feedback_callback, this, _1, _2);

        RCLCPP_INFO(this->get_logger(), "Send goal with position: (%f, %f), with rotation: %f", x, y, theta);
        move_robot_client_->async_send_goal(goal, options);
    }

private:
    //RESPONSE callback - accept or reject the goal
    void goal_response_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle){
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    //RESULT callback - return final position of the robot and status message
    void goal_result_callback(const MoveRobotGoalHandle::WrappedResult &result)
    {
        auto status = result.code;        
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        
        double position_x = result.result->final_position_x;
        double position_y = result.result->final_position_y;
        double position_theta = result.result->final_position_theta;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Position: (%f, %f), Rotation: %f", position_x, position_y, position_theta);
        RCLCPP_INFO(this->get_logger(), "Message: %s", message.c_str());
    }

    //FEEDBACK callback - send current robot position
    void goal_feedback_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const MoveRobot::Feedback> feedback)
    {
        (void)goal_handle;
        double position_x = feedback->current_position_x;
        double position_y = feedback->current_position_y;
        double position_theta = feedback->current_position_theta;
        RCLCPP_INFO(this->get_logger(), "Feedback - Position: (%f, %f), Rotation: %f", position_x, position_y, position_theta);
    }



    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;
    MoveRobotGoalHandle::SharedPtr goal_handle_;
};
} //namespace robot_namespace


#endif