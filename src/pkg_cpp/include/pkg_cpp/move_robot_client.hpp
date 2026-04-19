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
    void send_goal(int position){
        move_robot_client_->wait_for_action_server();

        auto goal = MoveRobot::Goal();
        goal.position = position;

        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MoveRobotClientNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&MoveRobotClientNode::goal_result_callback, this, _1);
        options.feedback_callback = std::bind(&MoveRobotClientNode::goal_feedback_callback, this, _1, _2);

        RCLCPP_INFO(this->get_logger(), "Send goal with position: %d", position);
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
        
        double position = result.result->position;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Position: %f", position);
        RCLCPP_INFO(this->get_logger(), "Message: %s", message.c_str());
    }

    //FEEDBACK callback - send current robot position
    void goal_feedback_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const MoveRobot::Feedback> feedback)
    {
        (void)goal_handle;
        double position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Feedback position: %f", position);
    }



    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;
    MoveRobotGoalHandle::SharedPtr goal_handle_;
};
} //namespace robot_namespace


#endif