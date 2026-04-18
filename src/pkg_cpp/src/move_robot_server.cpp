#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using MoveRobot = robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using namespace std::placeholders;


class MoveRobotServerNode : public rclcpp::Node {
public:
    MoveRobotServerNode() : Node("move_robot_server") {
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

        RCLCPP_INFO(this->get_logger(), "Action server has been started");
    }

    

private:
    //GOAL callback
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobot::Goal> goal){
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

    //CANCEL callback - accept request of canceling
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    //handle ACCEPT goal --> execute the goal
    void handle_accepted_callback(const std::shared_ptr<MoveRobotGoalHandle> goal_handle){
        execute_goal(goal_handle);
    }

    //ODOM callback - take position x of the robot
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        double x = msg->pose.pose.position.x;
        position_ = x;
    }
    
    //EXECUTE goal
    void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle){
        double goal_position = goal_handle->get_goal()->position;

        auto result = std::make_shared<MoveRobot::Result>();
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        rclcpp::Rate loop_rate(1.0);

        RCLCPP_INFO(this->get_logger(), "Execute goal");
        while (rclcpp::ok()) {
            
            //Check if cancel request
            if (goal_handle->is_canceling()) {
                result->position = position_;
                if (goal_position == position_) {
                   result->message = "Success";
                   goal_handle->succeed(result); 
                }
                else {
                    result->message = "Canceled";
                    goal_handle->canceled(result);
                }
                return;
            }

            //Check remains ditances
            double diff = goal_position - position_;
            if (std::abs(diff) < 0.1) {
                result->position = position_;
                result->message = "Success";
                goal_handle->succeed(result);
                return;
            }

            //Compute velocity
            double Kp = 0.5;  //proportional gain
            double cmd_vel = Kp*diff;
            if (cmd_vel > 0.5) cmd_vel = 0.5;
            if (cmd_vel < -0.5) cmd_vel = -0.5;
            //publsh velocity
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

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}