#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigationToPose : public rclcpp::Node
{
private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

    void goal_response_callback(std::shared_future<GoalHandleNav::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation is rejected!");
        } else {
        RCLCPP_INFO(this->get_logger(), "Navgation start!");
        }
    }

    void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        std::stringstream ss;
        // ss << "Next number in sequence received: ";
        ss << "The current pose is: (" << feedback->current_pose.pose.position.x <<
        ", " << feedback->current_pose.pose.position.y << ", " << feedback->current_pose.pose.position.z << ")";
        // for (auto number : feedback->partial_sequence) {
        // ss << number << " ";
        // }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
    
    void result_callback(const GoalHandleNav::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Goal reached!";
        // for (auto number : result.result->sequence) {
        // ss << number << " ";
        // }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }

public:
    NavigationToPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("NavgationToPose", options)
    {
        this->goal_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavigationToPose::send_goal, this));
    }

    void send_goal(){
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->goal_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = geometry_msgs::msg::PoseStamped();
        goal_msg.pose.pose.position.x = 4.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&NavigationToPose::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&NavigationToPose::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&NavigationToPose::result_callback, this, _1);
        this->goal_client_->async_send_goal(goal_msg, send_goal_options);
    }
    ~NavigationToPose(){}
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationToPose>());
    rclcpp::shutdown();
    return 0;
}
// RCLCPP_COMPONENTS_REGISTER_NODE(mynav::NavigationToPose)


