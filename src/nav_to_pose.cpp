#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

namespace mynav
{
class CleanNode : public rclcpp::Node
{
public:
    using Action_T = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<Action_T>;
    CleanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("my_navigator", options)
    {
        this->initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        this->goal_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CleanNode::main_task, this));
        this->get_costmap_global = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        this->get_costmap_local = this->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");

        in_nav = false; 
    }
   
    void main_task(){
        /**
         * @brief 运行的主要任务
         * 
         */
        this->timer_->cancel(); //停止定时器，使得这个主任务只运行一次
        // init_pose();
        send_goal(0.5, -1, 0);
        send_goal(4, 0, M_PI/2);
        // make_plan();
    }

    void make_plan(){
        /**
         * @brief 依据地图进行清扫任务的规划
         * 
         */
        while(!this->get_costmap_global->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        auto global_response_callback = [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture future){
            auto map = future.get()->map;
            auto data = map.data;
            auto metadata = map.metadata;
            RCLCPP_INFO(this->get_logger(), "获取了全局地图。");
        };
        auto future = this->get_costmap_global->async_send_request(req, global_response_callback);
        // rclcpp::spin_until_future_complete(this, future);
    }

    void init_pose(){
        /**
         * @brief 初始化机器人的位置
         * 
         */
        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose.orientation.w = 1.0;
        RCLCPP_INFO(this->get_logger(), "start init pose");
        this->initial_pose_publisher_->publish(initial_pose);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "init pose complete");
    }

    void send_goal(double x, double y, double yaw){
        /**
         * @brief 发送导航目标位置
         * @param x 位置的x坐标
         * @param y 位置的y坐标
         * @param yaw 位置的偏航角，使用弧度制，极坐标形式
         */
        using namespace std::placeholders;

        // this->timer_->cancel();

        if (!this->goal_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        while(in_nav){
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, yaw);
        auto goal_msg = Action_T::Goal();
        goal_msg.pose = geometry_msgs::msg::PoseStamped();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = static_cast<double>(orientation.getW());
        goal_msg.pose.pose.orientation.x = static_cast<double>(orientation.getX());
        goal_msg.pose.pose.orientation.y = static_cast<double>(orientation.getY());
        goal_msg.pose.pose.orientation.z = static_cast<double>(orientation.getZ());

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Action_T>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&CleanNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&CleanNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&CleanNode::result_callback, this, _1);
        this->goal_client_->async_send_goal(goal_msg, send_goal_options);
        this->in_nav = true;
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr goal_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_global;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_local;

    bool in_nav; //表示是否正在导航过程中。

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
    const std::shared_ptr<const Action_T::Feedback> feedback)
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
        // rclcpp::shutdown();
        this->in_nav = false;
    }
};

} //end of namespace mynav

RCLCPP_COMPONENTS_REGISTER_NODE(mynav::CleanNode)


