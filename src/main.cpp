#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <chrono>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

class NavigationToPose : public rclcpp::Node
{
private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_global;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_local;
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
        this->initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavigationToPose::main_task, this));
        this->get_costmap_global = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        this->get_costmap_local = this->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");
    }
   
    void main_task(){
        /**
         * @brief 运行的主要任务
         * 
         */
        this->timer_->cancel(); //停止定时器，使得这个主任务只运行一次
        init_pose();
        send_goal(4, 0, M_PI/2);
    }

    void make_plan(){
        /**
         * @brief 依据地图进行清扫任务的规划
         * 
         */
        while(!this->get_costmap_global->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        auto future = this->get_costmap_global->async_send_request(req);
        rclcpp::spin_until_future_complete(std::make_shared<NavigationToPose>(this), future);
        auto global_map = future.get()->map;
    }

    void init_pose(){
        /**
         * @brief 初始化机器人的位置
         * 
         */
        auto initial_pose = geometry_msgs::msg::PoseStamped();
        initial_pose.header.frame_id = "map";
        initial_pose.pose.orientation.w = 1.0;
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

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, yaw);
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = geometry_msgs::msg::PoseStamped();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = static_cast<double>(orientation.getW());
        goal_msg.pose.pose.orientation.x = static_cast<double>(orientation.getX());
        goal_msg.pose.pose.orientation.y = static_cast<double>(orientation.getY());
        goal_msg.pose.pose.orientation.z = static_cast<double>(orientation.getZ());

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&NavigationToPose::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&NavigationToPose::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&NavigationToPose::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
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


