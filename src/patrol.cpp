#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <csignal>

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
        this->cg1 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        this->cg2 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        this->initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        this->goal_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose", this->cg2);
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CleanNode::main_task, this), this->cg1);
        /* main_task是一个定时器回调，action_callback也是一个回调，默认处理是一个节点的所有回调是在一个互斥组中。
        这里，将定时器回调与动作回调放在不同的互斥组中，在定时器回调执行的同时，动作回调也可以在另一个线程中执行。 */
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = this->cg2;
        this->pose_complete_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&CleanNode::pose_complete_callback, this, std::placeholders::_1), sub_opt);

        this->in_nav = false;
        this->pose_is_complete = false;
    }

    void main_task(){
        /**
         * @brief 运行的主要任务
         * 
         */
        this->timer_->cancel(); //停止定时器，使得这个主任务只运行一次
        init_pose();
        while(true){
			send_goal(0.5, -1, 0);
			send_goal(3.5, -1, M_PI/2);
			send_goal(3.5, 2, M_PI);
			send_goal(0.5, 2, 3*M_PI/2);
        }
    }

    void wait_for_init_complete(){
        while(! this->pose_is_complete){}
        RCLCPP_INFO(this->get_logger(), "the initial pose is located");
        std::this_thread::sleep_for(std::chrono::seconds(5));
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
        wait_for_init_complete();
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

        while(this->in_nav){
        }
        
        if (!this->goal_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
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
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_complete_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::callback_group::CallbackGroup::SharedPtr cg1;
    rclcpp::callback_group::CallbackGroup::SharedPtr cg2;

    bool in_nav;
    bool pose_is_complete;

    void goal_response_callback(std::shared_future<GoalHandleNav::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation is rejected!");
        rclcpp::shutdown();
        } else {
        RCLCPP_INFO(this->get_logger(), "Navgation start!");
        }
    }

    void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const Action_T::Feedback> feedback)
    {
        // std::stringstream ss;
        // ss << "The current pose is: (" << feedback->current_pose.pose.position.x <<
        // ", " << feedback->current_pose.pose.position.y << ", " << feedback->current_pose.pose.position.z << ")";
        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
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

    void pose_complete_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        this->pose_is_complete = true;
    }
};

} //end of namespace mynav
void sign_handle(int sign){
		if(sign == SIGINT){
			rclcpp::shutdown();
		}
}
int main(int argc, char * argv[])
{
	
	signal(SIGINT, sign_handle);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<mynav::CleanNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(mynav::CleanNode)


