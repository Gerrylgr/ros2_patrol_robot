#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp" // 导入导航动作消息的头文件
#include "rclcpp/rclcpp.hpp"                     // 导入ROS 2的C++客户端库
#include "rclcpp_action/rclcpp_action.hpp"       // 导入ROS 2的C++ Action客户端库

using NavigationAction = nav2_msgs::action::NavigateToPose; // 定义导航动作类型为NavigateToPose

class NavToPoseClient : public rclcpp::Node
{
public:

    /*
    rclcpp_action::Client 是ROS 2 C++客户端库中的一个模板类，用于创建动作客户端。
    动作客户端用于发送目标到动作服务器，并获取反馈和结果。
    <NavigationAction> 是一个占位符，它表示动作客户端将要与之交互的动作的类型。
    NavigationAction 是一个定义了动作的目标、结果和反馈消息类型的结构。(接口)
    */
    using NavigationActionClient = rclcpp_action::Client<NavigationAction>; // 定义导航动作客户端类型
    /*
    rclcpp_action::ClientGoalHandle 是ROS 2 C++客户端库中的一个模板类，它用于管理客户端发送给服务器的动作目标。
    每个动作目标都有一个与之关联的 ClientGoalHandle，它允许客户端跟踪动作的进度，包括发送取消请求、检查目标是否已完成等。
    */
    using NavigationActionGoalHandle =
        rclcpp_action::ClientGoalHandle<NavigationAction>; // 定义导航动作目标句柄类型

    NavigationActionClient::SharedPtr action_client_;

    NavToPoseClient() : Node("nav_to_pose_client")
    {
        // 创建导航动作客户端
        //"navigate_to_pose" 是动作服务器的名称，客户端将使用这个名称来查找并与之通信。
        action_client_ = rclcpp_action::create_client<NavigationAction>(
            this, "navigate_to_pose");
    }

    void sendGoal()
    {
        // 等待导航动作服务器上线，等待时间为5秒
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }
        // 设置导航目标点
        auto goal_msg = NavigationAction::Goal();
        goal_msg.pose.header.frame_id = "map"; // 设置目标点的坐标系为地图坐标系
        goal_msg.pose.pose.position.x = -6.0f;  // 设置目标点的x坐标为2.0
        goal_msg.pose.pose.position.y = -6.0f;  // 设置目标点的y坐标为2.0

        auto send_goal_options =
            rclcpp_action::Client<NavigationAction>::SendGoalOptions();

        //这三个回调函数会在请求发送后调用
        // 设置请求目标结果回调函数
        send_goal_options.goal_response_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            }
        };
        // 设置移动过程反馈回调函数
        send_goal_options.feedback_callback =
            [this](
                NavigationActionGoalHandle::SharedPtr goal_handle,
                const std::shared_ptr<const NavigationAction::Feedback> feedback)
        {
            (void)goal_handle; // 假装调用，避免 warning: unused
            RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f",
                        feedback->distance_remaining);
        };
        // 设置执行结果回调函数
        send_goal_options.result_callback =
            [this](const NavigationActionGoalHandle::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "处理成功！");
            }
        };

        // 发送导航目标点
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();
    node->sendGoal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}