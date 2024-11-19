from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    #等待导航启动完成
    navigator.waitUntilNav2Active()
    #设置目标点坐标
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = 'map'
    pose_goal.header.stamp = navigator.get_clock().now().to_msg()
    pose_goal.pose.position.x = 1.0
    pose_goal.pose.position.y = 1.0
    pose_goal.pose.orientation.w = 1.0
    #发送请求并处理反馈
    navigator.goToPose(pose_goal)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s后到达')
        #超时取消
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
    #处理结果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('导航超时被取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('导航失败')
    else:
        navigator.get_logger().info('返回状态无效')


"""
feedback.estimated_time_remaining: 这是一个time类型的ROS 2消息，表示导航器估计的到达目标位置所需剩余时间。
Duration.from_msg(...): 这是一个将ROS 2 time 消息转换为 Duration 对象的函数。Duration 对象用于表示时间间隔。
.nanoseconds: 这是Duration对象的一个属性，返回时间间隔的纳秒表示。
/ 1e9: 这是将纳秒转换为秒的数学操作。由于1秒等于1e9纳秒，因此通过除以1e9可以将纳秒转换为秒。
Duration(seconds=600.0) 是创建 Duration 对象的一种方式，它表示一个持续时间为600秒的时间间隔。
"""