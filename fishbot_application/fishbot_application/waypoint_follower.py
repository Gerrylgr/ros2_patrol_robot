from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    #创建点集
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.0
    goal_pose1.pose.position.y = 1.0
    goal_pose1.pose.orientation.w = 1.0

    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -5.0
    goal_pose2.pose.position.y = -3.0
    goal_pose2.pose.orientation.w = 1.0

    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -4.0
    goal_pose3.pose.position.y = -6.0
    goal_pose3.pose.orientation.w = 1.0

    goal_poses.append(goal_pose3)
    #调用导航服务
    navigator.followWaypoints(goal_poses)
    #获取反馈
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'当前目标:{feedback.current_waypoint}')
    #最终结果判断
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('导航被取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('导航失败')
    else:
        navigator.get_logger().info('返回状态无效')

