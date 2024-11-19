import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path + '/config/display_robot_model.rviz'
    # 声明模型路径参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    #创建了一个 DeclareLaunchArgument 动作，用于声明启动文件的参数 model。
    # 该参数指定 URDF 文件路径，默认值是 first_robot.urdf 的路径。

    # 定义 robot_description 参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    #launch.substitutions.LaunchConfiguration('model') 读取 model 参数的值。
    #substitutions.Command(['cat', ...]) 使用 cat 命令读取 URDF 文件的内容。
    #ParameterValue(..., value_type=str) 将 URDF 文件内容作为 robot_description 参数，以字符串形式传递。

    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
        #需要传入文件内容
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
        #-d 指定 RViz 配置文件的路径 default_rviz_config_path，该配置文件可以定义 RViz 界面的显示设置。
        #相当于命令行中输入:ros2 run rviz2 rviz2 -d /home/gerry/chapt6/src/fishbot_description/config/display_robot_model.rviz
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])

"""

在 RViz 的 RobotModel 中，description topic 通常是从 robot_state_publisher 节点获取的。该节点负责发布机器人的模型描述，通常是 URDF 或 XACRO 文件的内容。

工作流程如下：
机器人描述的发布：

在启动文件中，robot_state_publisher 节点会被配置为订阅一个名为 /robot_description 的 topic。
这个 topic 的内容是机器人模型的描述，通常是 URDF 文件的内容。
URDF/XACRO 的读取：

启动时，通过 robot_description 参数将 URDF 或 XACRO 文件的内容传递给 robot_state_publisher 节点。这个参数的值是一个字符串，包含了机器人的模型描述。
机器人状态的更新：

robot_state_publisher 还会从其他节点（如 joint_state_publisher）获取机器人的关节状态，并更新模型在 RViz 中的表现。
RViz 的订阅：

RViz 的 RobotModel 组件会订阅 /robot_description topic，并使用其中的模型描述来绘制机器人的 3D 模型。
"""