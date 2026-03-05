#install(DIRECTORY config params launch DESTINATION share/${PROJECT_NAME}) #cmake配置
#<exec_depend>ros2launch</exec_depend> <!--package.xml配置-->
#from glob import glob #用于setup.py配置多个launch文件
#('share/' + package_name + '/launch', glob('launch/*launch.py')),
#('share/' + package_name + '/launch', glob('launch/*launch.xml')),
#('share/' + package_name + '/launch', glob('launch/*launch.yaml')),
#(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

from launch import LaunchDescription
from launch_ros.actions import Node
import os
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition #判断是否执行
# from launch.conditions import UnlessCondition #取反
# from launch.substitutions import PythonExpression #运行时计算表达式
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command

import os

def generate_launch_description():
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
    this_pkg_path = get_package_share_directory("demo_gazebo_sim")
    gz_sim = IncludeLaunchDescription(

        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v 4 -r " + os.path.join(this_pkg_path, "world","visualize_lidar.sdf")
        }.items()
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(this_pkg_path, "rviz", "sim.rviz")]
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        remappings=[
            ("model/vehicle_blue/cmd_vel", "cmd_vel"),
            ("model/vehicle_blue/tf", "tf")
        ]
    )


    return LaunchDescription(
        [ 
            gz_sim,
            rviz2_node,
            bridge


        ]
    )








# urdf实例代码:
"""
# 1.启动robot_satte_publisher节点,该节点要以参数的方式加载urdf文件内筒;
p_value = ParameterValue(value=Command(["xacro ",get_package_share_directory('cpp06_urdf') + '/urdf/urdf/demo01_helloworld.urdf']),value_type=str)
robot_state_pub = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': p_value}]
)
"""