#install(DIRECTORY config params launch DESTINATION share/${PROJECT_NAME}) #cmake配置
#<exec_depend>ros2launch</exec_depend> <!--package.xml配置-->
#from glob import glob #用于setup.py配置多个launch文件
#('share/' + package_name + '/launch', glob('launch/*launch.py')),
#('share/' + package_name + '/launch', glob('launch/*launch.xml')),
#('share/' + package_name + '/launch', glob('launch/*launch.yaml')),
#(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

from launch import LaunchDescription
from launch_ros.actions import Node, node
import os
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition #判断是否执行
# from launch.conditions import UnlessCondition #取反
# from launch.substitutions import PythonExpression #运行时计算表达式
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
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


# 需求：调用cartographer实现SLAM

def generate_launch_description():

    # 声明仿真时间参数
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",default_value="True"
    )
    dirc = DeclareLaunchArgument("dirc", default_value=os.path.join(get_package_share_directory("mycar_slam_cartographer"),"params"))
    base_name = DeclareLaunchArgument("base_name", default_value="mycar.lua")

    # 启动节点 1：子地图发布节点（cartographer_node）
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        # 配置文件加载 --- 设置文件路径（所属目录 + 文件名称）
        arguments=[
            # 设置目录路径
            "-configuration_directory", LaunchConfiguration("dirc"),
            # 设置配置文件
            "-configuration_basename", LaunchConfiguration("base_name")

        ]
    )
    # 启动节点2：地图拼接节点（cartographer_occupancy_grid_node）
    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],

    )

    return LaunchDescription(
        [
            dirc,
            base_name,
            use_sim_time,
            cartographer_node,
            cartographer_occupancy_grid_node


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