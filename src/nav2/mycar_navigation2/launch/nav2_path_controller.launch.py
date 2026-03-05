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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
"""
导航服务器的核心节点实现
1.基础部分实现
    1-1.生命周期管理器节点
        在Nav2中，所有节点都是具有生命的节点
        生命周期管理器节点会提供一套标准的方法，来管理Nav2中的节点
    1-2.行为树服务器节点
        控制导航的执行流程的
2.规划器实现
    生成从当前位置到目标点的路径
    2-1。规划器服务节点
    2-2.全局代价地图
    规划器节点依赖于全局代价地图
3.运动控制实现
    控制机器人按照规划的路径移动
    3-1.运动控制节点
    3-2.局部地图
    运动控制节点依赖于局部代价地图
"""
def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(name="use_sim_time",default_value="true" )
    current_pkg = get_package_share_directory("mycar_navigation2")
    bt_yaml = os.path.join(current_pkg, "params", "bt.yaml")
    planner_yaml = os.path.join(current_pkg, "params" ,"planner.yaml")
    controller_yaml = os.path.join(current_pkg, "params" ,"controller.yaml")

    # 运动控制节点
    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            controller_yaml,


        ]
    )

    # 创建规划器节点
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            planner_yaml,
            
        ]
    )

    # 行为树服务器
    bt_server =  Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            # 自定义行为树并载入
            {"default_nav_to_pose_bt_xml": os.path.join(current_pkg, "bts", "nav2_pose.xml")},
            {"default_nav_through_poses_bt_xml": os.path.join(current_pkg, "bts", "nav2_poses.xml")},
            # 加载yaml文件
            bt_yaml,

        ]
    )

    # 生命周期管理器节点
    life_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nav2",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": True},
            {"node_names":[# 被托管的节点列表
                "bt_navigator",
                "planner_server",
                "controller_server"
            ]}
        ]
    )
    return LaunchDescription(
        [
            controller_node,
            planner_server,
            use_sim_time,
            life_manager,
            bt_server # 创建行为管理服务器



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