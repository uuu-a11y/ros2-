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


import os
# 填写launch文件，启动 slam_toolbox 的异步建图节点
def generate_launch_description():
    # 为节点加载配置文件

    # # 高耦合，灵活性差，不方便实现代码移植
    # params_path = os.path.join(get_package_share_directory('mycar_slam_slam_toolbox'), 'params', 'online_sync_slam.yaml')
    # 解决： 使用 LaunchArgument 动态传参
    # 实现：
    # 1.DeclareLaunchArgument 声明 yaml 文件路径
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    params_path = DeclareLaunchArgument("params_path", default_value=os.path.join(get_package_share_directory('mycar_slam_slam_toolbox'), 'params', 'online_sync_slam.yaml'))

    # 创建同步建图节点对应的node对象
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="my_slam_toolbox",
        # parameters=[params_path]
        # 2. LaunchConfiguration 获取 yaml 文件路径
        parameters=[LaunchConfiguration("params_path"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    return LaunchDescription(
        [
            use_sim_time,
            params_path,
            slam_node
        ]
    )


# 3.执行launch文件是，动态传入yaml文件








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