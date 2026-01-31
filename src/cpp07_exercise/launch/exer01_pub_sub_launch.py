from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    t1 = Node(package="turtlesim",executable="turtlesim_node")
    t2 = Node(package="turtlesim",executable="turtlesim_node", namespace="t2")
    
    rotate = ExecuteProcess(
        cmd=["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        output="both",
        shell=True
    )
    exer01 = Node(package="cpp07_exercise",executable="exer01_pub_sub")
    register_rotate_exit_event = RegisterEventHandler(
        event_handler= OnProcessExit(
            target_action=rotate,
            on_exit=exer01
        )
    )

    return LaunchDescription([
        t1, t2, rotate, register_rotate_exit_event
    ])
