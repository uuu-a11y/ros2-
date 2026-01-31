from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    x = 6
    y = 9
    theta = 0.0
    name = "t2"
    spawn = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': " 
             + str(x) + ",'y': " + str(y) 
             + ",'theta': " + str(theta) 
             + ",'name': '" + name + "'}\""],
        output="both",
        shell=True
    )
    
    client = Node(package="cpp07_exercise", executable="exer05_action_client", arguments=[str(x), str(y), str(theta)])
    
    
    
    return LaunchDescription([spawn, client])