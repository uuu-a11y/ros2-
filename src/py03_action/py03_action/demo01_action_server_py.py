from argparse import Action
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces_demo.action import Progress
import time

"""  
    需求：编写动作服务端实习，可以提取客户端请求提交的整型数据，并累加从1到该数据之间的所有整数以求和，
       每累加一次都计算当前运算进度并连续反馈回客户端，最后，在将求和结果返回给客户端。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建动作服务端；
            3-2.生成连续反馈；
            3-3.生成最终响应。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""

class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("ProgressActionClient_node_py")
        self.get_logger().info(f"动作通信服务端成功创建节点ProgressActionClient_node_py")
        #3-1.创建动作服务端；
        #node: Any, action_type: Any, action_name: Any, execute_callback
        self.server = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback

        )

    #3-2.生成连续反馈；
    def execute_callback(self, goal_handle):
        num = goal_handle.request.num
        sum = 0
        
        for i in range(1, num + 1):
            sum += i
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("连续反馈：%.2f" % feedback.progress)
            time.sleep(1.0)
        #3-3.生成最终响应。
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum
        self.get_logger().info("计算结果：%d" % result.sum)
        return result


def main():
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    rclpy.spin(ProgressActionServer())

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()