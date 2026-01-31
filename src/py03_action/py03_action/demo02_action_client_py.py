import rclpy
from rclpy.node import Node
import sys
from base_interfaces_demo.action import Progress
from rclpy.logging import get_logger
from rclpy.action import ActionClient

"""  
    需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求；
            3-3.处理目标发送后的反馈；
            3-4.处理连续反馈；
            3-5.处理最终响应。
        4.调用spin函数，并传入节点对象；
        5.释放资源。

"""

class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("ProgressActionClient_node_py")
        self.get_logger().info(f"动作通信客户端成功创建节点ProgressActionClient_node_py")
        # 创建动作客户端
        # node: Any, action_type: Any, action_name
        self.client = ActionClient(self, Progress, "get_sum")
    # 3-2.发送请求；
    def send_goal(self,num):
        # 连接服务端
        self.client.wait_for_server()
        # 发送请求
        goal = Progress.Goal()
        goal.num = num
        self.future = self.client.send_goal_async(goal, self.fb_callback)
        self.future.add_done_callback(self.goal_response_callback)
    # 3-3.处理目标发送后的反馈；
    def goal_response_callback(self, future):
        # 获取目标句柄
        goal_handle = future.result()
        # 判断目标是否被正常接受
        if not goal_handle.accepted:
            self.get_logger().error("目标未被正常接受")
            return
        self.get_logger().info("目标被正常接受")

        # 处理最终响应结果
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("最终响应结果：%d" % result.sum)
    
    # 3-4.处理连续反馈；
    def fb_callback(self, fb_msg):
        progress = fb_msg.feedback.progress
        self.get_logger().info("连续反馈数据：%.2f" % progress)


def main():
    #动态解析传入参数
    if len(sys.argv) != 2:
        get_logger("rclpy").error("请输入一个整型数据")
        return
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    action_client = ProgressActionServer()
    action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(action_client)

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()