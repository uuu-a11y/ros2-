import rclpy
import sys
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts


"""  
    需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建客户端；
            3-2.等待服务连接；
            3-3.组织请求数据并发送；
        4.创建对象调用其功能，处理响应结果；
        5.释放资源。

"""

class Client(Node):
    def __init__(self):
        super().__init__("add_ints_client")
        self.get_logger().info(f"成功创建节点add_ints_client")
        self.client = self.create_client(AddInts, "add_ints")
        self.req = AddInts.Request()

        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info("等待服务连接...")

    def send_request(self):#回调函数
        self.req.num1 = int(sys.argv[1])
        self.req.num2 = int(sys.argv[2])
        self.future = self.client.call_async(self.req)
        

def main():
    #初始化ros2客户端
    rclpy.init()
    client = Client()
    client.send_request()

    #调用spin函数,传入自定义类对象
    rclpy.spin_until_future_complete(client, client.future)
    try:
        response = client.future.result()
    except Exception as e:
        client.get_logger().info("服务调用失败")
    else:
        client.get_logger().info("服务调用成功")
        client.get_logger().info(f"响应结果是: {response.sum}")

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()