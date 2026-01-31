import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts


"""  
    需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建服务端；
            3-2.处理请求数据并响应结果。
        4.调用spin函数，并传入节点对象；
        5.释放资源。

"""

class AddIntsSever(Node):
    def __init__(self):
        super().__init__("add_ints_sever")
        self.get_logger().info(f"成功创建节点")
        self.create_service(AddInts,"add_ints",self.call_back)
        self.get_logger().info(f"服务端创建成功")

    def call_back(self,request,response):#回调函数
        response.sum = request.num1 + request.num2
        self.get_logger().info(f"接收到的请求数据是: {request.num1}和{request.num2},响应结果是: {response.sum}")
        return response

def main():
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    rclpy.spin(AddIntsSever())

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()
