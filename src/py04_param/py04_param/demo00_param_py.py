import rclpy
from rclpy.node import Node


"""  
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象，调用参数操作函数，并传递给spin函数；
        5.释放资源。

"""


class MyParam(Node):
    def __init__(self):
        super().__init__("my_parameter_node_py")
        self.get_logger().info("参数API使用")
        # 3-1.声明参数；
        p1 = rclpy.Parameter("car_name", value="Tiger")
        p2 = rclpy.Parameter("widht", value=1.5)
        p3 = rclpy.Parameter("wheels", value=2)
        # 3-2.解析参数
        self.get_logger().info("car_name = %s" % p1.value)
        self.get_logger().info("widht = %.2f" % p2.value)
        self.get_logger().info("wheels = %ld" % p3.value)


        self.get_logger().info("key = %s" % p1.name)



def main():
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    rclpy.spin(MyParam())

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()
