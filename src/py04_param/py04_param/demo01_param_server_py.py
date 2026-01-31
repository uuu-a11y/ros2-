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

class ParamServer(Node):
    def __init__(self):
        # 如果允许删除参数，则需要设置参数选项 allow_undeclared_parameters=True
        super().__init__("param_server_node_py", allow_undeclared_parameters=True)
        self.get_logger().info(f"参数服务端！")


        # 3-1.声明参数；
    def declare_param(self):
        self.get_logger().info("-----------新增参数------------")
        self.declare_parameter("car_name", "tiger")
        self.declare_parameter("width", 1.55)
        self.declare_parameter("wheels", 5)

        self.set_parameters([rclpy.Parameter("haha", value="xixi")])

        # 3-2.查询参数；
    def get_param(self):
        self.get_logger().info("-----------查询参数------------")
        car_name = self.get_parameter("car_name")
        self.get_logger().info("%s = %s" % (car_name.name, car_name.value))
        params = self.get_parameters(["car_name", "width", "wheels"])
        for param in params:
            self.get_logger().info("%s = %s" % (param.name, param.value))

        self.get_logger().info("包含 car_name吗？ %s" % self.has_parameter("car_name"))
        self.get_logger().info("包含 height 吗？ %s" % self.has_parameter("height"))
        # 3-3.修改参数；
    def update_param(self):
        self.get_logger().info("-----------修改参数------------")
        self.set_parameters([rclpy.Parameter("car_name",value="Mouse")])
        car_name = self.get_parameter("car_name")
        self.get_logger().info("%s = %s" % (car_name.name, car_name.value))
        # 3-4.删除参数。
    def delete_param(self):
        self.get_logger().info("-----------删除参数------------")
        self.get_logger().info("包含 car_name吗？ %s" % self.has_parameter("car_name"))
        self.undeclare_parameter("car_name")
        self.get_logger().info("包含 car_name吗？ %s" % self.has_parameter("car_name"))



def main():
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    node = ParamServer()
    node.declare_param()
    node.get_param()
    node.update_param()
    node.delete_param()
    rclpy.spin(node)

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()