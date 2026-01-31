import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student
"""
    需求:
    流程:
        1.包含头文件
        2.初始化ros2客户端
        3.自定义节点类
            3-1
            3-2
            3-3
        4.调用spin函数,并传入节点对象指针
        5.释放资源
"""

class ListenerStu(Node):
    def __init__(self):
        super().__init__("listener_stu")
        self.get_logger().info(f"成功创建节点listener_stu")
        self.subscription = self.create_subscription(Student,"chatter_stu",self.do_cb,10)

    def do_cb(self, stu):#回调函数
        
        self.get_logger().info("名字: %s" % stu.name + " 年龄: %d" % stu.age + " 身高: %.2f" % stu.height)

def main():
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    rclpy.spin(ListenerStu())

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()