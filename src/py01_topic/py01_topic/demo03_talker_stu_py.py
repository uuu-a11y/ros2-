import rclpy
from rclpy.node import Node

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
from base_interfaces_demo.msg import Student 

class TalkerStu(Node):
    def __init__(self,):
        super().__init__("talker_stu")
        self.get_logger().info(f"成功创建节点talker_stu")
        self.publisher_ = self.create_publisher(Student, "chatter_stu", 10)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.i = 0

    def on_timer(self):#回调函数
        stu = Student()
        stu.name = "王五"
        stu.age = 18
        stu.height = 1.75
        self.publisher_.publish(stu)
        self.get_logger().info("名字: %s 年龄: %d 身高: %.2f"%(stu.name,stu.age,stu.height))
        self.i += 1

def main():
    #初始化ros2客户端
    rclpy.init()

    #调用spin函数,传入自定义类对象
    rclpy.spin(TalkerStu())

    #释放资源
    rclpy.shutdown()

if __name__ == "__main__":
    main()