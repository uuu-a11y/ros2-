#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"
#


/*
    需求:
    流程:
        1.包含头文件
        2.初始化ros2客户端
        3.自定义节点类
            3-1.创建发布方
            3-2。创建定时器
            3-3。组织并发布学生消息
        4.调用spin函数,并传入节点对象指针
        5.释放资源
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间
using base_interfaces_demo::msg::Student;


class TalkerStu :public rclcpp::Node{
public:
    TalkerStu(std::string str1):Node(str1){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功",str1.c_str());
        publisher_ = this->create_publisher<Student>("chatter_stu",10);
        timer_ = this->create_wall_timer(500ms, std::bind(&TalkerStu::on_timer, this));

    }

private:

    //回调函数
    void on_timer(){
        auto stu = Student();
        stu.name = "张三";
        stu.age = age;
        stu.height = 1.75;
        age++;
        publisher_->publish(stu);
        RCLCPP_INFO(this->get_logger(), "发布消息: %s %d %.2f", stu.name.c_str(), stu.age, stu.height);

    }
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int age;
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<TalkerStu>("TalkerStu_node_cpp"));//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}