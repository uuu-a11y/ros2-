#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

/*
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
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间
using base_interfaces_demo::msg::Student;

class ListenerStu :public rclcpp::Node{
public:
    ListenerStu(std::string str1):Node(str1){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功",str1.c_str());
        subscription_ = this->create_subscription<Student>("chatter_stu",10,std::bind(&ListenerStu::do_cb,this,_1));

    }

private:

    //回调函数
    void do_cb(const Student &stu){
        RCLCPP_INFO(this->get_logger(), "接受到的学生信息是: %s %d %.2f", stu.name.c_str(), stu.age, stu.height);

    }
    rclcpp::Subscription<Student>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<ListenerStu>("ListenerStu_node_cpp"));//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}