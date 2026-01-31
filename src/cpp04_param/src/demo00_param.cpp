#include "rclcpp/rclcpp.hpp"


/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；(获取键，值，)
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class MyParam :public rclcpp::Node{
public:
    MyParam():Node("my_param_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"参数API使用");
        // 3-1.声明参数；
        rclcpp::Parameter p1("car_name","tiger");
        rclcpp::Parameter p2("height",1.68);
        rclcpp::Parameter p3("wheels",4);
        // 3-2.查询参数；
        RCLCPP_INFO(this->get_logger(), "car_name: %s",p1.as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "height: %.2f",p2.as_double());
        RCLCPP_INFO(this->get_logger(), "wheels: %ld",p3.as_int());

        RCLCPP_INFO(this->get_logger(), "name: %s",p1.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "type: %s",p1.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "value2string: %s",p2.value_to_string().c_str());
        
    }

private:

    //回调函数
    void callback_this(){
    }
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<MyParam>());//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}