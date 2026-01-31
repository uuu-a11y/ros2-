/*
    "需求：订阅发布方的消息，并在终端输出"
    流程：
    1.包含头文件；
    2.初始化ROS节点；
    3.自定义节点类；
        3-1.创建订阅者对象；
        3-2.解析并输出函数；
    4.调用spin函数，并传入节点对象函数指针；
    5.释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class Listener : public rclcpp::Node{
public:
    Listener():Node("listener_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "订阅方创建！");
        // 3-1.创建订阅者对象；
        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter",10,std::bind(&Listener::do_cb,this,std::placeholders::_1));

    }
private:
    void do_cb(const std_msgs::msg::String &msg) {
         // 3-2.解析并输出函数；
        RCLCPP_INFO(this->get_logger(), "订阅到的消息：%s", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;

}

