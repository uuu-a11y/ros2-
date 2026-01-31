/*
  需求： 以某个固定频率发送文本“hello world”，文本后缀编号，发布每一条，编号加一
  流程：
  1.包含头文件
  2.初始化ROS节点
  3自定义节点类
    3-1.创建消息发布方；
    3-2.创建定时器；
    3-3.回调函数中组织数据并发布；
  4.调用spin函数，传入自定义类对象指针；
  5释放资源
*/



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker_node_cpp") ,count(0) {
    RCLCPP_INFO(this->get_logger(), "发布节点已创建" );
    // 3-1.创建消息发布方；
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    // 3-2.创建定时器；
    timer_ = this->create_wall_timer(1s, std::bind(&Talker::on_timer, this));
    // 3-3.回调函数中组织数据并发布；
  }
private:
  void on_timer() {
    auto message = std_msgs::msg::String();
    message.data = "hello world!" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(),"发布方的消息：%s", message.data.c_str());
    publisher_->publish(message);
    
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;

};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
}
