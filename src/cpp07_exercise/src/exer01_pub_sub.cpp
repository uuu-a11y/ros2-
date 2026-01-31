#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"


/*
   需求：订阅窗口1中的乌龟速度，然后生成控制窗口2乌龟运动的指令并发布。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
          3-1.创建控制第二个窗体乌龟运动的发布方；
          3-2.创建订阅第一个窗体乌龟pose的订阅方；
          3-3.根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class Exer01PubSub :public rclcpp::Node{
public:
    Exer01PubSub():Node("exer01_pub_sub_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"案例1对象创建");
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel",10);
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",10,
            std::bind(&Exer01PubSub::pose_cb,this,_1)
        );
        
    }

private:
    void pose_cb(const turtlesim::msg::Pose & pose){
        geometry_msgs::msg::Twist twist;
        twist.linear.x = pose.linear_velocity;
        twist.angular.z = -pose.angular_velocity;
        pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<Exer01PubSub>());//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}
