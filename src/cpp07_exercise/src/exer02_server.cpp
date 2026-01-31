#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp"




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
using base_interfaces_demo::srv::Distance;

class Exer02Server :public rclcpp::Node{
public:
    Exer02Server():Node("exer02_server_node_cpp"),x(0.0),y(0.0){
        RCLCPP_INFO(this->get_logger(),"exer02_server_node_cpp 节点创建成功");
        sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10,
            std::bind(&Exer02Server::pose_cb,this,_1)
        );
        server_ = this->create_service<Distance>("distance",std::bind(&Exer02Server::distance_cb,this,_1,_2));

    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Service<base_interfaces_demo::srv::Distance>::SharedPtr server_;
    float x,y;
    void pose_cb(const turtlesim::msg::Pose::SharedPtr pose){
        x = pose->x;
        y = pose->y;
    }
    void distance_cb(const Distance::Request::SharedPtr request, Distance::Response::SharedPtr response){
        float goal_x = request->x;
        float goal_y = request->y;

        float distance_x = goal_x - x;
        float distance_y = goal_y - y;
        float distance = sqrt(distance_x*distance_x + distance_y*distance_y);
        response->distance = distance;
        RCLCPP_INFO(this->get_logger(),"原生乌龟位置:(%.2f,%.2f),新乌龟位置:(%.2f,%.2f),距离:%.2f",x,y,goal_x,goal_y,distance);
    }
};


int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);
    // 4.调用spin函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<Exer02Server>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}