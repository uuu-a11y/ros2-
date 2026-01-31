#include "rclcpp/rclcpp.hpp"
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





class Exer03Client :public rclcpp::Node{
public:
    Exer03Client():Node("exer03_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"exer03_client_node_cpp 节点创建成功");
        client_ = this->create_client<Distance>("distance");

    }
    bool connect_server(){
        while (!client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"等待服务端上线,但节点已被关闭");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"等待服务端上线...");
        }
        return true;
    }
    rclcpp::Client<Distance>::FutureAndRequestId send_goal(float x, float y, float theta){
        auto request = std::make_shared<Distance::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<Distance>::SharedPtr client_;


};

int main(int argc, char * argv[])
{   if (argc !=5){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请输入3个参数: x y theta");
    return 1;
}
    float goal_x = atof(argv[1]);
    float goal_y = atof(argv[2]);
    float goal_theta = atof(argv[3]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"输入参数: x=%.2f, y=%.2f, theta=%.2f",goal_x,goal_y,goal_theta);  
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    auto client = std::make_shared<Exer03Client>();
    bool flag = client->connect_server();
    if (!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务端未上线");
        return 1;
    }

    auto future = client->send_goal(goal_x,goal_y,goal_theta);
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(),"服务端响应成功");
        auto response = future.get();
        RCLCPP_INFO(client->get_logger(),"服务端响应距离: %.2f",response->distance);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务端响应失败");
    }





    //释放资源
    rclcpp::shutdown();
    return 0;
}