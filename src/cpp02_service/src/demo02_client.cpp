#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"
using base_interfaces_demo::srv::AddInts;


//   需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
//   步骤：
//     1.包含头文件；
//     2.初始化 ROS2 客户端；
//     3.定义节点类；
//       3-1.创建客户端；
//       3-2.等待服务连接；
//       3-3.组织请求数据并发送；
//     4.创建对象指针调用其功能,并处理响应；
//     5.释放资源。

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class AddIntsClient :public rclcpp::Node{
public:
    AddIntsClient():Node("AddIntsClient_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"namesapce:  node: %s 节点创建成功","AddIntsClient_node_cpp");
        client = this->create_client<AddInts>("add_ints");
        RCLCPP_INFO(this->get_logger(),"client 创建成功");

    }
    bool connect_server(){
        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强制退出，节点已取消");
                return false;
            }

            RCLCPP_INFO(this->get_logger(),"等待服务连接...");
        }
        return true;
    }

    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int32_t num1, int32_t num2){
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client->async_send_request(request);
    }

private:
    rclcpp::Client<AddInts>::SharedPtr client;
};

int main(int argc, char * argv[])
{
    if(argc != 3){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"参数错误，请检查");
        return 1;
    }
    //初始化ros2客户端
    rclcpp::init(argc,argv);
    auto client = std::make_shared<AddIntsClient>();
    bool flag = client->connect_server();
    if(!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接失败，请检查");
        return 0;
    }
    auto response = client->send_request(std::stoi(argv[1]), std::stoi(argv[2]));

    if (rclcpp::spin_until_future_complete(client, response) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(), "响应正常处理");
        RCLCPP_INFO(client->get_logger(), "响应结果：%d", response.get()->sum);
    }else{
        RCLCPP_INFO(client->get_logger(), "响应异常");
    }

    
    

    //释放资源
    rclcpp::shutdown();
    return 0;
}