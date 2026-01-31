#include "rclcpp/rclcpp.hpp"


/*
    需求：编写参数客户端，获取或修改服务端参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.查询参数；
            3-2.修改参数；
        4.创建节点对象指针，调用参数操作函数；
        5.释放资源。
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class ParamClient :public rclcpp::Node{
public:
    ParamClient():Node("paramclient_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"paramclient_node_cpp 节点创建成功");

        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "paramserver_node_cpp");

    }

    bool connect_server(){
        while (!param_client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "客户端被取消");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端响应...");
        }
        
        return true;
    }
    // 查询
    void get_param(){
        RCLCPP_INFO(this->get_logger(), "-----------------参数查询操作-----------------");
        // 获取某个参数
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        double width = param_client_->get_parameter<double>("width");
        RCLCPP_INFO(this->get_logger(), "car_name: %s", car_name.c_str());
        RCLCPP_INFO(this->get_logger(), "width: %.2f", width);
        // 获取多个参数
        auto params = param_client_->get_parameters({"car_name", "width","wheels", "height"});
        for (auto &&param : params){
            RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(), param.value_to_string().c_str());
        }

        // 判断是否包含某个参数
        RCLCPP_INFO(this->get_logger(), "包含car_name吗：%d", param_client_->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "包含width吗：%d", param_client_->has_parameter("width"));
        RCLCPP_INFO(this->get_logger(), "包含height吗：%d", param_client_->has_parameter("height"));

    }

    // 修改
    void update_param(){
        RCLCPP_INFO(this->get_logger(), "-----------------参数修改操作-----------------");
        param_client_->set_parameters({
            rclcpp::Parameter("car_name", "turtle"), 
            rclcpp::Parameter("width", 0.5),
            rclcpp::Parameter("height", 1.2)
        });
        
    }

private:

    rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);
    auto client = std::make_shared<ParamClient>();
    bool flag = client->connect_server();
    if(!flag){
        RCLCPP_ERROR(client->get_logger(), "服务端连接失败");
        return 0;
    }
    client->get_param();
    client->update_param();
    client->get_param();

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(client);//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}