#include "rclcpp/rclcpp.hpp"
#


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
 //时间命名空间

class Exer06Param :public rclcpp::Node{
public:
    Exer06Param():Node("exer06_param_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"参数客户端");
        client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"/turtlesim");

    }
    bool connect_server(){
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"等待服务端超时,请检查服务端是否启动");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"服务端未启动,等待中...");
        }
        RCLCPP_INFO(this->get_logger(),"服务端启动成功");
        return true;
    }
    void update_param(){
        int red = client_->get_parameter<int>("background_r");

        rclcpp::Rate rate(30.0);
        int count = red;
        while (rclcpp::ok()){
            count <= 255 ? red +=5 : red -=5;
            count +=5;
            if(count > 511) count = 0; 
            client_->set_parameters({rclcpp::Parameter("background_r",red)});

            rate.sleep();
        }
    }

private:

    rclcpp::SyncParametersClient::SharedPtr client_;

};


int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);    
    auto client = std::make_shared<Exer06Param>();
    if(!client->connect_server()){
        return 1;
    }
    client->update_param();
    

    //释放资源
    rclcpp::shutdown();
    return 0;
}