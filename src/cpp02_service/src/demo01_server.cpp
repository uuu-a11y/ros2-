#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"
using base_interfaces_demo::srv::AddInts;


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
using std::placeholders::_1;
using std::placeholders::_2;

class AddIntServer :public rclcpp::Node{
public:
    AddIntServer():Node("AddIntServer_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"AddIntServer节点创建成功");
        service = this->create_service<AddInts>("add_ints",std::bind(&AddIntServer::add,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"---------------服务已创建-------------");
    }

private:
    rclcpp::Service<AddInts>::SharedPtr service;

    //回调函数
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res){
        res->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(),"服务端接收到请求,%d + %d = %d",req->num1,req->num2,res->sum);
    }

};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<AddIntServer>());//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}