#include "rclcpp/rclcpp.hpp"


/*
"""  
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象，调用参数操作函数，并传递给spin函数；
        5.释放资源。

"""
*/


using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class ParamServer :public rclcpp::Node{
public:
    // 
    ParamServer():Node("paramserver_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)){
        RCLCPP_INFO(this->get_logger(),"paramserver_node_cpp 节点创建成功");

    }
    // 增
    void declare_param(){
        RCLCPP_INFO(this->get_logger(), "---------------增-----------------");
        this->declare_parameter("car_name", "tiger");
        this->declare_parameter("width", 1.55);
        this->declare_parameter("wheels", 5);


        this->set_parameter(rclcpp::Parameter("height", 1.55));

    }
    // 查
    void get_param(){
        RCLCPP_INFO(this->get_logger(), "---------------查-----------------");
        auto car = this->get_parameter("car_name");
        // 获取指定
        RCLCPP_INFO(this->get_logger(), "key = %s, value = %s", car.get_name().c_str(), car.as_string().c_str());
        // 获取一些
        auto params = this->get_parameters({"car_name", "width","wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "key = %s, value = %s", param.get_name().c_str(), param.value_to_string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "是否包含car_name? %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否包含width? %d", this->has_parameter("width"));
        RCLCPP_INFO(this->get_logger(), "是否包含wheels? %d", this->has_parameter("wheels"));

    }
    // 改
    void update_param(){
        RCLCPP_INFO(this->get_logger(), "---------------改-----------------");
        this->set_parameter(rclcpp::Parameter("width", 2.0));
        RCLCPP_INFO(this->get_logger(), "width: %.2f", this->get_parameter("width").as_double());
    }
    // 删
    void delete_param(){
        RCLCPP_INFO(this->get_logger(), "---------------删-----------------");
        this->undeclare_parameter("height");
        RCLCPP_INFO(this->get_logger(), "是否包含height? %d", this->has_parameter("height"));

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
    auto node = std::make_shared<ParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->delete_param();
    rclcpp::spin(node);//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}