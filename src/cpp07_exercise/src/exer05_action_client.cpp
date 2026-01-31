#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"


/*
   需求：向动作服务端发送目标点数据，并处理服务端的响应数据。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求数据，并处理服务端响应；
            3-3.处理目标响应；
            3-4.处理响应的连续反馈；
            3-5.处理最终响应。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间
using base_interfaces_demo::action::Nav;
using namespace std::chrono_literals; //使用时间命名空间

class Exer05ActionClient :public rclcpp::Node{
public:
    Exer05ActionClient():Node("exer05_action_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"动作客户端节点创建成功");
        client_ = rclcpp_action::create_client<Nav>(this,"nav");
    }
    void send_goal(float x,float y,float theta){
        if(!client_->wait_for_action_server(10s)){
            RCLCPP_ERROR(this->get_logger(),"动作服务端未启动");
            return;
        }
        Nav::Goal goal;
        goal.goal_x = x;
        goal.goal_y = y;
        goal.goal_theta = theta;
        rclcpp_action::Client<Nav>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&Exer05ActionClient::goal_response_callback,this,_1);
        options.feedback_callback = std::bind(&Exer05ActionClient::feedback_callback,this,_1,_2);
        options.result_callback = std::bind(&Exer05ActionClient::result_callback,this,_1);
        client_->async_send_goal(goal,options);

    }
    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>> goal_handle){
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "目标请求被服务端拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端接受");
        }
    }
    void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>>goal_handle, std::shared_ptr<const Nav::Feedback> feedback){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "剩余%.2f米", feedback->distance);


    }
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult & result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(this->get_logger(), "乌龟的最终位姿信息，坐标：(%.2f,%.2f),航向:%.2f",result.result->turtle_x,result.result->turtle_y,result.result->turtle_theta);

        } else {
            RCLCPP_ERROR(this->get_logger(), "目标执行失败");
        }


    }

private:
    rclcpp_action::Client<Nav>::SharedPtr client_;


};

int main(int argc, char * argv[])
{
    if (argc != 5){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"参数个数错误");
        return 1;
    }
    //初始化ros2客户端
    rclcpp::init(argc,argv);
    auto client = std::make_shared<Exer05ActionClient>();
    client->send_goal(atof(argv[1]),atof(argv[2]),atof(argv[3]));


    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(client);//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}