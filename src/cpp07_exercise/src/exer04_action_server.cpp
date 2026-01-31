#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"


/*
   需求：处理请求发送的目标点，控制乌龟向该目标点运动，并连续反馈乌龟与目标点之间的剩余距离。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建原生乌龟位姿订阅方，回调函数中获取乌龟位姿；
            3-2.创建原生乌龟速度发布方；
            3-3.创建动作服务端；
            3-4.解析动作客户端发送的请求；
            3-5.处理动作客户端发送的取消请求；
            3-6.创建新线程处理请求；
            3-7.新线程产生连续反馈并响应最终结果。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/

using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间
using base_interfaces_demo::action::Nav;


class Exer04ActionServer :public rclcpp::Node{
public:
    Exer04ActionServer():Node("exer04_action_server_node_cpp"),x(0.0),y(0.0){
        RCLCPP_INFO(this->get_logger(),"动作服务器节点创建成功");
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",10,std::bind(&Exer04ActionServer::pose_cb,this,_1)
        );
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        action_server_ = rclcpp_action::create_server<Nav>(
            this,
            "nav",
            std::bind(&Exer04ActionServer::handle_goal,this,_1,_2),
            std::bind(&Exer04ActionServer::handle_cancel,this,_1),
            std::bind(&Exer04ActionServer::handle_accepted,this,_1)
        );



    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<Nav>::SharedPtr action_server_;
    float x,y;
    //回调函数
    void pose_cb(const turtlesim::msg::Pose::SharedPtr pose){
        x = pose->x;
        y = pose->y;
    }
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Nav::Goal> goal){
        (void)uuid;
        // (void)goal;
        if(goal->goal_x < 0 || goal->goal_x > 11.08 || goal->goal_y < 0 || goal->goal_y > 11.08){
            RCLCPP_INFO(this->get_logger(),"目标点超出范围");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),"目标点合法");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"取消请求接受");
        return rclcpp_action::CancelResponse::ACCEPT;

    }
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        RCLCPP_INFO(this->get_logger(),"执行线程开始");
        rclcpp::Rate rate(1.0);
        auto result = std::make_shared<Nav::Result>();
        auto feedback = std::make_shared<Nav::Feedback>();
        geometry_msgs::msg::Twist twist;
        while (true){
            if (goal_handle->is_canceling()){
                goal_handle->canceled(result);
                return;
            }
            float goal_x = goal_handle->get_goal()->goal_x;
            float goal_y = goal_handle->get_goal()->goal_y;
            float distance_x = goal_x - x;
            float distance_y = goal_y - y;
            float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
            feedback->distance = distance;
            goal_handle->publish_feedback(feedback);

            float scale = 0.5;
            float linear_x = scale * distance_x;
            float linear_y = scale * distance_y;
            twist.linear.x = linear_x;
            twist.linear.y = linear_y;
            cmd_pub_->publish(twist);

            if (distance <= 0.05){
                RCLCPP_INFO(this->get_logger(),"到达目标点");
                break;
            }

            rate.sleep();
        }

        if (rclcpp::ok()){
            result->turtle_x = x;
            result->turtle_y = y;
            goal_handle->succeed(result);
        }
    }
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        // (void)goal_handle;
        std::thread(std::bind(&Exer04ActionServer::execute,this,goal_handle)).detach();

    }
};

int main(int argc, char * argv[])
{
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    rclcpp::spin(std::make_shared<Exer04ActionServer>());//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}