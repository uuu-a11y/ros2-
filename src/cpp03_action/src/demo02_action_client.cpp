#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"



/*
    需求:
    流程: 可以解析终端下动态传入的参数
        前提：
        1.包含头文件
        2.初始化ros2客户端
        3.自定义节点类
            3-1 创建动作客户端
            3-2 发送请求
            3-3 处理关羽目标值的服务器响应（回调函数）
            3-4 处理连续反馈（回调函数）
            3-5 处理最终响应（回调函数）
        4.调用spin函数,并传入节点对象指针
        5.释放资源
*/
using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals; //使用时间命名空间
using namespace std::placeholders; //占位符命名空间

class ProgressActionClient :public rclcpp::Node{
public:
    ProgressActionClient():Node("minimal_action_server"){
        RCLCPP_INFO(this->get_logger(),"客户端节点创建成功");
        // 3-1 创建动作客户端
        /*
        rclcpp_action::Client<ActionT>::SharedPtr 
        create_client<ActionT, NodeT>(NodeT node, 
        const std::string &name, 
        rclcpp::CallbackGroup::SharedPtr group = nullptr, 
        const rcl_action_client_options_t &options = rcl_action_client_get_default_options())
        */
        client_ = rclcpp_action::create_client<Progress>(this, "get_sum");
    }
        // 3-2 发送请求
    void send_goal(int num){
        if(!client_->wait_for_action_server(10s)){
            RCLCPP_INFO(this->get_logger(),"服务连接失败");
            return;
        }
        /*
        std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr> 
        async_send_goal(const base_interfaces_demo::action::Progress::Goal &goal, 
        const rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions &options)
        
        */
        auto goal = Progress::Goal();
        goal.num = num;
        rclcpp_action::Client<Progress>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&ProgressActionClient::result_callback, this, _1);
        // 2.发送具体请求
        auto future = client_->async_send_goal(goal, options);

    }
/*
// 3-3 处理关羽目标值的服务器响应（回调函数）
  using GoalHandle = ClientGoalHandle<ActionT>;
  using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
*/
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
        if (!goal_handle){
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端接受");
        }

    }

    
        // 3-4 处理连续反馈（回调函数）
        /*
        std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
        
        */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback){
        (void)goal_handle;
        double progress = feedback->progress;
        int pro = (int)(progress * 100);
        RCLCPP_INFO(this->get_logger(), "当前进度：%d%%", pro);

        
    }
        // 3-5 处理最终响应（回调函数）
        /*
        std::function<void (const WrappedResult & result)>
        */
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "最终结果：%d", result.result->sum);
        } else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端中止");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(this->get_logger(), "目标请求被服务端取消");
        } else {
            RCLCPP_INFO(this->get_logger(), "未知异常");
        }

    }

private:

    rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    if(argc !=2 ){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"参数个数错误");
        return 1;
    }
    //初始化ros2客户端
    rclcpp::init(argc,argv);

    //调用spin函数,使用自定义类对象指针
    auto node = std::make_shared<ProgressActionClient>();
    node->send_goal(atoi(argv[1]));
    rclcpp::spin(node);//node_name, (namespace可选)

    //释放资源
    rclcpp::shutdown();
    return 0;
}