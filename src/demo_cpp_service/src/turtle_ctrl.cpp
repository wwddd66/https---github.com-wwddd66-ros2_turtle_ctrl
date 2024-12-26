#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/patrol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
using namespace std::chrono_literals;
using Patrol = interfaces::srv::Patrol;

class TurtleCtrlNode : public rclcpp::Node {
  private:
    rclcpp::Service<Patrol>::SharedPtr _patrol_service;
    rclcpp::TimerBase::SharedPtr _timer;
    /*创建发布者的智能指针*/
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    /*订阅者的智能共享指针*/
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _subscriber;

    double _target_x{1.0};
    double _target_y{1.0};
    double _k{1.0};         /*比例系数*/
    double _max_speed{3.0}; /*最大速度*/

  public:
    /*explicit是为了防止类对象通过隐式转换方式构造*/
    explicit TurtleCtrlNode(const std::string &node_msg) : Node(node_msg) {
        _patrol_service = this->create_service<Patrol>(
            "patrol", [&](const Patrol::Request::SharedPtr request, Patrol::Response::SharedPtr reponse) -> void {
                if ((0 < request->target_x && request->target_x < 12.0f) &&
                    (0 < request->target_y && request->target_y < 12.0f)
                ) {
                    this->_target_x = request->target_x;
                    this->_target_y = request->target_y;
                    reponse->result = Patrol::Response::SUCCESS;
                } else {
                    reponse->result = Patrol::Response::FAIL;
                }
            });
        _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        // _timer = this->create_wall_timer(1000ms, std::bind(&TurtleCtrlNode::timer_callback, this));
        _subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleCtrlNode::on_pose_recved_callback, this, std::placeholders::_1));
    }

    // void timer_callback() {
    //     auto msg = geometry_msgs::msg::Twist();
    //     msg.linear.x = 1.0;
    //     msg.angular.z = 0.5;
    //     _publisher->publish(msg);
    // }

    void on_pose_recved_callback(const turtlesim::msg::Pose::SharedPtr pose_ptr) {
        /*获取龟龟当前位置*/
        auto current_x = pose_ptr->x;
        auto current_y = pose_ptr->y;
        RCLCPP_INFO(get_logger(), "当前坐标(x,y)=(%f, %f)", current_x, current_y);

        /*计算当前位置和目标位置距离差和角度差*/
        auto distance = std::sqrt((_target_x - current_x) * (_target_x - current_x) +
                                  (_target_y - current_y) * (_target_y - current_y));
        auto angle = std::atan2((_target_y - current_y), (_target_x - current_x)) - pose_ptr->theta;

        /*控制策略*/
        auto msg = geometry_msgs::msg::Twist();
        if (distance > 0.1) {
            if (fabs(angle) > 0.2) {
                msg.angular.z = fabs(angle);
            } else {
                msg.linear.x = _k * distance;
            }
        }

        /*限制最大线速度*/
        if (msg.linear.x > _max_speed) {
            msg.linear.x = _max_speed;
        }

        /*把设置好的消息发出去*/
        _publisher->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleCtrlNode>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}