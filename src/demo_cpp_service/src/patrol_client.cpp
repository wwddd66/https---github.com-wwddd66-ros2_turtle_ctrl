#include "interfaces/srv/patrol.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ctime>
using namespace std::chrono_literals;
using Patrol = interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node {
  private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Client<Patrol>::SharedPtr _patrol_client;

  public:
    PatrolClient() : Node("sa") {
        srand(time(NULL));
        _patrol_client = this->create_client<Patrol>("patrol");
        _timer = this->create_wall_timer(10s, [&]() -> void {
            /*检测服务端是否上线*/
            while (!this->_patrol_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "等待服务上线过程中，rclcpp挂了");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "等待服务上线中...");
            }

            /*构造请求对象*/
            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand() % 15;
            request->target_y = rand() % 15;
            RCLCPP_INFO(this->get_logger(), "准备好目标坐标 (%f,%f)", request->target_x, request->target_y);

            /*发送请求*/
            this->_patrol_client->async_send_request(
                request, [&](rclcpp::Client<Patrol>::SharedFuture result_sf) -> void {
                    auto response = result_sf.get();
                    if (response->result == Patrol::Response::SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "请求巡逻目标成功👻");
                    } else if (response->result == Patrol::Response::FAIL) {
                        RCLCPP_INFO(this->get_logger(), "请求巡逻目标失败🪖");
                    }
                });
        });
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}