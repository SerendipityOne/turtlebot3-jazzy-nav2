#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "turtlebot3_navigation2/initial_pose_message.hpp"

namespace turtlebot3_navigation2
{
namespace
{

using namespace std::chrono_literals;

class InitialPosePublisher final : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher"), deadline_(std::chrono::steady_clock::now() + 30s)
  {
    x_ = declare_parameter<double>("initial_pose_x", -2.0);
    y_ = declare_parameter<double>("initial_pose_y", -0.5);
    yaw_ = declare_parameter<double>("initial_pose_yaw", 0.0);

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).reliable());
    get_state_client_ = create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");
    timer_ = create_wall_timer(100ms, std::bind(&InitialPosePublisher::checkReadiness, this));

    RCLCPP_INFO(
      get_logger(), "等待 AMCL 激活后初始化位姿: x=%.3f, y=%.3f, yaw=%.3f", x_, y_, yaw_);
  }

  bool failed() const
  {
    return failed_;
  }

private:
  void checkReadiness()
  {
    if (std::chrono::steady_clock::now() >= deadline_) {
      failed_ = true;
      timer_->cancel();
      RCLCPP_ERROR(get_logger(), "30 秒内 AMCL 未就绪或未订阅 /initialpose，未发布初始位姿");
      rclcpp::shutdown();
      return;
    }

    if (amcl_active_) {
      publishWhenSubscribed();
      return;
    }

    if (!get_state_client_->service_is_ready() || state_request_in_flight_) {
      return;
    }

    state_request_in_flight_ = true;
    get_state_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this](const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
        state_request_in_flight_ = false;
        if (future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          amcl_active_ = true;
          RCLCPP_INFO(get_logger(), "AMCL 已激活，等待 /initialpose 订阅者");
        }
      });
  }

  void publishWhenSubscribed()
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    auto message = makeInitialPose(x_, y_, yaw_);
    message.header.stamp = now();
    publisher_->publish(message);
    if (!publisher_->wait_for_all_acked(500ms)) {
      RCLCPP_WARN(get_logger(), "等待 /initialpose 接收确认超时，仍将退出发布器");
    }
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "已向 /initialpose 发布初始 AMCL 位姿");
    rclcpp::shutdown();
  }

  double x_;
  double y_;
  double yaw_;
  bool amcl_active_{false};
  bool failed_{false};
  bool state_request_in_flight_{false};
  std::chrono::steady_clock::time_point deadline_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace turtlebot3_navigation2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtlebot3_navigation2::InitialPosePublisher>();
  rclcpp::spin(node);
  return node->failed() ? 1 : 0;
}
