#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pick_and_place_interfaces/msg/joint_control.hpp"

using namespace std::chrono_literals;

class JointCommandTestPub : public rclcpp::Node
{
public:
  JointCommandTestPub(): Node("joint_command_test_publisher")
  {
    publisher_ = this->create_publisher<pick_and_place_interfaces::msg::JointControl>(
      "/arm/joint_command", 10);

    timer_ = this->create_wall_timer(
      1s, std::bind(&JointCommandTestPub::publish_command, this));

    RCLCPP_INFO(this->get_logger(), "JointCommandTestPub node initialized.");
  }

private:
  void publish_command()
  {
    auto msg = pick_and_place_interfaces::msg::JointControl();
    msg.header.stamp = this->now();     //현재 시각의 타임스탬프(시간)을 추출해 저장. (sec, nanosec)
    msg.joint_names = {"joint2"};
    msg.positions = {0.5};
    msg.max_velocity = 0.5;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published JointControl command for joint2");
  }

  rclcpp::Publisher<pick_and_place_interfaces::msg::JointControl>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandTestPub>());
  rclcpp::shutdown();
  return 0;
}