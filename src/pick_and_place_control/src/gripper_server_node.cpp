#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pick_and_place_interfaces/msg/joint_control.hpp>
#include <pick_and_place_interfaces/srv/set_gripper.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <mutex>
#include <chrono>
#include <numeric>
#include <thread>

class GripperServerNode : public rclcpp::Node
{
public:
  GripperServerNode() : Node("gripper_server_node")
  {
    // 1. 파라미터 선언 및 기본값 설정
    this->declare_parameter<std::vector<std::string>>("finger_joints", std::vector<std::string>{"finger_left_joint", "finger_right_joint"});
    this->declare_parameter<double>("open_position", 0.04);
    this->declare_parameter<double>("close_position", 0.018);
    this->declare_parameter<double>("velocity", 0.05);
    this->declare_parameter<double>("tolerance", 0.001);
    this->declare_parameter<double>("default_timeout", 2.0);

    // 파라미터 읽기
    finger_joints_ = this->get_parameter("finger_joints").as_string_array();
    open_position_ = this->get_parameter("open_position").as_double();
    close_position_ = this->get_parameter("close_position").as_double();
    velocity_ = this->get_parameter("velocity").as_double();
    tolerance_ = this->get_parameter("tolerance").as_double();
    default_timeout_ = this->get_parameter("default_timeout").as_double();

    // 관절 이름 -> 인덱스 매핑 테이블 및 캐시 벡터 초기화
    for (size_t i = 0; i < finger_joints_.size(); ++i) {
      finger_joints_name_to_idx_[finger_joints_[i]] = i;
    }
    current_finger_positions_.resize(finger_joints_.size(), 0.0);

    // 2. 콜백 그룹 분리 (데드락 방지)
    cb_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 3. Publisher 생성
    gripper_joint_cmd_pub_ = this->create_publisher<pick_and_place_interfaces::msg::JointControl>(
      "/arm/joint_command", 10);

    // 4. Subscriber 생성 (독립 콜백 그룹 지정)
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_sub_;

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GripperServerNode::jointStateCallback, this, std::placeholders::_1),
      sub_options);

    // 5. Service Server 생성 (독립 콜백 그룹 지정)
    set_gripper_srv_ = this->create_service<pick_and_place_interfaces::srv::SetGripper>(
      "/gripper/set",
      std::bind(&GripperServerNode::handleSetGripper, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_service_);

    RCLCPP_INFO(this->get_logger(), "Gripper Server Node has been initialized.");
  }

private:
  // /joint_states 구독 콜백 (백그라운드에서 현재 손가락 위치 캐싱)
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = finger_joints_name_to_idx_.find(msg->name[i]);
      if (it != finger_joints_name_to_idx_.end()) {
        current_finger_positions_[it->second] = msg->position[i];
      }
    }
  }

  // /gripper/set 서비스 요청 처리 콜백
  void handleSetGripper(
    const std::shared_ptr<pick_and_place_interfaces::srv::SetGripper::Request> request,
    std::shared_ptr<pick_and_place_interfaces::srv::SetGripper::Response> response)
  {
    // 1. 요청 명령어 검증 (OPEN: 0, CLOSE: 1)
    if (request->command != pick_and_place_interfaces::srv::SetGripper::Request::OPEN &&
        request->command != pick_and_place_interfaces::srv::SetGripper::Request::CLOSE)
    {
      response->success = false;
      response->message = "Invalid command. Use OPEN(0) or CLOSE(1).";
      response->final_width = 0.0;
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // 2. 목표 위치 및 타임아웃 결정
    double target_pos = (request->command == pick_and_place_interfaces::srv::SetGripper::Request::OPEN)
                        ? open_position_ : close_position_;
    double timeout_sec = (request->timeout_sec <= 0.0) ? default_timeout_ : request->timeout_sec;

    // 3. /arm/joint_command 에 손가락 관절 이동 명령 발행
    pick_and_place_interfaces::msg::JointControl cmd_msg;
    cmd_msg.header.stamp = this->now();
    cmd_msg.joint_names = finger_joints_;
    cmd_msg.positions = std::vector<double>(finger_joints_.size(), target_pos);
    cmd_msg.max_velocity = velocity_;
    gripper_joint_cmd_pub_->publish(cmd_msg);

    // 4. 도착 판정 대기 루프 (10ms 주기)
    auto start_time = std::chrono::steady_clock::now();
    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok()) {
      // 타임아웃 검사
      auto elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      if (elapsed_sec > timeout_sec) {
        response->success = false;
        response->message = "Gripper motion timed out.";
        {
          std::lock_guard<std::mutex> lock(mutex_);
          response->final_width = std::accumulate(current_finger_positions_.begin(), current_finger_positions_.end(), 0.0);
        }
        RCLCPP_WARN(this->get_logger(), "SetGripper failed: Timeout after %.2f seconds", elapsed_sec);
        return;
      }

      // 현재 위치 복사 (스레드 안전)
      std::vector<double> current_pos;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pos = current_finger_positions_;
      }

      // 목표 위치 오차 검사
      bool reached = true;
      for (double pos : current_pos) {
        if (std::abs(pos - target_pos) > tolerance_) {
          reached = false;
          break;
        }
      }

      // 도달 완료 시 성공 응답
      if (reached) {
        response->success = true;
        response->message = "Gripper reached target position successfully.";
        response->final_width = std::accumulate(current_pos.begin(), current_pos.end(), 0.0);
        RCLCPP_INFO(this->get_logger(), "SetGripper succeeded. Final width: %.4f m", response->final_width);
        return;
      }

      loop_rate.sleep();
    }
  }

  // 파라미터 및 멤버 변수
  std::vector<std::string> finger_joints_;
  double open_position_;
  double close_position_;
  double velocity_;
  double tolerance_;
  double default_timeout_;

  std::unordered_map<std::string, size_t> finger_joints_name_to_idx_;
  std::vector<double> current_finger_positions_;
  std::mutex mutex_;

  rclcpp::CallbackGroup::SharedPtr cb_group_service_;
  rclcpp::CallbackGroup::SharedPtr cb_group_sub_;

  rclcpp::Publisher<pick_and_place_interfaces::msg::JointControl>::SharedPtr gripper_joint_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Service<pick_and_place_interfaces::srv::SetGripper>::SharedPtr set_gripper_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperServerNode>();

  // 멀티스레드 Executor 설정 (서비스 대기 중에도 구독 콜백이 실행되도록 필수)
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}