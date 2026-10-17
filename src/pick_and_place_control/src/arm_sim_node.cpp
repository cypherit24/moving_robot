#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pick_and_place_interfaces/msg/joint_control.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <algorithm>

class ArmSimNode : public rclcpp::Node
{
public:
    ArmSimNode() : Node("arm_sim_node")
    {
        // 1. 파라미터 선언 (기본값 타입 명시)
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
        this->declare_parameter<std::vector<double>>("lower_limits", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("upper_limits", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("initial_positions", std::vector<double>{});
        this->declare_parameter<double>("default_velocity", 1.0);
        this->declare_parameter<double>("publish_rate_hz", 50.0);

        // 2. 파라미터 가져오기
        std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
        std::vector<double> lower_limits = this->get_parameter("lower_limits").as_double_array();
        std::vector<double> upper_limits = this->get_parameter("upper_limits").as_double_array();
        std::vector<double> initial_positions = this->get_parameter("initial_positions").as_double_array();

        double default_velocity = this->get_parameter("default_velocity").as_double();
        double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

        const size_t num_joints = joint_names.size();

        // 3. 배열 크기 검증
        if (num_joints != lower_limits.size() || 
            num_joints != upper_limits.size() || 
            num_joints != initial_positions.size())
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "파라미터 배열 길이 불일치: joint_names: %zu, lower_limits: %zu, "
                "upper_limits: %zu, initial_positions: %zu",
                num_joints, lower_limits.size(), upper_limits.size(), initial_positions.size()
            );
            throw std::runtime_error("arm_sim_node 파라미터 배열 크기 검증 실패");
        }

        // 4. 멤버 변수에 복사 보관
        joint_names_ = joint_names;
        lower_limits_ = lower_limits;
        upper_limits_ = upper_limits;
        default_velocity_ = default_velocity;

        current_positions_ = initial_positions;
        target_positions_ = initial_positions;
        target_velocities_ = std::vector<double>(num_joints, default_velocity);        

        for (size_t i = 0; i < num_joints; i++)
        {
            joint_name_to_index_[joint_names[i]] = i;
        }

        // 5. 통신 및 타이머 생성
        joint_cmd_sub_ = this->create_subscription<pick_and_place_interfaces::msg::JointControl>(
            "/arm/joint_command", 10,
            std::bind(&ArmSimNode::joint_cmd_callback, this, std::placeholders::_1)
        );

        last_update_time_ = this->now();

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&ArmSimNode::timer_callback, this)
        );
    }

private:
    void joint_cmd_callback(const pick_and_place_interfaces::msg::JointControl::SharedPtr msg)
    {
        if (msg->joint_names.size() != msg->positions.size()) {
            RCLCPP_WARN(this->get_logger(), "관절 이름과 포지션 개수 불일치");
            return;
        }

        for (double pos : msg->positions) {
            if (!std::isfinite(pos)) {
                RCLCPP_WARN(this->get_logger(), "position 원소가 수치적으로 유효하지 않음(NaN/Inf)");
                return;
            }
        }

        for (const auto & name : msg->joint_names) {
            if (joint_name_to_index_.find(name) == joint_name_to_index_.end()) {
                RCLCPP_WARN(this->get_logger(), "알 수 없는 관절 이름 수신: %s", name.c_str());
                return;
            }
        }

        for (size_t i = 0; i < msg->joint_names.size(); i++) {
            size_t idx = joint_name_to_index_[msg->joint_names[i]];
            double clamped_pos = std::clamp(msg->positions[i], lower_limits_[idx], upper_limits_[idx]);
            
            if (clamped_pos != msg->positions[i]) {
                RCLCPP_WARN(this->get_logger(), "관절 %s 의 position 범위 초과 -> Clamp 처리: %f", 
                            msg->joint_names[i].c_str(), clamped_pos);
            }
            
            target_positions_[idx] = clamped_pos;

            if (msg->max_velocity > 0.0) {
                target_velocities_[idx] = msg->max_velocity;
            } else {
                target_velocities_[idx] = default_velocity_;
            }
        }
    }

    void timer_callback()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        for (size_t i = 0; i < current_positions_.size(); i++) {
            double diff = target_positions_[i] - current_positions_[i];
            double max_step = target_velocities_[i] * dt;

            if (std::abs(diff) <= max_step) {
                current_positions_[i] = target_positions_[i];
            } else {
                if (diff > 0.0) {
                    current_positions_[i] += max_step;
                } else {
                    current_positions_[i] -= max_step;
                }
            }
        }

        // 메시지 내보내기 (timer_callback 내부)
        sensor_msgs::msg::JointState state_msg;
        state_msg.header.stamp = current_time;
        state_msg.name = joint_names_;
        state_msg.position = current_positions_;

        joint_state_pub_->publish(state_msg);
    }

    // 멤버 변수
    std::unordered_map<std::string, size_t> joint_name_to_index_;
    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> target_positions_;
    std::vector<double> target_velocities_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    double default_velocity_;
    rclcpp::Time last_update_time_;

    rclcpp::Subscription<pick_and_place_interfaces::msg::JointControl>::SharedPtr joint_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}