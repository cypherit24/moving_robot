#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pick_and_place_interfaces/msg/joint_control.hpp>
#include <vector>
#include <string>
#include <unordered_map>

// robot_state_publisher , arm_sim_node, rviz2 사용
class ArmSimNode : public rclcpp::Node
{
public:
    ArmSimNode(): Node("arm_sim_node")
    {
        this->declare_parameter<std::vector<std::string>>("joint_names",{});
        this->declare_parameter<std::vector<double>>("lower_limits",{});
        this->declare_parameter<std::vector<double>>("upper_limits", {});
        this->declare_parameter<std::vector<double>>("initial_positions", {});
        this->declare_parameter<double>("default_velocity", 1.0);
        this->declare_parameter<double>("publish_rate_hz", 50.0);

        std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
        std::vector<double> lower_limits = this->get_parameter("lower_limits").as_double_array();
        std::vector<double> upper_limits = this->get_parameter("upper_limits").as_double_array();
        std::vector<double> initial_positions = this->get_parameter("initial_positions").as_double_array();

        double default_velocity = this->get_parameter("default_velocity").as_double();
        double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

        const size_t num_joints = joint_names.size();

        if (num_joints!=lower_limits.size() || 
        num_joints!=upper_limits.size() || num_joints!=initial_positions.size())
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "파라미터 배열 길이 불일치: joint_names: %zu, lower_limits: %zu,
                upper_limits: %zuinitial_positions: %zu",
                num_joints,lower_limits.size(),upper_limits.size(),initial_positions.size();
            );
            throw std::runtime_error("arm_sim_node 파라미터 배열 크기 검증 실패");
        }

// 4. 관절 이름 -> 인덱스 맵핑(unordered_map) 초기화 및 현재/목표 위치 vector 크기 맞추기
        current_positions_ =initial_positions;
        target_positions_ =initial_positions;
        target_velocities_ =std::vector<double>(num_joints, default_velocity);        

        for(size_t i=0; i<num_joints; i++)
        {
            joint_name_to_index_[joint_names[i]]=i;
        }

        joint_cmd_sub_= this->create_subscription<pick_and_place_interfaces::msg::JointControl>(
            "/arm/joint_command",10,
            std::bind(&ArmSimNode::joint_cmd_callback, this, std::placeholders::_1)
        );
    }
private:

    void joint_cmd_callback(const pick_and_place_interfaces::msg::JointControl::SharedPtr msg)
        {
            if (msg->joint_names.size() != msg->positions.size()){
                RCLCPP_WARN(this->get_logger(),"관절 이름과 포지션 갯수 불일치");
                return;
            }
            // positions 배열을 순회하며 원소가 무한대인지 검사 > 오류출력
            for(double pos: msg->positions){
                if (!std::isfinite(pos)){
                    RCLCPP_WARN(this->get_logger(),"position 원소가 무한대");
                    return;
                }
            }

            for(size_t i=0; i<msg->joint_names.size(); i++){
                size_t idx = joint_name_to_index_[msg->joint_names[i]];
                double clamped_pos = std::clamp(msg->positions[i], lower_limits_[idx],
                    upper_limits_[idx]);
                if(clamped_pos != msg->positions[i]){
                    RCLCPP_WARN(this->get_logger(),"position 범위 초과");
                }
                if (msg->max_velocity > 0.0){
                    target_velocities_[idx]= msg->max_velocity;
                } else {
                    target_velocities_[idx] = default_velocity_;
                }
            }
        }
  // 1. 내부 관절 상태 데이터 (C++ 표준 타입)
        std::unordered_map<std::string, size_t> joint_name_to_index_; // 관절 이름 -> vector 인덱스 매핑
        std::vector<double> current_positions_;                       // 현재 관절 위치 [rad 또는 m]
        std::vector<double> target_positions_;                        // 목표 관절 위치 [rad 또는 m]
        std::vector<double> target_velocities_;                       // 관절별 설정 속도 [rad/s 또는 m/s]
        rclcpp::Time last_update_time_;                               // dt 계산용 시간 기록

  // 2. ROS 2 통신 및 타이머 객체 (ROS 2 스마트 포인터)
        rclcpp::Subscription<pick_and_place_interfaces::msg::JointControl>::SharedPtr joint_cmd_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<double> lower_limits_;
        std::vector<double> upper_limits_;
        double default_velocity_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node =std::make_shared<ArmSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}