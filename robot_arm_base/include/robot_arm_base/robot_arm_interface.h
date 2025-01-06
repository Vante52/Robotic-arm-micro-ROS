#pragma once

#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <std_msgs/msg/int64_multi_array.hpp>

namespace robot_arm_base {
    class RobotArmInterface : public hardware_interface::SystemInterface {
    public:
        RobotArmInterface() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration& period) override;
    private:
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr joints_publish_;
        
        rclcpp::Node::SharedPtr node_;
        std_msgs::msg::Int64MultiArray joint_value_;

        std::vector<double> position_commnads_;
        std::vector<double> prev_position_commands_;
        std::vector<double> position_states_;
        rclcpp::Logger logger_{rclcpp::get_logger("RobotArmInterface")};
    };
}