#include "robot_arm_base/robot_arm_interface.h"

namespace robot_arm_base {

    hardware_interface::CallbackReturn RobotArmInterface::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "On init...");

        position_commnads_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        prev_position_commands_.reserve(info.joints.size());

        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__node:=robot_arm_ros2_control" + info_.name});

        node_ = rclcpp::Node::make_shared("_", options);

        joints_publish_ = node_->create_publisher<std_msgs::msg::Int64MultiArray>(
            "robot_arm/joints", rclcpp::QoS(1));
        RCLCPP_INFO(logger_, "Finished On init.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotArmInterface::on_configure(const rclcpp_lifecycle::State&) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    std::vector<hardware_interface::StateInterface> RobotArmInterface::export_state_interfaces() {

        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotArmInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        for (size_t i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(
            hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commnads_[i]));
        }
        
        return command_interfaces;
    }

    hardware_interface::CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State& ) {
        RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"),"On activate ....");

        position_commnads_ = {0.0, 0.0, 0.0, 0.0};
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0};

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotArmInterface::on_deactivate(const rclcpp_lifecycle::State& ) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotArmInterface::read(const rclcpp::Time& , const rclcpp::Duration&) {
        position_states_ = position_commnads_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotArmInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
        if (position_commnads_ == prev_position_commands_) {
            return hardware_interface::return_type::OK;
        }
        joint_value_.data = {static_cast<int>(((position_commnads_.at(0) +  (M_PI/2)) * 180) / M_PI),
                             180 - static_cast<int>(((position_commnads_.at(1) +  (M_PI/2)) * 180) / M_PI),
                             static_cast<int>(((position_commnads_.at(2) +  (M_PI/2)) * 180) / M_PI),
                             static_cast<int>((-position_commnads_.at(3) + 180) / (M_PI / 2))};
        if (rclcpp::ok()) {
            joints_publish_->publish(joint_value_);
        }

        prev_position_commands_ = position_commnads_;

        return hardware_interface::return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_arm_base::RobotArmInterface, hardware_interface::SystemInterface)