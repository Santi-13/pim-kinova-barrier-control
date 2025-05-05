#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/logging.hpp"
#include <vector>
#include <memory>
#include <string>
#include <map>       // Add this line

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace custom_hardware_plugins {
  class VelocityIntegratorHardware : public hardware_interface::SystemInterface {
    private:
      std::vector<double> position_states_;
      std::vector<double> velocity_commands_;
      std::vector<double> velocity_states_;

    public:
      using hardware_interface::SystemInterface::SystemInterface;

      CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
        RCLCPP_INFO(rclcpp::get_logger("VelocityIntegratorHardware"), "Plugin initialized!");
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
          return CallbackReturn::ERROR;
        }

        position_states_.resize(info_.joints.size(), 0.0);
        velocity_commands_.resize(info_.joints.size(), 0.0);
        velocity_states_.resize(info_.joints.size(), 0.0);

        return CallbackReturn::SUCCESS;
      }

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
          state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        }
        return state_interfaces;
      }

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
        }
        return command_interfaces;
      }

      CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          velocity_commands_[i] = velocity_states_[i];  // Initialize with current state
        }
        return CallbackReturn::SUCCESS;
      }

      hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override {
        // Simulate sensor read: Copy commands to states
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          velocity_states_[i] = velocity_commands_[i];
        }
        return hardware_interface::return_type::OK;
      }

      hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration& period) override {
        // Integrate velocity to position
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          position_states_[i] += velocity_commands_[i] * period.seconds();
        }
        return hardware_interface::return_type::OK;
      }
  };
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  custom_hardware_plugins::VelocityIntegratorHardware,  // Namespaced class
  hardware_interface::SystemInterface
);
