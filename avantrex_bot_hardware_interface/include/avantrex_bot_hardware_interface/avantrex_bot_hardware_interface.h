#ifndef AVANTREX_BOT_HARDWARE_INTERFACE_REAL_ROBOT_H
#define AVANTREX_BOT_HARDWARE_INTERFACE_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace avantrex_bot_hardware_interface
{
  using return_type = hardware_interface::return_type;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using StateInterface = hardware_interface::StateInterface;
  using CommandInterface = hardware_interface::CommandInterface;

  using JointState = sensor_msgs::msg::JointState;
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;

  class AvantrexBotHardwareInterface : public hardware_interface::SystemInterface
  {

  public:
    AvantrexBotHardwareInterface();

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<StateInterface> export_state_interfaces() override;

    std::vector<CommandInterface> export_command_interfaces() override;
    
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    void cleanup_node();

    realtime_tools::RealtimeBox<std::shared_ptr<JointState>> received_motor_state_msg_ptr_{ nullptr };

    std::shared_ptr<rclcpp::Publisher<Float32MultiArray>> motor_command_publisher_ = nullptr;

    std::shared_ptr<realtime_tools::RealtimePublisher<Float32MultiArray>> realtime_motor_command_publisher_ = nullptr;

    rclcpp::Subscription<JointState>::SharedPtr motor_state_subscriber_ = nullptr;

    std::map<std::string, double> vel_commands_;
    std::map<std::string, double> pos_state_;
    std::map<std::string, double> vel_state_;

    bool subscriber_is_active_ = false;

    std::shared_ptr<rclcpp::Node> node_;

    void motor_state_cb(const std::shared_ptr<JointState> msg);
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::unique_ptr<std::thread> executor_thread_;

    std::vector<std::string> velocity_command_joint_order_;

    uint connection_check_period_ms_;
    uint connection_timeout_ms_;

  private:
    rclcpp::Logger logger_;

    std::chrono::time_point<std::chrono::system_clock> time_;
  };
} // namespace avantrex_bot_hardware_interface

#endif // AVANTREX_BOT_HARDWARE_INTERFACE_REAL_ROBOT_H