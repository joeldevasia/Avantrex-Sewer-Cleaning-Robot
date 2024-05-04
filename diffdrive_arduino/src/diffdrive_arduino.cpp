#include "diffdrive_arduino/diffdrive_arduino.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{

  DiffDriveArduino::DiffDriveArduino()
      : logger_(rclcpp::get_logger("DiffDriveArduino"))
  {
  }

  CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring...");

    time_ = std::chrono::system_clock::now();

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' has %zu command interfaces found. 1 expected.",
                     joint.name.c_str(), joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                     joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' has %zu state interface. 2 expected.",
                     joint.name.c_str(), joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                     joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                     joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }

    for (auto &j : info_.joints)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' found", j.name.c_str());

      pos_state_[j.name] = 0.0;
      vel_state_[j.name] = 0.0;
      vel_commands_[j.name] = 0.0;
    }

    connection_timeout_ms_ = std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
    connection_check_period_ms_ = std::stoul(info_.hardware_parameters["connection_check_period_ms"]);

    std::string velocity_command_joint_order_raw = info_.hardware_parameters["velocity_command_joint_order"];

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Received velocity command joint order raw: %s",
    //             velocity_command_joint_order_raw.c_str());

    // remove whitespaces
    velocity_command_joint_order_raw.erase(
        std::remove_if(velocity_command_joint_order_raw.begin(), velocity_command_joint_order_raw.end(), [](char c)
                       { return std::isspace(static_cast<unsigned char>(c)); }),
        velocity_command_joint_order_raw.end());

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Removed Whitespaces from velocity command joint order: %s",
    //             velocity_command_joint_order_raw.c_str());

    std::stringstream velocity_command_joint_order_stream(velocity_command_joint_order_raw);
    std::string joint_name;
    while (getline(velocity_command_joint_order_stream, joint_name, ','))
    {
      velocity_command_joint_order_.push_back(joint_name);
    }

    if (velocity_command_joint_order_.size() != info_.joints.size())
    {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint order size is invalid");
      return CallbackReturn::ERROR;
    }

    for (auto &j : info_.joints)
    {
      if (std::find(velocity_command_joint_order_.begin(), velocity_command_joint_order_.end(), j.name) ==
          velocity_command_joint_order_.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduino"), "Joint '%s' missing from velocity command joint order",
                     j.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    node_ = std::make_shared<rclcpp::Node>("avantrexbot_system_node");
    executor_.add_node(node_);
    executor_thread_ =
        std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

    return CallbackReturn::SUCCESS;

    RCLCPP_INFO(logger_, "Finished Configuration");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DiffDriveArduino::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Configuring");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DiffDriveArduino::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Cleaning up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    motor_command_publisher_ = node_->create_publisher<Float32MultiArray>("/motors_cmd", rclcpp::SensorDataQoS());
    realtime_motor_command_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<Float32MultiArray>>(motor_command_publisher_);

    motor_state_subscriber_ =
        node_->create_subscription<JointState>("/motors_response", rclcpp::SensorDataQoS(),
                                               std::bind(&DiffDriveArduino::motor_state_cb, this, std::placeholders::_1));

    std::shared_ptr<JointState> motor_state;
    for (uint wait_time = 0; wait_time <= connection_timeout_ms_; wait_time += connection_check_period_ms_)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DiffDriveArduino"), *node_->get_clock(), 5000, "Feedback message from motors wasn't received yet");
      received_motor_state_msg_ptr_.get(motor_state);
      if (motor_state)
      {
        RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
        return CallbackReturn::SUCCESS;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
    }

    RCLCPP_FATAL(node_->get_logger(), "Activation failed, timeout reached while waiting for feedback from motors");
    return CallbackReturn::ERROR;
    // return CallbackReturn::SUCCESS;
  }

  CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Deactivating");
    cleanup_node();
    received_motor_state_msg_ptr_.set(nullptr);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DiffDriveArduino::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Shutting down");
    cleanup_node();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DiffDriveArduino::on_error(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Handling error");
    cleanup_node();
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> DiffDriveArduino::export_state_interfaces()
  {

    // We need to set up a position and a velocity interface for each wheel

    std::vector<StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
          StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[info_.joints[i].name]));
      state_interfaces.emplace_back(
          StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[info_.joints[i].name]));
    }

    return state_interfaces;

  }

  std::vector<CommandInterface> DiffDriveArduino::export_command_interfaces()
  {
    // We need to set up a velocity command interface for each wheel

    std::vector<CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[info_.joints[i].name]));
    }

    return command_interfaces;
  }

  void DiffDriveArduino::motor_state_cb(const std::shared_ptr<JointState> msg)
  {
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Received motors response '%s'", msg->name[0].c_str());
    received_motor_state_msg_ptr_.set(std::move(msg));
  }

  void DiffDriveArduino::cleanup_node()
  {
    motor_state_subscriber_.reset();
    realtime_motor_command_publisher_.reset();
    motor_command_publisher_.reset();
  }

  return_type DiffDriveArduino::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    std::shared_ptr<JointState> motor_state;
    received_motor_state_msg_ptr_.get(motor_state);

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Reading motors state");
    
    // Print JointStateMessage from motor_state

    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "motor_state->name[0]: %s", motor_state->name[0].c_str());
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "motor_state->position[0]: %f", motor_state->position[0]);
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "motor_state->velocity[0]: %f", motor_state->velocity[0]);
    

    if (!motor_state)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Feedback message from motors wasn't received");
      return return_type::ERROR;
    }

    // ros2 topic pub /motors_response sensor_msgs/msg/JointState "{header: {frame_id: "base_link"}, name: ['left_wheel_joint', 'right_wheel_joint'], position: [0.0, 0.0,], velocity: [0.0, 0.1], effort: [0.0, 0.0]}"

    for (auto i = 0u; i < motor_state->name.size(); i++)
    {
      if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
          vel_state_.find(motor_state->name[i]) == vel_state_.end())
      {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Position or velocity feedback not found for joint %s",
                     motor_state->name[i].c_str());
        return return_type::ERROR;
      }

      pos_state_[motor_state->name[i]] = motor_state->position[i];
      vel_state_[motor_state->name[i]] = motor_state->velocity[i];

      // RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveArduino"), "Position feedback: %f, velocity feedback: %f",
      //              pos_state_[motor_state->name[i]], vel_state_[motor_state->name[i]]);
    }

    return return_type::OK;
  }

  return_type DiffDriveArduino::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {


    if (realtime_motor_command_publisher_->trylock())
    {
      auto &motor_command = realtime_motor_command_publisher_->msg_;
      motor_command.data.clear();

      // RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveArduino"), "Wrtiting motors cmd message");

      for (auto const &joint : velocity_command_joint_order_)
      {
        motor_command.data.push_back(vel_commands_[joint]);
      }

      realtime_motor_command_publisher_->unlockAndPublish();
    }

    return return_type::OK;
  }

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    diffdrive_arduino::DiffDriveArduino,
    hardware_interface::SystemInterface)
