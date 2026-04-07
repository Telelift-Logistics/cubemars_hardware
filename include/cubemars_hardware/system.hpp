// credit: https://github.com/ros-controls/ros2_control_demos
#ifndef CUBEMARS_HARDWARE__SYSTEM_HPP_
#define CUBEMARS_HARDWARE__SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "cubemars_hardware/visibility_control.h"
#include "cubemars_hardware/can.hpp"
#include "cubemars_hardware/can_utilities.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "univan_control_msgs/msg/motor_command.hpp"
#include "univan_control_msgs/msg/motor_command_group.hpp"
#include "univan_control_msgs/srv/motor_control_service.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"

namespace cubemars_hardware
{
class CubeMarsSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CubeMarsSystemHardware);
  
  virtual ~CubeMarsSystemHardware();

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CUBEMARS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CUBEMARS_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using MotorControlService = univan_control_msgs::srv::MotorControlService;
  using MotorControlServiceRequest = MotorControlService::Request;
  using MotorControlServiceResponse = MotorControlService::Response;

  using MotorCommandMsg = univan_control_msgs::msg::MotorCommand;
  using MotorCommandGrp = univan_control_msgs::msg::MotorCommandGroup;

  using ControlMessage = control_msgs::msg::DynamicInterfaceGroupValues;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;
  std::vector<double> hw_states_temperatures_;

  std::vector<double> erpm_conversions_;
  std::vector<double> torque_constants_;
  std::vector<double> enc_offs_;
  std::vector<double> trq_limits_;
  std::vector<JointLimits> hardware_limits_;
  std::vector<bool> mount_dir_; //True = up +ve, False = up -ve
  /// @brief Boolean variable to indicate whether to set zero at midpoint
  std::vector<bool> zero_at_midpoint_;
  std::vector<std::pair<std::int16_t, std::int16_t>> limits_;
  std::vector<bool> read_only_;

  CanSocket can_;
  std::string can_itf_;
  std::vector<std::uint32_t> can_ids_;

  enum control_mode_t : std::uint8_t
  {
    CURRENT_LOOP = 1,
    SPEED_LOOP = 3,
    POSITION_LOOP = 4,
    SET_ORIGIN_MODE = 5,
    POSITION_SPEED_LOOP = 6,
    UNDEFINED
  };

  // command mode switch variables
  std::vector<bool> stop_modes_;
  std::vector<control_mode_t> start_modes_;

  // active control mode for each actuator
  std::vector<control_mode_t> control_mode_;

  // The Service Callback (Runs in the non-RT ROS executor thread)
  /**
   * @brief Callback to process motor control commands
   * @param request The service request.
   * @param response The service response.
   */
  void motor_control_callback(
    const std::shared_ptr<MotorControlServiceRequest> request,
    std::shared_ptr<MotorControlServiceResponse> response);
  
  /**
   * @brief Function to process GPIO messages
   * @param msg The latest GPIO message.
   */
  void process_gpio_message(const ControlMessage &msg);
  
  /// @brief A dedicated node for the service
  rclcpp::Node::SharedPtr node_;
  // /// @brief A list of motor commands for safe handling
  // std::vector<MotorCommand> motor_commands_;
  /// @brief Real-time safe buffer
  realtime_tools::RealtimeThreadSafeBox<std::vector<MotorCommandMsg>> command_mailbox_;
  /// @brief Configure service server
  rclcpp::Service<MotorControlService>::SharedPtr motor_srvr_;
  /// @brief Subscriber for receiving GPIO states.
  rclcpp::Subscription<ControlMessage>::SharedPtr sub_gpio_states_;
  // @brief Background Thread Management
  std::thread service_thread_;
  std::atomic<bool> thread_running_{false};

  /// @brief Configure realtime publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<MotorCommandGrp>> state_publisher_;
  std::shared_ptr<rclcpp::Publisher<MotorCommandGrp>> s_publisher_;
  std::vector<MotorCommandMsg> motor_msgs_;
  MotorCommandGrp motor_msg_grp_;


  // /// @brief Boolean variable to determine calibration state
  std::atomic<bool> limit_sensor_state_{false};
  /// @brief Boolean variable to determine if calibration is running
  std::atomic<bool> is_calibration_running_{false};
  /// @brief Boolean variable to determine if state change is requested
  std::atomic<bool> is_changing_state_{false};
  /// @brief Boolean variable to determine if has pending request
  std::atomic<bool> has_request_{false};
  /// @brief Boolean variable to indicate if command input/output should be in m or cm
  bool use_meters_{false};
  /// @brief Boolean variable to indicate if sensor state should be used
  bool use_limit_sensor_{false};
  /// @brief container to store and track calibration process
  std::vector<CalibrationPhase> calibration_phase_{};

};

}  // namespace cubemars_hardware

#endif  // CUBEMARS_HARDWARE__SYSTEM_HPP_
