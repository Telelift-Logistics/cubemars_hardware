// credit: https://github.com/ros-controls/ros2_control_demos
#ifndef CUBEMARS_HARDWARE__SYSTEM_HPP_
#define CUBEMARS_HARDWARE__SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/executor.hpp"

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
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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

  // ---- ros2_control state / command storage ----
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;
  std::vector<double> hw_states_temperatures_;

  // ---- per-joint static motor parameters ----
  std::vector<double> erpm_conversions_;
  std::vector<double> torque_constants_;
  std::vector<double> enc_offs_;
  std::vector<double> trq_limits_;
  std::vector<double> m_per_rad_;  // lead screw factor; only used when use_meters_
  std::vector<JointLimits> hardware_limits_;
  std::vector<bool> mount_dir_;          // true = up is +ve in raw encoder frame
  std::vector<bool> zero_at_midpoint_;
  std::vector<std::pair<std::int16_t, std::int16_t>> limits_;  // (vel, acc) for POSITION_SPEED_LOOP
  std::vector<bool> read_only_;

  // ---- calibration parameters & runtime state (one entry per joint) ----
  std::vector<CalibrationConfig> calibration_cfg_;
  std::vector<CalibrationRuntime> calibration_rt_;

  // ---- CAN ----
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

  // ---- command mode switch ----
  std::vector<bool> stop_modes_;
  std::vector<control_mode_t> start_modes_;
  std::vector<control_mode_t> control_mode_;

  // ---- service plumbing ----
  void motor_control_callback(
    const std::shared_ptr<MotorControlServiceRequest> request,
    std::shared_ptr<MotorControlServiceResponse> response);

  /// @brief Process a GPIO states message and update per-joint limit sensor seen
  /// flags in calibration_rt_.
  void process_gpio_message(const ControlMessage & msg);

  /// @brief Reset per-joint runtime state and clear any prior offset so the
  /// raw encoder frame is the calibration frame.
  void enter_calibration(std::size_t joint_idx);

  /// @brief Transition a joint into a new calibration phase, stamping the
  /// phase start time and logging the transition.
  void set_phase(std::size_t joint_idx, CalibrationPhase new_phase);

  /// @brief Drive a single joint's calibration state machine for one cycle.
  /// Returns the raw-frame setpoint to be sent on POSITION_SPEED_LOOP this
  /// cycle (NaN means "do not issue a new setpoint").
  double step_calibration(std::size_t joint_idx);

  /// @brief Issue SETZEROPOSCMD on the CAN bus for the given joint and stamp
  /// the runtime state so subsequent reads honor the settle window.
  void issue_zero_command(std::size_t joint_idx);

  /// @brief Return the joint index that owns the given CAN id, or
  /// info_.joints.size() if no joint matches. can_id (immutable after on_init)
  /// is the mapping key for service requests, decoupling them from list order.
  std::size_t joint_index_for_can_id(std::uint8_t can_id) const;

  /// @brief Terminal calibration cleanup: stop motion, mark the joint
  /// uncalibrated, and disable it. The caller must set the phase to FAILED
  /// first (this only performs the side effects).
  void abort_calibration_failed(std::size_t joint_idx);

  rclcpp::Node::SharedPtr node_;
  realtime_tools::RealtimeThreadSafeBox<std::vector<MotorCommandMsg>> command_mailbox_;
  rclcpp::Service<MotorControlService>::SharedPtr motor_srvr_;
  rclcpp::Subscription<ControlMessage>::SharedPtr sub_gpio_states_;

  /// @brief Weak reference to the ControllerManager's executor (provided
  /// through HardwareComponentInterfaceParams). The CM spins our `node_`
  /// for us; we do NOT spawn our own thread.
  std::weak_ptr<rclcpp::Executor> executor_weak_;
  
  /// @brief Realtime motor status publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<MotorCommandGrp>> rt_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<MotorCommandGrp>> s_publisher_;
  std::vector<MotorCommandMsg> motor_msgs_;
  MotorCommandGrp motor_msg_grp_;

  /// @brief Realtime SAFE stop publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<ControlMessage>> rt_pub_gpio_command_;
   std::shared_ptr<rclcpp::Publisher<ControlMessage>> pub_gpio_command_;

  // ---- top-level flags ----
  std::atomic<bool> is_calibration_running_{false};
  std::atomic<bool> is_changing_state_{false};
  std::atomic<bool> has_request_{false};

  /// Whether to publish output in meters or rads
  bool use_meters_{false};

  /// @brief Resolved set-origin CAN payload. Pointer + length so we can
  /// dispatch the legacy MIT-mode form (8 bytes) and the servo-mode form
  /// (1 byte) through the same write path.
  struct SetOriginPayload
  {
    const std::uint8_t * data{nullptr};
    std::uint8_t        len{0};
  } set_origin_payload_;

  /// @brief Per-hardware-stack tunables loaded from URDF <hardware> parameters.
  /// Per-joint values live in CalibrationConfig.
  struct GlobalCalibrationCfg
  {
    bool auto_calibrate_on_activate{false};
    /// @brief Topic to subscribe to for GPIO sensor states.
    std::string gpio_states_topic{"gpio_controller/gpio_states"};
    /// @brief to publish GPIO commands.
    std::string gpio_cmd_topic{"gpio_controller/commands"};
    /// @brief Topic for publishing Lift motor states
    std::string status_topic{"lift_position_controller/status"};
    /// @brief Empty power sensor name disables GPIO check (only telemetry timeout is used).
    std::string gpio_power_group_name{};
    std::string gpio_power_ifc_name{};
    /// @brief Lower Limit Sensor (INPUT)
    std::string gpio_limit_group_name{};
    std::string gpio_limit_ifc_name{};
    /// @brief SAFE Stop "STOP_LIFT" (OUTPUT)
    std::string gpio_stop_lift_group_name{};
    std::string gpio_stop_lift_ifc_name{};
    /// Hard ceiling on int16 encoder count where we declare "overflow imminent".
    /// CubeMars reports position as int16 centidegrees; ±32000 ≈ ±320°.
    std::int16_t encoder_overflow_threshold{32000};
    /// Whether to use LIMIT Sensor for finding ROOT
    bool use_limit_sensor{false};
    /// Number of calibration retry attempts
    std::int16_t max_retries;

    /// Telemetry-timeout fallback for power-loss detection.
    std::chrono::milliseconds power_loss_timeout{500};

    /// How many consecutive good telemetry frames must arrive before
    /// declaring the lift RECOVERING (debounce against transient blips).
    int min_telemetry_frames_to_resume{3};

    /// @brief On transition RECOVERING → calibrating, run calibration automatically
    /// (true) or wait for a service request (false).
    bool auto_recalibrate_on_power_restore{true};

    /// Consecutive active lower-limit-sensor readings required before the limit
    /// is asserted (debounce). Deactivation is immediate. 1 disables debounce.
    int limit_debounce_frames{2};
  } global_cfg_;

  // ---- lift power state tracking ----
  std::atomic<LiftPowerState> lift_power_state_{LiftPowerState::ONLINE};
  std::atomic<bool> gpio_power_seen_high_{true};  // latched from GPIO callback
  std::atomic<bool> gpio_top_sensor_seen_{false};  // latched from GPIO callback
  std::atomic<bool> gpio_bottom_sensor_seen_{false};  // latched from GPIO callback
  std::vector<std::chrono::steady_clock::time_point> last_telemetry_;
  std::vector<int> good_frames_since_offline_;

  // ---- lower-limit sensor state ----
  /// @brief Debounced lower-limit state, one bit per joint (bit i set => joint
  /// i is at its lower limit). Written by process_gpio_message (node thread),
  /// read by read()/write() (RT thread); atomic for that cross-thread access.
  std::atomic<std::uint64_t> limit_active_mask_{0};
  /// @brief Per-joint consecutive active-reading count for debounce. Only
  /// touched inside process_gpio_message (node thread).
  std::vector<int> limit_active_count_;

  /// @brief Evaluate GPIO + telemetry timestamps and update lift_power_state_.
  /// Called from read() each cycle.
  void update_power_state();

  /// @brief Clear all per-joint state that is invalidated by a power loss
  /// (calibration, encoder offset, mailbox, pending requests).
  void enter_offline_state();

  /// @brief On transition from OFFLINE → RECOVERING: either auto-queue a
  /// calibration request or just log that we're awaiting one.
  void enter_recovering_state();

  /// @brief Publisher to trigger SAFE stop.
  /// Called from read() each cycle.
  void enter_safe_state(const bool &set = true);

};

}  // namespace cubemars_hardware

#endif  // CUBEMARS_HARDWARE__SYSTEM_HPP_