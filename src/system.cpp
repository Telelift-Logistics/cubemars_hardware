#include "cubemars_hardware/system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{

namespace {
// Local helpers --------------------------------------------------------------

/// @brief Pull a hardware parameter by name as int, with a default.
int get_hw_param_int(
  const hardware_interface::HardwareInfo & info, const std::string & key, int def)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return def;
  try { return std::stoi(it->second); } catch (...) { return def; }
}

/// @brief Pull a hardware parameter by name as string, with a default.
std::string get_hw_param_str(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & def)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? it->second : def;
}

/// @brief Pull a per-joint parameter as double, with a default.
double get_joint_param_double(
  const hardware_interface::ComponentInfo & joint, const std::string & key, double def)
{
  auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) return def;
  try { return std::stod(it->second); } catch (...) { return def; }
}

/// @brief Pull a per-joint parameter as int, with a default.
int get_joint_param_int(
  const hardware_interface::ComponentInfo & joint, const std::string & key, int def)
{
  auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) return def;
  try { return std::stoi(it->second); } catch (...) { return def; }
}

/// @brief Pull a per-joint parameter as string, with a default.
std::string get_joint_param_str(
  const hardware_interface::ComponentInfo & joint, const std::string & key,
  const std::string & def)
{
  auto it = joint.parameters.find(key);
  return (it != joint.parameters.end()) ? it->second : def;
}

}  // namespace


CubeMarsSystemHardware::~CubeMarsSystemHardware()
{
  // If the controller manager is shutdown via Ctrl + C
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // -------------------- hardware-level parameters --------------------
  if (info_.hardware_parameters.count("can_interface") != 0) {
    can_itf_ = info_.hardware_parameters.at("can_interface");
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "No can_interface specified in URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    use_meters_       = (get_hw_param_int(info_, "use_meters", 0) == 1);
    global_cfg_.use_limit_sensor = (get_hw_param_int(info_, "use_limit_sensor", 0) == 1);
    global_cfg_.auto_calibrate_on_activate =
      (get_hw_param_int(info_, "auto_calibrate", 0) == 1);
    global_cfg_.gpio_states_topic =
      get_hw_param_str(info_, "gpio_states_topic", "gpio_controller/gpio_states");
    global_cfg_.gpio_cmd_topic =
      get_hw_param_str(info_, "gpio_cmd_topic", "gpio_controller/commands");
    global_cfg_.status_topic =
      get_hw_param_str(info_, "status_topic", "lift_position_controller/status");
    global_cfg_.encoder_overflow_threshold =
      static_cast<std::int16_t>(get_hw_param_int(info_, "encoder_overflow_threshold", 32000));
    global_cfg_.max_retries =
      static_cast<std::int16_t>(get_hw_param_int(info_, "max_retries", 0));
  }

  // Resolve the SET_ORIGIN_MODE CAN payload form.
  //   "temporary" (default): servo-mode 1-byte 0x00 -- not persisted to NVM
  //   "permanent":           servo-mode 1-byte 0x01 -- persisted to NVM
  //   "restore":             servo-mode 1-byte 0x02 -- restore factory zero
  //   "legacy":              MIT-mode 8-byte 0xFF..0xFE form (older firmware)
  {
    const std::string mode =
      get_hw_param_str(info_, "set_origin_mode", "temporary");
    if (mode == "temporary") {
      set_origin_payload_ = {SET_ORIGIN_TEMPORARY, 1};
    } else if (mode == "permanent") {
      set_origin_payload_ = {SET_ORIGIN_PERMANENT, 1};
    } else if (mode == "restore") {
      set_origin_payload_ = {SET_ORIGIN_RESTORE, 1};
    } else if (mode == "legacy") {
      set_origin_payload_ = {SETZEROPOSCMD, 8};
    } else {
      RCLCPP_FATAL(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "Unknown set_origin_mode '%s'. Valid: temporary|permanent|restore|legacy",
        mode.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                "Using set_origin_mode='%s' (%u-byte payload)",
                mode.c_str(), set_origin_payload_.len);
  }

  // Lift power monitoring/ SAFE stop triggering
  {
    global_cfg_.gpio_power_group_name =
      get_hw_param_str(info_, "gpio_power_group_name", "");
    global_cfg_.gpio_power_ifc_name =
      get_hw_param_str(info_, "gpio_power_ifc_name", "");
    if (global_cfg_.gpio_power_group_name.empty() !=
        global_cfg_.gpio_power_ifc_name.empty())
    {
      RCLCPP_FATAL(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "gpio_power_group_name and gpio_power_ifc_name must be "
                  "set together (or both empty to disable GPIO check for Lift power state)");
      return hardware_interface::CallbackReturn::ERROR;
    }

    global_cfg_.gpio_stop_lift_group_name =
      get_hw_param_str(info_, "gpio_stop_lift_group_name", "");
    global_cfg_.gpio_stop_lift_ifc_name =
      get_hw_param_str(info_, "gpio_stop_lift_ifc_name", "");
    if (global_cfg_.gpio_stop_lift_group_name.empty() !=
        global_cfg_.gpio_stop_lift_ifc_name.empty())
    {
      RCLCPP_FATAL(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "gpio_stop_lift_group_name and gpio_stop_lift_ifc_name must be "
                  "set together (or both empty to disable SAFE stop publishing)");
      return hardware_interface::CallbackReturn::ERROR;
    }

    global_cfg_.power_loss_timeout = std::chrono::milliseconds(
      get_hw_param_int(info_, "power_loss_timeout_ms", 500));
    global_cfg_.min_telemetry_frames_to_resume =
      get_hw_param_int(info_, "min_telemetry_frames_to_resume", 3);
    global_cfg_.auto_recalibrate_on_power_restore =
      (get_hw_param_int(info_, "auto_recalibrate_on_power_restore", 1) == 1);
    global_cfg_.limit_debounce_frames =
      std::max(1, get_hw_param_int(info_, "limit_debounce_frames", 2));
  }

  // -------------------- resize per-joint vectors --------------------
  const std::size_t n = info_.joints.size();
  hw_states_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_temperatures_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.assign(n, std::numeric_limits<double>::quiet_NaN());
  control_mode_.assign(n, control_mode_t::UNDEFINED);
  calibration_cfg_.assign(n, CalibrationConfig{});
  calibration_rt_.assign(n, CalibrationRuntime{});
  last_telemetry_.assign(n, std::chrono::steady_clock::time_point{});
  good_frames_since_offline_.assign(n, 0);
  limit_active_count_.assign(n, 0);

  // -------------------- per-joint parameters --------------------
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Required motor parameters
    if (joint.parameters.count("can_id") == 0 ||
        joint.parameters.count("kt") == 0 ||
        joint.parameters.count("pole_pairs") == 0 ||
        joint.parameters.count("gear_ratio") == 0)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "Missing parameters in URDF for %s", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    can_ids_.emplace_back(std::stoul(joint.parameters.at("can_id")));
    torque_constants_.emplace_back(std::stod(joint.parameters.at("kt")));
    const double erpm_conversion =
      std::stoi(joint.parameters.at("pole_pairs")) *
      std::stoi(joint.parameters.at("gear_ratio")) * 60.0 / (2.0 * M_PI);
    erpm_conversions_.emplace_back(erpm_conversion);

    // (vel, acc) limits for POSITION_SPEED_LOOP. Users specify both in
    // OUTPUT-SHAFT units:
    //   vel_limit:  rad/s
    //   acc_limit:  rad/s^2
    // erpm_conversion already accounts for gear ratio and pole pairs, so
    // the conversion to the wire's ERPM / ERPM-per-s units is a single
    // multiplication. Bounds-check the floating-point result before casting
    // to int32 to avoid UB on overflow, NaN, or inf.
    if (joint.parameters.count("acc_limit") != 0 &&
        joint.parameters.count("vel_limit") != 0)
    {
      const double vel_rad_s  = std::stod(joint.parameters.at("vel_limit"));
      const double acc_rad_s2 = std::stod(joint.parameters.at("acc_limit"));
      const double vel_erpm   = vel_rad_s  * erpm_conversion;
      const double acc_erpm   = acc_rad_s2 * erpm_conversion;

      auto in_int16_range = [](double v) {
        return std::isfinite(v) && v > 0.0 && v < 32767.0;
      };

      if (!in_int16_range(vel_erpm)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "vel_limit (%.3f rad/s) -> %.1f ERPM is out of range 1..32766. "
          "Reduce vel_limit or check pole_pairs/gear_ratio.",
          vel_rad_s, vel_erpm);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!in_int16_range(acc_erpm)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "acc_limit (%.3f rad/s^2) -> %.1f ERPM/s is out of range 1..32766. "
          "Reduce acc_limit or check pole_pairs/gear_ratio.",
          acc_rad_s2, acc_erpm);
        return hardware_interface::CallbackReturn::ERROR;
      }

      limits_.emplace_back(static_cast<std::int16_t>(vel_erpm),
                           static_cast<std::int16_t>(acc_erpm));
    } else {
      limits_.emplace_back(std::make_pair(0, 0));
    }

    enc_offs_.emplace_back(get_joint_param_double(joint, "enc_off", 0.0));
    trq_limits_.emplace_back(std::max(0.0, get_joint_param_double(joint, "trq_limit", 0.0)));
    read_only_.emplace_back(get_joint_param_int(joint, "read_only", 0) == 1);

    // Lead screw conversion: lead_pitch is linear travel (m) per revolution
    // of the OUTPUT shaft. Stored internally as m/rad = lead_pitch / (2π).
    // Required when use_meters_ is set; ignored otherwise.
    if (use_meters_) {
      const double lead_pitch =
        get_joint_param_double(joint, "lead_pitch", 0.0);
      if (lead_pitch <= 0.0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %s: use_meters=1 requires a positive 'lead_pitch' parameter "
          "(meters of linear travel per output-shaft revolution)",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      m_per_rad_.push_back(lead_pitch / (2.0 * M_PI));
    } else {
      m_per_rad_.push_back(1.0);  // unused; placeholder keeps vector size aligned
    }

    JointLimits jl;
    jl.range = std::max(0.0, get_joint_param_double(joint, "max_range", 0.0));
    hardware_limits_.push_back(jl);

    // mount_dir: param value 1 means "up is -ve in raw encoder frame"
    mount_dir_.emplace_back(get_joint_param_int(joint, "mount_dir", 0) != 1);
    // Default true: center the raw encoder at the midpoint (best int16
    // headroom, and the behavior this driver has always used). Set to 0 for the
    // simpler bottom-only strategy when the joint's range fits int16 from 0.
    zero_at_midpoint_.emplace_back(get_joint_param_int(joint, "zero_at_midpoint", 1) == 1);

    // ---- Calibration config (per joint) ----
    CalibrationConfig & cfg = calibration_cfg_.back();
    cfg = CalibrationConfig{};  // defaults already initialized in struct

    const bool bypass = (get_joint_param_int(joint, "bypass_calibration", 0) == 1);
    cfg.enabled = !bypass && (jl.range > 0.0);

    cfg.search_step =
      get_joint_param_double(joint, "calibration_search_step", cfg.search_step);
    cfg.position_tolerance =
      get_joint_param_double(joint, "calibration_position_tolerance",
                             cfg.position_tolerance);
    cfg.stall_torque =
      get_joint_param_double(joint, "calibration_stall_torque", cfg.stall_torque);

    cfg.step_period = std::chrono::milliseconds(
      get_joint_param_int(joint, "calibration_step_period_ms",
                          static_cast<int>(cfg.step_period.count())));
    cfg.phase_timeout = std::chrono::seconds(
      get_joint_param_int(joint, "calibration_phase_timeout_s",
                          static_cast<int>(cfg.phase_timeout.count())));
    cfg.zero_settle = std::chrono::milliseconds(
      get_joint_param_int(joint, "calibration_zero_settle_ms",
                          static_cast<int>(cfg.zero_settle.count())));

    cfg.gpio_group_name  = get_joint_param_str(joint, "gpio_group_name", "");
    cfg.gpio_ifc_name = get_joint_param_str(joint, "gpio_ifc_name", "");

    // ---- Validation ----
    if (cfg.enabled) {
      if (cfg.search_step <= 0.0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %s: calibration_search_step must be > 0", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (cfg.search_step > jl.range * 0.1) {
        RCLCPP_WARN(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %s: calibration_search_step (%f) is large relative to range (%f); "
          "consider reducing for safety",
          joint.name.c_str(), cfg.search_step, jl.range);
      }
      if (cfg.position_tolerance <= 0.0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %s: calibration_position_tolerance must be > 0", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // GPIO sensor requires both names if global_cfg_.use_limit_sensor is on AND the joint
      // declares one. Mixed config is OK (some joints use sensor, others fall
      // back to torque+timeout).
      const bool gpio_partial =
        (cfg.gpio_group_name.empty()) != (cfg.gpio_ifc_name.empty());
      if (gpio_partial) {
        RCLCPP_ERROR(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %s: gpio_group_name and gpio_ifc_name must be set together",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      const bool will_use_sensor =
        global_cfg_.use_limit_sensor && !cfg.gpio_ifc_name.empty();
      if (!will_use_sensor && cfg.stall_torque <= 0.0) {
        RCLCPP_WARN(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %s: no GPIO sensor configured and stall_torque <= 0; "
          "FIND_ROOT will only be terminated by encoder overflow or timeout",
          joint.name.c_str());
      }
    }

    // ---- Lower-limit sensor availability (informational) ----
    const bool limit_sensor_active =
      global_cfg_.use_limit_sensor && !cfg.gpio_ifc_name.empty();
    RCLCPP_INFO(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Joint %s: lower-limit sensor %s", joint.name.c_str(),
      limit_sensor_active
        ? "ENABLED (protective downward stop active)"
        : "not configured (behavior unchanged)");

    // ---- Build outbound MotorCommand record ----
    MotorCommandMsg msg;
    msg.can_id = static_cast<uint8_t>(std::stoul(joint.parameters.at("can_id")));
    msg.is_calibrated = bypass;  // bypass means we trust the existing zero
    msg.calibrate = false;
    motor_msgs_.emplace_back(msg);
  }

  // -------------------- ROS plumbing --------------------
  executor_weak_ = params.executor;

  if (auto locked_executor = executor_weak_.lock())
  {
    std::string node_name = this->get_name() + "_internal_node";
    node_ = std::make_shared<rclcpp::Node>(node_name);
    
    motor_srvr_ = node_->create_service<MotorControlService>(
    "lift_platform_hardware",
    std::bind(&CubeMarsSystemHardware::motor_control_callback, this,
              std::placeholders::_1, std::placeholders::_2));

    if (global_cfg_.use_limit_sensor) {
      sub_gpio_states_ = node_->create_subscription<ControlMessage>(
        global_cfg_.gpio_states_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&CubeMarsSystemHardware::process_gpio_message,
                  this, std::placeholders::_1));
    }

    // Realtime Publisher setup
    {
      s_publisher_ = node_->create_publisher<MotorCommandGrp>(
        std::string(global_cfg_.status_topic), rclcpp::SystemDefaultsQoS());
      rt_state_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<MotorCommandGrp>>(s_publisher_);

      pub_gpio_command_ = node_->create_publisher<ControlMessage>(
        std::string(global_cfg_.gpio_cmd_topic), rclcpp::SystemDefaultsQoS());
      rt_pub_gpio_command_ =
        std::make_unique<realtime_tools::RealtimePublisher<ControlMessage>>(pub_gpio_command_);
    }
    

    // Register the custom node into the ControllerManager's executor hierarchy
    try {
      locked_executor->add_node(node_->get_node_base_interface());
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("CubeMarsSystemHardware"),
                   "Failed to add internal node to executor: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node_->get_logger(), "Successfully registered internal node and hooks to executor.");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Failed to lock the executor pointer!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // -------------------- auto-calibrate latching --------------------
  // We DON'T set has_request_ here directly; on_activate will queue a synthetic
  // request once interfaces are claimed. Setting it at init time risks
  // triggering before the controller is ready.

  // Mark default ENABLE for downstream message publishing.
  for (std::size_t i = 0; i < n; ++i) {
    motor_msgs_[i].get_motor_state = MotorCommandMsg::ENABLE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result =
    can_.connect(can_itf_, can_ids_, 0xFFU)
      ? hardware_interface::CallbackReturn::SUCCESS
      : hardware_interface::CallbackReturn::FAILURE;

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication active");
  return result;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (node_) {
    if (auto locked_executor = executor_weak_.lock()) {
      try {
        locked_executor->remove_node(node_->get_node_base_interface());
      } catch (const std::exception & e) {
        RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "remove_node during cleanup raised: %s", e.what());
      }
    }
    node_.reset();
  }

  const hardware_interface::CallbackReturn result =
    can_.disconnect()
      ? hardware_interface::CallbackReturn::SUCCESS
      : hardware_interface::CallbackReturn::FAILURE;
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication closed");
  return result;
}

std::vector<hardware_interface::StateInterface>
CubeMarsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "temperature", &hw_states_temperatures_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CubeMarsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type CubeMarsSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  stop_modes_.clear();
  start_modes_.clear();
  stop_modes_.resize(info_.joints.size(), false);

  std::unordered_set<std::string> eff{"effort"};
  std::unordered_set<std::string> vel{"velocity"};
  std::unordered_set<std::string> pos{"position"};

  std::unordered_set<std::string> joint_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    for (std::string key : stop_interfaces) {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "stop interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos) {
        stop_modes_[i] = true;
        break;
      }
    }

    joint_interfaces.clear();
    for (std::string key : start_interfaces) {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "start interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos) {
        joint_interfaces.insert(key.substr(key.find("/") + 1));
      }
    }

    control_mode_t resolved;
    if (joint_interfaces == eff) {
      resolved = CURRENT_LOOP;
    } else if (joint_interfaces == vel) {
      resolved = SPEED_LOOP;
    } else if (joint_interfaces == pos) {
      resolved = (limits_[i].first == 0 || limits_[i].second == 0)
                   ? POSITION_LOOP : POSITION_SPEED_LOOP;
    } else if (joint_interfaces.empty()) {
      resolved = stop_modes_[i] ? UNDEFINED : control_mode_[i];
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                   "Joint %zu (%s): unrecognized interface combination, "
                   "rejecting mode switch", i, info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                "Joint %zu (%s): resolved mode = %d",
                i, info_.joints[i].name.c_str(), resolved);
    start_modes_.push_back(resolved);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (stop_modes_[i]) {
      hw_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_commands_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    }
    control_mode_[i] = start_modes_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Auto-calibrate kicks in here, only for joints that are configured for it.
  // We synthesize a "calibrate everything" command into the mailbox so the
  // normal request path handles it.
  if (global_cfg_.auto_calibrate_on_activate) {
    std::vector<MotorCommandMsg> auto_cmd(info_.joints.size());
    bool any_to_calibrate = false;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      auto_cmd[i].can_id = motor_msgs_[i].can_id;
      auto_cmd[i].set_motor_state = 0;
      if (calibration_cfg_[i].enabled && !motor_msgs_[i].is_calibrated) {
        auto_cmd[i].calibrate = true;
        any_to_calibrate = true;
      }
    }
    if (any_to_calibrate) {
      command_mailbox_.set(auto_cmd);
      has_request_.store(true);
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Auto-calibration queued on activate");
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Calibration helpers
// ============================================================================

void CubeMarsSystemHardware::enter_calibration(std::size_t i)
{
  // Recover the current raw encoder position BEFORE wiping enc_offs_, so the
  // first FIND_ROOT step is a small delta from where the lift actually is, not
  // a jump. The calibration state machine drives raw-frame setpoints, but
  // hw_states_positions_ is in the reported frame:
  //   reported = (raw - enc_offs_) * dir_up   (see read())
  // Inverting (dir_up = +/-1, so 1/dir_up = dir_up):
  //   raw = reported * dir_up + enc_offs_
  // enc_offs_ must still hold the OLD value here; reading it after the wipe
  // would drop the offset a prior calibration left behind and seed the lift
  // half_range away from its true position. If the read hasn't populated yet,
  // fall back to 0.
  const double dir_up = mount_dir_[i] ? +1.0 : -1.0;
  const double cur_raw = std::isnan(hw_states_positions_[i])
    ? 0.0
    : hw_states_positions_[i] * dir_up + enc_offs_[i];

  // Wipe any prior offset so the raw encoder frame == calibration frame.
  // This is critical: phases 1-3 rely on raw readings.
  enc_offs_[i] = 0.0;

  calibration_rt_[i] = CalibrationRuntime{};
  calibration_rt_[i].commanded_setpoint = cur_raw;
  motor_msgs_[i].is_calibrated = false;
  motor_msgs_[i].calibrate = true;

  set_phase(i, CalibrationPhase::FIND_ROOT);
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
              "Joint %zu: entering calibration (range=%.4f, mount_dir=%s, start=%.4f)",
              i, hardware_limits_[i].range, mount_dir_[i] ? "up=+" : "up=-",
              calibration_rt_[i].commanded_setpoint);
}

void CubeMarsSystemHardware::set_phase(std::size_t i, CalibrationPhase p)
{
  const auto now = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
              "Joint %zu: phase %s -> %s",
              i, to_string(calibration_rt_[i].phase), to_string(p));
  calibration_rt_[i].phase = p;
  calibration_rt_[i].phase_started = now;
  calibration_rt_[i].last_step = now;
  calibration_rt_[i].limit_sensor_seen = false;
}

void CubeMarsSystemHardware::issue_zero_command(std::size_t i)
{
  // Set encoder origin.
  can_.write_message(can_ids_[i] | SET_ORIGIN_MODE << 8,
                     set_origin_payload_.data, set_origin_payload_.len);
  calibration_rt_[i].zero_issued = std::chrono::steady_clock::now();
  calibration_rt_[i].zero_cmd_pending = true;
}

std::size_t CubeMarsSystemHardware::joint_index_for_can_id(std::uint8_t can_id) const
{
  for (std::size_t i = 0; i < motor_msgs_.size(); ++i) {
    if (motor_msgs_[i].can_id == can_id) return i;
  }
  return info_.joints.size();  // not found
}

void CubeMarsSystemHardware::abort_calibration_failed(std::size_t i)
{
  can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, ZEROCMD, 4);
  motor_msgs_[i].calibrate = false;
  motor_msgs_[i].is_calibrated = false;
  motor_msgs_[i].get_motor_state = MotorCommandMsg::DISABLE;
}

double CubeMarsSystemHardware::step_calibration(std::size_t i)
{
  // Returns the raw-frame setpoint to send this cycle, or NaN to skip writing.
  using namespace std::chrono;
  auto & rt = calibration_rt_[i];
  const auto & cfg = calibration_cfg_[i];
  const auto now = steady_clock::now();

  // Honor the zero-settle window: don't transition until the motor has had
  // time to acknowledge the SETZEROPOSCMD and report fresh readings.
  if (rt.zero_cmd_pending) {
    if (now - rt.zero_issued < cfg.zero_settle) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    rt.zero_cmd_pending = false;
  }

  // Direction multiplier: in raw frame, "up" is +1 when mount_dir_ is true.
  const double dir_up = mount_dir_[i] ? +1.0 : -1.0;
  const double half_range = hardware_limits_[i].range * 0.5;

  switch (rt.phase) {
    case CalibrationPhase::FIND_ROOT: {
      // First, check whether we've already arrived at the bottom this cycle.
      // hw_states_positions_[i] is the reported position; during FIND_ROOT
      // enc_offs_ == 0 so reported == raw (in joint units). For the overflow
      // backstop we need the int16 raw count: convert the threshold into
      // joint units so we can compare apples to apples.
      const double overflow_thresh_units =
        (global_cfg_.encoder_overflow_threshold * 0.1 * M_PI / 180.0) *
        (use_meters_ ? m_per_rad_[i] : 1.0);

      const bool gpio_active =
        global_cfg_.use_limit_sensor && !cfg.gpio_ifc_name.empty();
      bool bottom_found = false;

      if (gpio_active) {
        bottom_found = rt.limit_sensor_seen;
      } else {
        if (cfg.stall_torque > 0.0 &&
            std::abs(hw_states_efforts_[i]) >= cfg.stall_torque)
        {
          bottom_found = true;
        }
        if (!bottom_found &&
            std::chrono::steady_clock::now() - rt.phase_started >= cfg.phase_timeout)
        {
          RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                      "Joint %zu: FIND_ROOT timed out, assuming bottom reached", i);
          bottom_found = true;
        }
      }
      // Overflow backstop applies regardless of detection method.
      if (!bottom_found &&
          std::abs(hw_states_positions_[i]) >= overflow_thresh_units)
      {
        RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "Joint %zu: encoder near int16 overflow during FIND_ROOT",
                    i);
        bottom_found = true;
      }

      if (bottom_found) {
        // Stop motion immediately and transition.
        can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, ZEROCMD, 4);
        set_phase(i, CalibrationPhase::SET_BOTTOM_ZERO);
        return std::numeric_limits<double>::quiet_NaN();
      }

      // Otherwise advance the setpoint by one search step (paced).
      if (now - rt.last_step >= cfg.step_period) {
        rt.commanded_setpoint += -dir_up * cfg.search_step;
        if(std::abs(rt.commanded_setpoint) > hardware_limits_[i].range)
        {
          rt.commanded_setpoint = -dir_up * hardware_limits_[i].range;
        }
        rt.last_step = now;
      }
      return rt.commanded_setpoint;
    }

    case CalibrationPhase::SET_BOTTOM_ZERO: {
      // First entry into this phase: issue the zero command and return.
      // Subsequent entries are gated by the settle-window check at the top
      // of step_calibration; only after settle elapses do we reach here
      // again and do the actual phase-exit work.
      if (rt.zero_issued < rt.phase_started) {
        issue_zero_command(i);
        return std::numeric_limits<double>::quiet_NaN();
      }
      // Settle elapsed — encoder now reads ~0 at the bottom.
      if (!zero_at_midpoint_[i]) {
        // Bottom-only strategy: the hard stop is the operational zero. enc_offs_
        // stays 0 (reported == raw), and the lift is already here, so there is
        // no second move — calibration is complete. Trades int16 headroom (raw
        // spans [0, range] instead of +/- half_range) for a shorter, safer run.
        enc_offs_[i] = 0.0;
        motor_msgs_[i].is_calibrated = true;
        motor_msgs_[i].calibrate = false;
        set_phase(i, CalibrationPhase::DONE);
        RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "Joint %zu: calibration DONE (bottom zero, enc_off=0.0)", i);
        return std::numeric_limits<double>::quiet_NaN();
      }
      // Midpoint strategy: stage the midpoint setpoint and keep centering.
      rt.commanded_setpoint = dir_up * half_range;
      set_phase(i, CalibrationPhase::FIND_MIDSECTION);
      return std::numeric_limits<double>::quiet_NaN();
    }

    case CalibrationPhase::SET_RETRY_ZERO: {
      if (rt.zero_issued < rt.phase_started) {
        issue_zero_command(i);
        return std::numeric_limits<double>::quiet_NaN();
      }
      // After settle, the re-origined encoder reads ~0 at the current position.
      // Re-seed the setpoint from that fresh reading before resuming the search;
      // otherwise FIND_ROOT would resume from the stale (out-of-range) setpoint
      // and trip the range check again immediately, burning every retry without
      // moving. enc_offs_ is 0 during calibration, so raw == reported * dir_up.
      rt.commanded_setpoint = std::isnan(hw_states_positions_[i])
        ? 0.0
        : hw_states_positions_[i] * dir_up;
      set_phase(i, CalibrationPhase::FIND_ROOT);
      return std::numeric_limits<double>::quiet_NaN();
    }

    case CalibrationPhase::FIND_MIDSECTION: {
      // Drive to mid in the raw frame. enc_offs_ is still 0 here so reported
      // position == unit converted position.
      const double err = std::abs(rt.commanded_setpoint) - std::abs(hw_states_positions_[i]);
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"), *node_->get_clock(), 500,
                  "Commanded pos: %f Cur pos: %f Err: %f", rt.commanded_setpoint, hw_states_positions_[i], err);
      if (std::abs(err) < cfg.position_tolerance) {
        set_phase(i, CalibrationPhase::SET_MID_ZERO);
        return std::numeric_limits<double>::quiet_NaN();
      }
      
      // Phase timeout protects against the lift never reaching mid.
      if (now - rt.phase_started >= cfg.phase_timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                     "Joint %zu: FIND_MIDSECTION timed out (err=%.4f)", i, err);
        set_phase(i, CalibrationPhase::FAILED);
        return std::numeric_limits<double>::quiet_NaN();
      }
      return rt.commanded_setpoint;
    }

    case CalibrationPhase::SET_MID_ZERO: {
      // First entry: issue the zero command and return. Subsequent entries
      // gated by settle window (see SET_BOTTOM_ZERO).
      if (rt.zero_issued < rt.phase_started) {
        issue_zero_command(i);
        return std::numeric_limits<double>::quiet_NaN();
      }
      // Compute the offset. After zeroing at the midpoint, raw encoder == 0
      // physically corresponds to the midpoint. We want reported position
      // == 0 at the *bottom*, so reported = raw - offset, and at the bottom:
      //   raw at bottom = -dir_up * half_range
      //   reported = 0  =>  offset = -dir_up * half_range
      enc_offs_[i] = -dir_up * half_range;
      // Drive to the bottom in the raw frame; reported position will then
      // become 0 as we arrive.
      rt.commanded_setpoint = 0.0;
      set_phase(i, CalibrationPhase::RETURN_TO_HOME);
      return std::numeric_limits<double>::quiet_NaN();
    }

    case CalibrationPhase::RETURN_TO_HOME: {
      // hw_states_positions_[i] is in the reported (offset-applied) frame
      // at this point. We want reported ≈ 0 (the bottom = operational zero).
      const double err = hw_states_positions_[i];  // target is 0
      if (std::abs(err) < cfg.position_tolerance) {
        motor_msgs_[i].is_calibrated = true;
        motor_msgs_[i].calibrate = false;
        set_phase(i, CalibrationPhase::DONE);
        RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "Joint %zu: calibration DONE (enc_off=%.4f)", i, enc_offs_[i]);
        return std::numeric_limits<double>::quiet_NaN();
      }
      if (now - rt.phase_started >= cfg.phase_timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                     "Joint %zu: RETURN_TO_HOME timed out (err=%.4f)", i, err);
        set_phase(i, CalibrationPhase::FAILED);
        return std::numeric_limits<double>::quiet_NaN();
      }
      // Target in raw frame == bottom == -dir_up*half_range, but the
      // POSITION_SPEED_LOOP write path adds enc_offs_ to the input, so we
      // pass the reported-frame target (0.0) and let it be re-offset.
      return 0.0;
    }

    case CalibrationPhase::DONE:
    case CalibrationPhase::FAILED:
    case CalibrationPhase::DEFAULT:
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

// ============================================================================
// read()
// ============================================================================
hardware_interface::return_type CubeMarsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const bool offline = (lift_power_state_.load() == LiftPowerState::OFFLINE);

  std::vector<bool> all_ids(can_ids_.size(), false);
  std::uint32_t read_id;
  std::uint8_t read_data[8];
  std::uint8_t read_len;

  while (can_.read_nonblocking(read_id, read_data, read_len)) {
    if (read_data[7] != 0) {
      switch (read_data[7]) {
        case 1: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor over-temperature fault."); break;
        case 2: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-current fault."); break;
        case 3: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-voltage fault."); break;
        case 4: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Under-voltage fault."); break;
        case 5: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Encoder fault."); break;
        case 6: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "MOSFET over-temperature fault."); break;
        case 7: RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor stall."); break;
        default: break;
      }
    }
    auto it = std::find(can_ids_.begin(), can_ids_.end(), read_id);
    if (it == can_ids_.end()) continue;

    const std::size_t i = std::distance(can_ids_.begin(), it);
    all_ids[i] = true;

    // A firmware fault (byte 7 != 0: over-current, stall, etc.) during
    // calibration means the motor has likely cut out. Its effort reading then
    // collapses to ~0, so FIND_ROOT's stall-torque detection can never fire and
    // the phase would grind out its full timeout while jammed. Fail fast by
    // driving the joint to FAILED; write()'s FAILED handler stops motion,
    // disables, and clears the calibration latch.
    if (read_data[7] != 0 && is_calibration_running_.load() &&
        motor_msgs_[i].calibrate &&
        calibration_rt_[i].phase != CalibrationPhase::FAILED)
    {
      RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                   "Joint %zu: motor fault (code %u) during calibration; aborting",
                   i, read_data[7]);
      set_phase(i, CalibrationPhase::FAILED);
    }

    last_telemetry_[i] = std::chrono::steady_clock::now();
    if (offline) {
      good_frames_since_offline_[i]++;
    }

    const std::int16_t pos_raw  = static_cast<std::int16_t>((read_data[0] << 8) | read_data[1]);
    const std::int16_t vel_raw  = static_cast<std::int16_t>((read_data[2] << 8) | read_data[3]);
    const std::int16_t curr_raw = static_cast<std::int16_t>((read_data[4] << 8) | read_data[5]);
    const std::uint8_t temp_raw = read_data[6];

    if(!offline)
    {
      // Unit conversions ------------------------------------------------------
      // Position: pos_raw is in centidegrees (0.1° per LSB) at the output shaft.
      //   raw → output radians: pos_raw * 0.1 * π/180
      //   If use_meters_: multiply by m_per_rad_ (lead-screw factor).
      //   Then subtract enc_offs_, which is in output units (m or rad).
      const double dir_up = mount_dir_[i] ? +1.0 : -1.0;
      const double pos_rad = pos_raw * 0.1 * M_PI / 180.0;
      const double pos_output = use_meters_ ? (pos_rad * m_per_rad_[i]) : pos_rad;
      hw_states_positions_[i] = (pos_output - enc_offs_[i]) * dir_up;
      // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"), *node_->get_clock(), 1000,
      //               "Joint %zu: pos-output: %f enc_offs: %f pos-hdw: %f", i, pos_output, enc_offs_[i], hw_states_positions_[i]);
  
      // Velocity: vel_raw is in ERPM (electrical RPM) at the motor.
      //   ERPM / erpm_conversion = output rad/s. No extra factor of 10.
      //   If use_meters_, scale rad/s → m/s with the lead-screw factor.
      const double vel_rad_s = vel_raw / erpm_conversions_[i];
      hw_states_velocities_[i] =
        use_meters_ ? (vel_rad_s * m_per_rad_[i]) : vel_rad_s;
  
      // Effort: raw is centi-amps; multiply by Kt and gear ratio.
      const int gear_ratio = std::stoi(info_.joints[i].parameters.at("gear_ratio"));
      hw_states_efforts_[i] = curr_raw * 0.01 * torque_constants_[i] * gear_ratio;
      // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"),*node_->get_clock(), 1000,
      //               "Raw Current %f: Effort: %f", (curr_raw*0.01), hw_states_efforts_[i]);
  
      hw_states_temperatures_[i] = static_cast<double>(temp_raw);

      // ---- Torque-limit handling (operational, NOT calibration) ----
      // Calibration's own stall threshold is checked inside step_calibration's
      // FIND_ROOT branch; the operational trq_limit applies only when *not*
      // calibrating to avoid double-handling.
      if (trq_limits_[i] != 0 &&
          std::abs(hw_states_efforts_[i]) > trq_limits_[i] &&
          !(is_calibration_running_ && motor_msgs_[i].calibrate))
      {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "Joint %zu went over torque limit (%f > %f), disabling.",
                    i, hw_states_efforts_[i], trq_limits_[i]);
        can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, ZEROCMD, 4);
        motor_msgs_[i].get_motor_state = MotorCommandMsg::DISABLE;
        enter_safe_state(true);
      }

      motor_msgs_[i].at_lower_limit =
        (i < 64) && (((limit_active_mask_.load() >> i) & 1ULL) != 0ULL);
      motor_msg_grp_.commands.push_back(motor_msgs_[i]);
    }
  }

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (!all_ids[i]) {
      RCLCPP_WARN_ONCE(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "No CAN message received from CAN ID: %u.", can_ids_[i]);
    } else if (read_only_[i]) {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"), *node_->get_clock(), 1000,
                  "Joint %zu: pos: %f", i, hw_states_positions_[i]);
    }
  }

  update_power_state();

  if(!motor_msg_grp_.commands.empty()) {
    motor_msg_grp_.header.stamp = node_->get_clock()->now();
    rt_state_publisher_->try_publish(motor_msg_grp_);
    motor_msg_grp_.commands.clear();
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// write()
// ============================================================================
hardware_interface::return_type CubeMarsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // While OFFLINE: drop the mailbox if anything's in it, don't send CAN.
  // Returning OK keeps the controller manager happy and avoids the lifecycle
  // dance that ERROR would force.
  if (lift_power_state_.load() == LiftPowerState::OFFLINE) {
    if (has_request_.load()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"),
                           *node_->get_clock(), 2000,
                           "Lift OFFLINE: dropping service request");
      has_request_.store(false);
    }
    return hardware_interface::return_type::OK;
  }

  // ---- Consume mailbox once per request ----
  // Order matters: set is_calibration_running_ BEFORE clearing has_request_
  // so other readers don't see a brief window with neither flag set.
  if (has_request_.load()) {
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Has Request");
    std::vector<MotorCommandMsg> latest_cmd = command_mailbox_.get();
    bool any_calibration = false;
    bool any_state_change = false;

    // Map each command to its joint by CAN id (not list position), so a request
    // may target a subset of joints in any order.
    for (const auto & cmd : latest_cmd) {
      const std::size_t i = joint_index_for_can_id(cmd.can_id);
      if (i >= info_.joints.size()) {
        // Unknown CAN id. The service callback rejects these up front, so this
        // is only a defensive guard against a malformed mailbox entry.
        RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "Dropping command for unknown CAN id %u", cmd.can_id);
        continue;
      }

      if (cmd.calibrate && calibration_cfg_[i].enabled) {
        // Never auto-move a disabled/faulted joint. If it isn't ENABLE, skip
        // WITHOUT latching is_calibration_running_ (the per-joint loop below
        // skips disabled joints, so a latched calibration would never progress
        // and would wedge the service). Backstops the same check in the service
        // callback against a race on get_motor_state.
        if (motor_msgs_[i].get_motor_state != MotorCommandMsg::ENABLE) {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "Joint %zu: cannot calibrate while disabled; send "
                       "set_motor_state=ENABLE first", i);
        } else {
          enter_calibration(i);
          any_calibration = true;
        }
      } else {
        motor_msgs_[i].set_motor_state = cmd.set_motor_state;
        if (cmd.set_motor_state > 0) any_state_change = true;
      }
    }

    if (any_calibration) is_calibration_running_.store(true);
    if (any_state_change) is_changing_state_.store(true);
    has_request_.store(false);
  }

  // ---- Per-joint command emission ----
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (read_only_[i] || motor_msgs_[i].get_motor_state != MotorCommandMsg::ENABLE) {
      // Check if state change to ENABLE requested
      if (is_changing_state_.load() &&
          motor_msgs_[i].set_motor_state == MotorCommandMsg::ENABLE)
      {
        motor_msgs_[i].get_motor_state = MotorCommandMsg::ENABLE;
        // Transition applied; clear the pending request so is_changing_state_
        // can latch down (0 == no pending state change).
        motor_msgs_[i].set_motor_state = 0;
      }
      continue;
    }

    // Calibration in progress for this joint? It overrides whatever the
    // controller is commanding.
    const bool joint_calibrating =
      is_calibration_running_.load() && motor_msgs_[i].calibrate;
    // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"), *node_->get_clock(), 1000,
    //               "User req: %b Trigger: %b Jt Calib flag: %b", motor_msgs_[i].calibrate, is_calibration_running_.load(), joint_calibrating);

    if (joint_calibrating) {
      // A phase already marked FAILED (e.g. by read()'s fault handler) aborts
      // immediately, independent of control mode.
      if (calibration_rt_[i].phase == CalibrationPhase::FAILED) {
        abort_calibration_failed(i);
        continue;
      }
      // Calibration runs ONLY in POSITION_SPEED_LOOP. If the controller hasn't
      // claimed position interfaces, we cannot safely move the joint; mark it
      if (control_mode_[i] != POSITION_SPEED_LOOP &&
          control_mode_[i] != POSITION_LOOP)
      {
        // Wait for the position controller to claim interfaces, but BOUND the
        // wait: a controller that never enters position mode must not hang
        // calibration forever. Track when the wait began and FAIL after
        // phase_timeout. While waiting, keep phase_started fresh so the
        // FIND_ROOT timer starts clean once position mode is acquired.
        const auto now = std::chrono::steady_clock::now();
        if (calibration_rt_[i].mode_wait_started.time_since_epoch().count() == 0) {
          calibration_rt_[i].mode_wait_started = now;
        }
        if (now - calibration_rt_[i].mode_wait_started >= calibration_cfg_[i].phase_timeout) {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "Joint %zu: timed out waiting for position control mode "
                       "(current=%d); aborting calibration", i, control_mode_[i]);
          set_phase(i, CalibrationPhase::FAILED);
          abort_calibration_failed(i);
          continue;
        }
        calibration_rt_[i].phase_started = now;
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          *node_->get_clock(), 2000,
          "Joint %zu: calibration waiting for position control mode "
          "(current=%d)", i, control_mode_[i]);
        continue;
      }
      else {
        // Position mode acquired; clear the mode-wait clock.
        calibration_rt_[i].mode_wait_started = {};
        const double cal_cmd = step_calibration(i);

        // FAILED terminates this joint: stop motion, mark uncalibrated, disable.
        if (calibration_rt_[i].phase == CalibrationPhase::FAILED) {
          abort_calibration_failed(i);
          continue;
        }

        if (!std::isnan(cal_cmd)) {
          // Build a position-speed-loop frame. `cal_cmd` is in reported-frame
          // units (m or rad); convert through enc_offs_ then to centidegrees*1e2.
          // cal_cmd is in output units (m or rad). Add offset in output units,
          // convert to output radians via the lead-screw factor (if meters),
          // then scale to wire centidegree-LSB (× 1e4 × 180/π).
          double cmd_with_off = cal_cmd + enc_offs_[i];
          if (use_meters_) cmd_with_off /= m_per_rad_[i];  // now in rad

          // RCLCPP_INFO_THROTTLE(
          //   rclcpp::get_logger("CubeMarsSystemHardware"),
          //   *node_->get_clock(), 1000,
          //   "Joint %zu: exec command - cal_cmd %f: cmd_w_off %f enc_off %f", i, cal_cmd, cmd_with_off, enc_offs_[i]);
          const std::int32_t position =
            static_cast<std::int32_t>(cmd_with_off * 10000.0 * 180.0 / M_PI);
            
          if (std::abs(position) >= 360000000) {
            RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "command %f: enc_off %f: cmd_w_enc %f: m_per_rad %f", cal_cmd, enc_offs_[i], cmd_with_off, m_per_rad_[i]);
            RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "Joint %zu: calibration position command out of range: %d",
                         i, position);
            if (calibration_rt_[i].retry_count >= global_cfg_.max_retries) {
              set_phase(i, CalibrationPhase::FAILED);
            } else {
              calibration_rt_[i].retry_count += 1;
              RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "Joint %zu: calibration retry %u of %u", i,
                         calibration_rt_[i].retry_count, global_cfg_.max_retries);
              set_phase(i, CalibrationPhase::SET_RETRY_ZERO);
            }
            continue;
          }
          const std::int16_t vel = limits_[i].first;
          const std::int16_t acc = limits_[i].second;
          std::uint8_t data[8];
          data[0] = position >> 24; data[1] = position >> 16;
          data[2] = position >> 8;  data[3] = position;
          data[4] = vel >> 8;       data[5] = vel;
          data[6] = acc >> 8;       data[7] = acc;
          can_.write_message(can_ids_[i] | POSITION_SPEED_LOOP << 8, data, 8);
        }
        continue;
      }
    }

    // Pending DISABLE from a state-change request?
    if (is_changing_state_.load() &&
        motor_msgs_[i].set_motor_state == MotorCommandMsg::DISABLE)
    {
      can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, ZEROCMD, 4);
      motor_msgs_[i].get_motor_state = MotorCommandMsg::DISABLE;
      // Transition applied; clear the pending request. Without this, the
      // latch-down check (which looks for set_motor_state == DISABLE) would
      // find it set forever and is_changing_state_ would never clear, wedging
      // the service on "Motor state change in progress".
      motor_msgs_[i].set_motor_state = 0;
      continue;
    }

    // Reject normal commands to uncalibrated joints that require calibration.
    if (!motor_msgs_[i].is_calibrated && calibration_cfg_[i].enabled) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        *node_->get_clock(), 5000,
        "Joint %zu: ignoring command, not calibrated", i);
      continue;
    }

    // ---- Normal operation ----
    // Lower-limit protective stop: while the (debounced) limit sensor is active,
    // inhibit commands that would drive the joint further toward the limit
    // (downward), evaluated in the controller/reported command frame — a
    // decreasing position, negative velocity, or negative effort. Motion away
    // from the limit passes unchanged. Non-latching: clears with the sensor.
    const bool at_lower_limit =
      (i < 64) && (((limit_active_mask_.load() >> i) & 1ULL) != 0ULL);

    switch (control_mode_[i])
    {
      case UNDEFINED:
        break;

      case CURRENT_LOOP: {
        if (std::isnan(hw_commands_efforts_[i])) break;
        double eff_cmd = clamp_downward_at_lower_limit(hw_commands_efforts_[i], at_lower_limit);
        std::int32_t current =
          static_cast<std::int32_t>(eff_cmd * 1000.0 / torque_constants_[i]);
        if (std::abs(current) >= 60000) {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "current command out of range: %d", current);
          return hardware_interface::return_type::ERROR;
        }
        std::uint8_t data[4]{
          static_cast<std::uint8_t>(current >> 24),
          static_cast<std::uint8_t>(current >> 16),
          static_cast<std::uint8_t>(current >> 8),
          static_cast<std::uint8_t>(current)};
        can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, data, 4);
        break;
      }

      case SPEED_LOOP: {
        if (std::isnan(hw_commands_velocities_[i])) break;
        double vel_cmd = clamp_downward_at_lower_limit(hw_commands_velocities_[i], at_lower_limit);
        if (use_meters_) vel_cmd /= m_per_rad_[i];  // m/s → rad/s at output shaft
        std::int32_t speed = static_cast<std::int32_t>(vel_cmd * erpm_conversions_[i]);
        if (std::abs(speed) >= 100000) {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "speed command out of range: %d", speed);
          return hardware_interface::return_type::ERROR;
        }
        std::uint8_t data[4]{
          static_cast<std::uint8_t>(speed >> 24),
          static_cast<std::uint8_t>(speed >> 16),
          static_cast<std::uint8_t>(speed >> 8),
          static_cast<std::uint8_t>(speed)};
        can_.write_message(can_ids_[i] | SPEED_LOOP << 8, data, 4);
        break;
      }

      case POSITION_LOOP: {
        if (std::isnan(hw_commands_positions_[i])) break;
        double pos_cmd = clamp_position_at_lower_limit(
          hw_commands_positions_[i], hw_states_positions_[i], at_lower_limit);
        // Add the offset in output units, convert to output radians via the
        // lead-screw factor (if meters), then scale to wire centidegree-LSB.
        const double dir_up = mount_dir_[i] ? +1.0 : -1.0;
        double cmd_with_off = (dir_up * pos_cmd) + enc_offs_[i];
        if (use_meters_) cmd_with_off /= m_per_rad_[i];  // now in rad
        const std::int32_t position =
          static_cast<std::int32_t>(cmd_with_off * 10000.0 * 180.0 / M_PI);
        if (std::abs(position) >= 360000000) {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "position command out of range: %d", position);
          return hardware_interface::return_type::ERROR;
        }
        std::uint8_t data[4]{
          static_cast<std::uint8_t>(position >> 24),
          static_cast<std::uint8_t>(position >> 16),
          static_cast<std::uint8_t>(position >> 8),
          static_cast<std::uint8_t>(position)};
        can_.write_message(can_ids_[i] | POSITION_LOOP << 8, data, 4);
        break;
      }

      case POSITION_SPEED_LOOP: {
        if (std::isnan(hw_commands_positions_[i])) break;
        const double dir_up = mount_dir_[i] ? +1.0 : -1.0;
        double cmd = hw_commands_positions_[i];

        // Operational clamp: post-calibration, valid range is [0, range].
        if (motor_msgs_[i].is_calibrated && hardware_limits_[i].range > 0.0) {
          if (cmd < 0.0) cmd = 0.0;
          if (cmd > hardware_limits_[i].range) cmd = hardware_limits_[i].range;
        }
        // At the lower limit, do not command below the current position.
        cmd = clamp_position_at_lower_limit(cmd, hw_states_positions_[i], at_lower_limit);
        cmd = cmd * dir_up;
        // Add the offset in output units, convert to output radians via the
        // lead-screw factor (if meters), then scale to wire centidegree-LSB.
        double cmd_with_off = cmd + enc_offs_[i];
        if (use_meters_) cmd_with_off /= m_per_rad_[i];  // now in rad

        // RCLCPP_INFO_THROTTLE(
        // rclcpp::get_logger("CubeMarsSystemHardware"),
        // *node_->get_clock(), 1000,
        // "Joint %zu: exec command - cmd %f: cmd_w_off %f enc_off %f", i, cmd, cmd_with_off, enc_offs_[i]);

        const std::int32_t position =
          static_cast<std::int32_t>(cmd_with_off * 10000.0 * 180.0 / M_PI);
        if (std::abs(position) >= 360000000) {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "position command out of range: %d", position);
          return hardware_interface::return_type::ERROR;
        }
        const std::int16_t vel = limits_[i].first;
        const std::int16_t acc = limits_[i].second;
        std::uint8_t data[8];
        data[0] = position >> 24; data[1] = position >> 16;
        data[2] = position >> 8;  data[3] = position;
        data[4] = vel >> 8;       data[5] = vel;
        data[6] = acc >> 8;       data[7] = acc;
        can_.write_message(can_ids_[i] | POSITION_SPEED_LOOP << 8, data, 8);
        // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CubeMarsSystemHardware"),*node_->get_clock(), 3000,
        //           "Raw Position %zu: Vel: %zu Acc: %zu", position, vel, acc);
        break;
      }

      case SET_ORIGIN_MODE:
        break;
    }
  }

  // ---- Latch global flags down once all joints finish ----
  if (is_calibration_running_.load()) {
    bool any_pending = false;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (motor_msgs_[i].calibrate &&
          calibration_rt_[i].phase != CalibrationPhase::DONE &&
          calibration_rt_[i].phase != CalibrationPhase::FAILED)
      {
        any_pending = true;
        break;
      }
    }
    if (!any_pending) {
      is_calibration_running_.store(false);
      // Summarize.
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
        if (!calibration_cfg_[i].enabled) continue;
        RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                    "Calibration summary -- joint %zu: %s (offset=%.4f)",
                    i, to_string(calibration_rt_[i].phase), enc_offs_[i]);
      }
    }
  }

  if (is_changing_state_.load()) {
    bool any_pending = false;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (motor_msgs_[i].set_motor_state == MotorCommandMsg::DISABLE) {
        any_pending = true;
        break;
      }
    }
    if (!any_pending) is_changing_state_.store(false);
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Service callback (non-RT thread)
// ============================================================================
void CubeMarsSystemHardware::motor_control_callback(
  const std::shared_ptr<MotorControlServiceRequest> request,
  std::shared_ptr<MotorControlServiceResponse> response)
{
  // Reject any commands when Lift is OFFLINE
   if (lift_power_state_.load() == LiftPowerState::OFFLINE) {
    response->success = false;
    response->message = "Lift is OFFLINE (power lost)";
    return;
  }

  if (has_request_.load() || is_calibration_running_.load() ||
      is_changing_state_.load())
  {
    response->success = false;
    response->message = is_calibration_running_.load()
      ? "Calibration in progress"
      : (is_changing_state_.load() ? "Motor state change in progress"
                                   : "Previous request still pending");
    return;
  }

  if (request->commands.empty()) {
    response->success = false;
    response->message = "Empty request: no commands";
    return;
  }
  if (request->commands.size() > info_.joints.size()) {
    response->success = false;
    response->message = "Invalid request: more commands than joints";
    return;
  }

  // Validate every command up front and reject with a specific message so the
  // caller learns exactly why a request can't be fulfilled. Commands are keyed
  // by CAN id, not list position.
  bool trigger_calibration = false;
  bool has_state_change_req = false;
  for (const auto & cmd : request->commands) {
    const std::size_t j = joint_index_for_can_id(cmd.can_id);
    if (j >= info_.joints.size()) {
      response->success = false;
      response->message =
        "No joint configured for CAN id " + std::to_string(cmd.can_id);
      return;
    }

    if (cmd.calibrate) {
      if (!calibration_cfg_[j].enabled) {
        response->success = false;
        response->message =
          "Joint (CAN " + std::to_string(cmd.can_id) +
          ") is not eligible for calibration";
        return;
      }
      // Advisory: reject calibrating a disabled joint with a clear message.
      // get_motor_state is owned by the RT write() thread; a stale read here is
      // harmless because write() re-checks race-free before calibrating.
      if (motor_msgs_[j].get_motor_state != MotorCommandMsg::ENABLE) {
        response->success = false;
        response->message =
          "Joint (CAN " + std::to_string(cmd.can_id) +
          ") is disabled; send set_motor_state=ENABLE before calibrating";
        return;
      }
      trigger_calibration = true;
    } else if (cmd.set_motor_state > 0) {
      if (cmd.set_motor_state != MotorCommandMsg::ENABLE &&
          cmd.set_motor_state != MotorCommandMsg::DISABLE) {
        response->success = false;
        response->message =
          "Invalid set_motor_state for CAN " + std::to_string(cmd.can_id) +
          " (use ENABLE=10 or DISABLE=11)";
        return;
      }
      has_state_change_req = true;
    }

    if (has_state_change_req && trigger_calibration) {
      response->success = false;
      response->message = "Cannot mix calibrate and state-change in one request";
      return;
    }
  }

  if (!has_state_change_req && !trigger_calibration) {
    response->success = false;
    response->message = "Unknown request: no calibrate or state-change set";
    return;
  }

  command_mailbox_.set(request->commands);
  has_request_.store(true);

  response->success = true;
  response->message = trigger_calibration ? "Starting calibration" : "Changing motor state";
}

// ============================================================================
// GPIO message processing — updates per-joint limit-sensor latches.
// ============================================================================
void CubeMarsSystemHardware::process_gpio_message(const ControlMessage & msg)
{
  for (std::size_t g = 0; g < msg.interface_groups.size(); ++g) {
    const std::string & grp_name = msg.interface_groups[g];
    const auto & ifc_values = msg.interface_values[g];
    
    // Limit Sensor Handling (debounced activation, immediate deactivation)
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & cfg = calibration_cfg_[i];
      if (cfg.gpio_group_name.empty() || cfg.gpio_ifc_name.empty()) continue;
      if (cfg.gpio_group_name != grp_name) continue;

      for (std::size_t t = 0; t < ifc_values.interface_names.size(); ++t) {
        if (ifc_values.interface_names[t] != cfg.gpio_ifc_name) continue;

        // > 0.5 means "pressed/high". Assert the limit only after
        // limit_debounce_frames consecutive active readings (filters bounce);
        // clear immediately on the first inactive reading so motion away from
        // the limit is never held back.
        const std::uint64_t bit = (i < 64) ? (1ULL << i) : 0ULL;
        const bool asserted = limit_debounce_update(
          limit_active_count_[i], global_cfg_.limit_debounce_frames,
          ifc_values.values[t] > 0.5);
        if (asserted) {
          limit_active_mask_.fetch_or(bit);
          // Debounced trigger also feeds calibration bottom detection
          // (per-phase latch, reset in set_phase()).
          if (is_calibration_running_.load() && motor_msgs_[i].calibrate) {
            calibration_rt_[i].limit_sensor_seen = true;
          }
        } else {
          limit_active_mask_.fetch_and(~bit);
        }
      }
    }

    // Hardware-wide power signal.
    if (!global_cfg_.gpio_power_group_name.empty() &&
        global_cfg_.gpio_power_group_name == grp_name)
    {
      for (std::size_t t = 0; t < ifc_values.interface_names.size(); ++t) {
        if (ifc_values.interface_names[t] == global_cfg_.gpio_power_ifc_name) {
          gpio_power_seen_high_.store(ifc_values.values[t] > 0.5);
        }
      }
    }
  }
}

void CubeMarsSystemHardware::enter_offline_state()
{
  // Clear per-joint state that's invalidated by power loss. We do NOT touch
  // motor_msgs_[i].can_id or set_origin_payload_ etc. - those are static
  // hardware config.
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    motor_msgs_[i].is_calibrated = false;
    motor_msgs_[i].calibrate = false;
    enc_offs_[i] = 0.0;
    // Drop any in-progress calibration runtime.
    calibration_rt_[i] = CalibrationRuntime{};
    good_frames_since_offline_[i] = 0;
    // NaN out the reported state so downstream sees the unknown.
    hw_states_positions_[i]    = std::numeric_limits<double>::quiet_NaN();
    hw_states_velocities_[i]   = std::numeric_limits<double>::quiet_NaN();
    hw_states_efforts_[i]      = std::numeric_limits<double>::quiet_NaN();
    hw_states_temperatures_[i] = std::numeric_limits<double>::quiet_NaN();
  }
  // Drop any pending service request - it was issued under the previous
  // power-on epoch and is no longer meaningful.
  has_request_.store(false);
  is_calibration_running_.store(false);
  is_changing_state_.store(false);

  // Unset STOP LIFT bit
  enter_safe_state(false);

  RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
              "Lift OFFLINE: clearing calibration, dropping pending commands");
}

void CubeMarsSystemHardware::enter_recovering_state()
{
  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
              "Lift RECOVERING: telemetry resumed");
  if (!global_cfg_.auto_recalibrate_on_power_restore) {
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                "Auto-recalibrate disabled; awaiting service request to "
                "trigger calibration");
    return;
  }
  // Synthesize a calibration request just like on_activate does.
  std::vector<MotorCommandMsg> auto_cmd(info_.joints.size());
  bool any = false;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    auto_cmd[i].can_id = motor_msgs_[i].can_id;
    auto_cmd[i].set_motor_state = 0;
    if (calibration_cfg_[i].enabled) {
      auto_cmd[i].calibrate = true;
      any = true;
    }
  }
  if (any) {
    command_mailbox_.set(auto_cmd);
    has_request_.store(true);
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                "Auto-recalibration queued");
  }
}

void CubeMarsSystemHardware::update_power_state()
{
  const auto now = std::chrono::steady_clock::now();
  const LiftPowerState prev = lift_power_state_.load();

  // GPIO check (primary). When disabled (no name configured) we treat the
  // signal as "always high" so only telemetry timeout drives the decision.
  const bool gpio_check_enabled =
    !global_cfg_.gpio_power_group_name.empty();
  const bool gpio_high =
    !gpio_check_enabled || gpio_power_seen_high_.load();

  // Telemetry check (fallback): any joint had a frame within timeout?
  bool any_recent_telemetry = false;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (last_telemetry_[i].time_since_epoch().count() == 0) continue;
    if (now - last_telemetry_[i] < global_cfg_.power_loss_timeout) {
      any_recent_telemetry = true;
      break;
    }
  }

  LiftPowerState next = prev;
  switch (prev) {
    case LiftPowerState::ONLINE:
      if (!gpio_high || !any_recent_telemetry) {
        next = LiftPowerState::OFFLINE;
      }
      break;
    case LiftPowerState::OFFLINE:
      // To leave OFFLINE we need BOTH GPIO high AND telemetry resumption.
      if (gpio_high && any_recent_telemetry) {
        // Debounce on min_telemetry_frames_to_resume across joints.
        bool enough = true;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
          if (good_frames_since_offline_[i] <
              global_cfg_.min_telemetry_frames_to_resume)
          {
            enough = false;
            break;
          }
        }
        if (enough) next = LiftPowerState::RECOVERING;
      }
      break;
    case LiftPowerState::RECOVERING:
      // Drop back to OFFLINE if signals regress.
      if (!gpio_high || !any_recent_telemetry) {
        next = LiftPowerState::OFFLINE;
        break;
      }
      // Promote to ONLINE once every enabled joint is calibrated.
      {
        bool all_ok = true;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
          if (calibration_cfg_[i].enabled && !motor_msgs_[i].is_calibrated) {
            all_ok = false;
            break;
          }
        }
        if (all_ok) next = LiftPowerState::ONLINE;
      }
      break;
  }

  if (next != prev) {
    RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                "Lift power state: %s -> %s",
                to_string(prev), to_string(next));
    lift_power_state_.store(next);
    if (next == LiftPowerState::OFFLINE) enter_offline_state();
    else if (next == LiftPowerState::RECOVERING) enter_recovering_state();
  }
}

void CubeMarsSystemHardware::enter_safe_state(const bool &set)
{
  if(!global_cfg_.gpio_stop_lift_group_name.empty() &&
      !global_cfg_.gpio_stop_lift_group_name.empty())
  {
    // Create a message to the safety sequence.
    auto dynamic_interface_group_values_msg = ControlMessage();
    dynamic_interface_group_values_msg.header.stamp = get_clock()->now();
    dynamic_interface_group_values_msg.interface_groups.push_back(
          global_cfg_.gpio_stop_lift_group_name);
    auto interface_value_msg = control_msgs::msg::InterfaceValue();
    interface_value_msg.interface_names.push_back(global_cfg_.gpio_stop_lift_ifc_name);
    interface_value_msg.values.push_back((set)? 1 : 0);
    dynamic_interface_group_values_msg.interface_values.push_back(interface_value_msg);
    // Publish the command (RT).
    rt_pub_gpio_command_->try_publish(dynamic_interface_group_values_msg);
  }
}

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware::CubeMarsSystemHardware, hardware_interface::SystemInterface)