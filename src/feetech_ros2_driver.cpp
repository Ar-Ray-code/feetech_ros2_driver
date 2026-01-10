#include <fmt/ranges.h>

#include <algorithm>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace feetech_ros2_driver {
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    return CallbackReturn::ERROR;
  }

  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "FeetechHardware::on_init Hardware parameter [{}] not found!. "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }
  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);

  const bool skip_probe = [&]() {
    const auto it = info_.hardware_parameters.find("skip_probe");
    if (it == info_.hardware_parameters.end()) return false;
    const auto& v = it->second;
    return v == "1" || v == "true" || v == "True" || v == "TRUE";
  }();

  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("FeetechHardware::on_init -> {}", result.error());
    if (!skip_probe) {
      return CallbackReturn::ERROR;
    }
    spdlog::warn("FeetechHardware::on_init proceeding in skip_probe mode despite serial error");
  }

  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  joint_ids_.resize(info_.joints.size(), 0);
  joint_offsets_.resize(info_.joints.size(), 0);
  last_raw_ticks_.resize(info_.joints.size(), 0);
  joint_min_ticks_.resize(info_.joints.size(), 0);
  joint_max_ticks_.resize(info_.joints.size(), 4095);
  home_rads_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto& joint_params = info_.joints[i].parameters;
    joint_ids_[i] = std::stoi(joint_params.at("id"));
    joint_offsets_[i] = [&] {
      if (const auto offset_it = joint_params.find("offset"); offset_it != joint_params.end()) {
        return std::stoi(offset_it->second);
      }
      spdlog::info("Joint '{}' does not specify an offset parameter - Setting it to 0", info_.joints[i].name);
      return 0;
    }();

    if (const auto rmin_it = joint_params.find("range_min"); rmin_it != joint_params.end()) {
      try {
        joint_min_ticks_[i] = std::stoi(rmin_it->second);
      } catch (...) {
      }
    }
    if (const auto rmax_it = joint_params.find("range_max"); rmax_it != joint_params.end()) {
      try {
        joint_max_ticks_[i] = std::stoi(rmax_it->second);
      } catch (...) {
      }
    }

    if (const auto home_it = joint_params.find("home_rad"); home_it != joint_params.end()) {
      try {
        home_rads_[i] = std::stod(home_it->second);
      } catch (...) {
        home_rads_[i] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    if (!skip_probe) {
      for (const auto& [parameter_name, address] : {std::pair{"p_cofficient", SMS_STS_P_COEF},
                                                    {"d_cofficient", SMS_STS_D_COEF},
                                                    {"i_cofficient", SMS_STS_I_COEF}}) {
        if (const auto param_it = joint_params.find(parameter_name); param_it != joint_params.end()) {
          const auto result = communication_protocol_->write(
              joint_ids_[i], address,
              std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
          if (!result) {
            spdlog::error("FeetechHardwareInterface::on_init -> {}", result.error());
            return CallbackReturn::ERROR;
          }
        }
      }
    }
    // Disable holding torque for joints that do not have command interfaces.
    if (info_.joints[i].command_interfaces.empty()) {
      communication_protocol_->set_torque(joint_ids_[i], false);
    }
  }

  // Optional: auto zero offsets on activate
  if (const auto it = info_.hardware_parameters.find("auto_zero_on_activate"); it != info_.hardware_parameters.end()) {
    const auto& v = it->second;
    auto_zero_on_activate_ = (v == "1" || v == "true" || v == "True" || v == "TRUE");
  }
  if (const auto it = info_.hardware_parameters.find("apply_home_on_activate"); it != info_.hardware_parameters.end()) {
    const auto& v = it->second;
    apply_home_on_activate_ = (v == "1" || v == "true" || v == "True" || v == "TRUE");
  }
  if (const auto it = info_.hardware_parameters.find("disable_position_control"); it != info_.hardware_parameters.end()) {
    const auto& v = it->second;
    disable_position_control_ = (v == "1" || v == "true" || v == "True" || v == "TRUE");
  }

  if (!skip_probe) {
    const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                      return communication_protocol_->read_model_number(id)
                                          .and_then(feetech_driver::get_model_name)
                                          .and_then(feetech_driver::get_model_series);
                                    });

    if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
      spdlog::error("FeetechHardware::on_init [One of the joints has an error]. Input: {}",
                    ranges::views::zip(joint_ids_, joint_model_series));
      return CallbackReturn::ERROR;
    }

    const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

    // TODO: Support other series
    if (ranges::any_of(js, [](const auto& series) { return series != feetech_driver::ModelSeries::kSts; })) {
      spdlog::error("FeetechHardware::on_init [Only STS series is supported]. Input (id, series): {}",
                    ranges::views::zip(joint_ids_, js));
      return CallbackReturn::ERROR;
    }
  } else {
    spdlog::warn("FeetechHardware::on_init skipping model probing and series check (skip_probe=true)");
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
  }

  return command_interfaces;
}

hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // 4 = 2 bytes for position + 2 bytes for speed
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    spdlog::error("FeetechHardwareInterface::read -> {}", result.error());
    return hardware_interface::return_type::ERROR;
  }
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, readings] = values;
    const int raw_ticks = feetech_driver::from_sts(
        feetech_driver::WordBytes{.low = readings[0], .high = readings[1]});
    last_raw_ticks_[index] = raw_ticks;
    // Relative ticks around zero using calibration offset
    int delta = raw_ticks - joint_offsets_[index];
    // Normalize to [-2048, 2048)
    delta = ((delta + 2048) % 4096 + 4096) % 4096 - 2048;
    state_hw_positions_[index] = feetech_driver::to_radians(delta);

    const int raw_speed = feetech_driver::from_sts(
        feetech_driver::WordBytes{.low = readings[2], .high = readings[3]});
    state_hw_velocities_[index] = feetech_driver::to_radians(raw_speed);
  });
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Skip write if position control is disabled (leader mode)
  if (disable_position_control_) {
    return hardware_interface::return_type::OK;
  }

  // Create vectors only for joints that have command interfaces
  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;

  for (uint i = 0; i < info_.joints.size(); i++) {
    // Only include joints with command interfaces
    if (!info_.joints[i].command_interfaces.empty()) {
      commanded_joint_ids.push_back(joint_ids_[i]);
      int rel_ticks = feetech_driver::from_radians(hw_positions_[i]);
      // Normalize relative ticks to [-2048, 2048)
      rel_ticks = ((rel_ticks + 2048) % 4096 + 4096) % 4096 - 2048;
      // Convert to absolute ticks in [0, 4096)
      int abs_ticks = (rel_ticks + joint_offsets_[i]) % 4096;
      if (abs_ticks < 0) abs_ticks += 4096;
      // Clamp to calibrated ticks range if provided
      const int rmin = joint_min_ticks_[i];
      const int rmax = joint_max_ticks_[i];
      if (rmin <= rmax) {
        abs_ticks = std::clamp(abs_ticks, rmin, rmax);
      }
      commanded_positions.push_back(abs_ticks);
      commanded_speeds.push_back(2400);       // Default speed
      commanded_accelerations.push_back(50);  // Default acceleration
    }
  }

  // Only send commands if there are joints to command
  if (!commanded_joint_ids.empty()) {
    const auto write_result = communication_protocol_->sync_write_position(
        commanded_joint_ids, commanded_positions, commanded_speeds, commanded_accelerations);
    if (!write_result) {
      spdlog::error("FeetechHardwareInterface::write -> {}", write_result.error());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Initial read
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  // If requested, set offsets so that current position becomes zero
  if (auto_zero_on_activate_) {
    for (size_t i = 0; i < joint_offsets_.size(); ++i) {
      joint_offsets_[i] = last_raw_ticks_[i];
    }
    spdlog::info("Auto-zero: joint offsets set to raw ticks: {}",
                 fmt::join(joint_offsets_, ", "));
    // Recompute state with new offsets
    read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  }

  if (apply_home_on_activate_) {
    for (size_t i = 0; i < joint_offsets_.size(); ++i) {
      if (!std::isnan(home_rads_[i])) {
        const int home_ticks = feetech_driver::from_radians(home_rads_[i]);
        joint_offsets_[i] = last_raw_ticks_[i] - home_ticks;
      }
    }
    spdlog::info("Apply-home: joint offsets set for home_rad: {}",
                 fmt::join(joint_offsets_, ", "));
    read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  }

  // Disable torque if position control is disabled (for leader arm)
  if (disable_position_control_) {
    for (auto id : joint_ids_) {
      communication_protocol_->set_torque(id, false);
    }
    spdlog::info("Position control disabled: torque off for all joints (leader mode)");
  }

  // Set the initial command to current joint positions
  hw_positions_ = state_hw_positions_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // all joints torque off
  const auto torque_disable_parameters =
      std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result =
          communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_disable_parameters);
      !result) {
    spdlog::error("FeetechHardwareInterface::on_deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
