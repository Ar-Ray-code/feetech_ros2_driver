// Calibration node: disables torque, reads positions, publishes joint_states,
// captures home by gesture, tracks limits, and saves calibration to JSON.

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_driver/common.hpp>

#include <chrono>
#include <deque>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

enum class Phase { WAIT_HOME, TRACK_LIMITS, CONFIRM_SAVE, DONE };

struct ButtonDetector {
  int joint_index{-1};
  int last_ticks{0};
  int segment_start_ticks{0};
  int last_direction{0};
  double last_trigger_time{-1e9};
  bool initialized{false};
  std::deque<double> reversal_times;
  std::deque<int> reversal_amplitudes;
};

class FeetechCalibrationNode : public rclcpp::Node {
 public:
  FeetechCalibrationNode() : Node("feetech_calibration") {
    usb_port_ = this->declare_parameter<std::string>("usb_port", "/dev/ttyACM0");
    auto ids64 = this->declare_parameter<std::vector<int64_t>>("ids", {1, 2, 3, 4, 5, 6});
    ids_.assign(ids64.begin(), ids64.end());
    rate_hz_ = this->declare_parameter<int>("rate_hz", 20);
    save_path_ = this->declare_parameter<std::string>("save_path", "calibration.json");
    torque_off_ = this->declare_parameter<bool>("torque_off", true);
    button_a_id_ = this->declare_parameter<int>("button_a_id", 6);
    button_b_id_ = this->declare_parameter<int>("button_b_id", 1);
    button_min_step_ticks_ = this->declare_parameter<int>("button_min_step_ticks", 4);
    button_min_amplitude_ticks_ = this->declare_parameter<int>("button_min_amplitude_ticks", 80);
    button_required_reversals_ = this->declare_parameter<int>("button_required_reversals", 4);
    button_min_interval_sec_ = this->declare_parameter<double>("button_min_interval_sec", 0.08);
    button_max_interval_sec_ = this->declare_parameter<double>("button_max_interval_sec", 0.70);
    button_window_sec_ = this->declare_parameter<double>("button_window_sec", 2.0);
    button_cooldown_sec_ = this->declare_parameter<double>("button_cooldown_sec", 1.5);
    button_max_interval_ratio_ = this->declare_parameter<double>("button_max_interval_ratio", 1.8);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "save_calibration",
        std::bind(&FeetechCalibrationNode::on_save, this, std::placeholders::_1, std::placeholders::_2));

    // Setup serial + protocol
    serial_ = std::make_unique<feetech_driver::SerialPort>(usb_port_);
    auto res = serial_->configure();
    if (!res) {
      RCLCPP_ERROR(get_logger(), "Serial configure failed: %s", res.error().c_str());
      throw std::runtime_error("serial configure failed");
    }
    protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_));

    // Torque off if requested
    if (torque_off_) {
      for (auto id : ids_) {
        auto r = protocol_->set_torque(static_cast<uint8_t>(id), false);
        if (!r) {
          RCLCPP_WARN(get_logger(), "set_torque off failed for id %d: %s", id, r.error().c_str());
        }
      }
    }

    // Prepare joint message template
    joint_names_.clear();
    for (auto id : ids_) joint_names_.push_back(std::to_string(id));

    last_ticks_.assign(ids_.size(), 0);
    homing_ticks_.assign(ids_.size(), 0);
    min_ticks_.assign(ids_.size(), std::numeric_limits<int>::max());
    max_ticks_.assign(ids_.size(), std::numeric_limits<int>::min());
    unwrapped_ticks_.assign(ids_.size(), 0);
    have_tick_sample_.assign(ids_.size(), false);

    a_button_.joint_index = find_joint_index(button_a_id_);
    b_button_.joint_index = find_joint_index(button_b_id_);

    RCLCPP_INFO(get_logger(), "Calibration started.");
    RCLCPP_INFO(get_logger(), "Phase 1: Move arm to desired HOME pose by hand, then wiggle joint id=%d to press A and capture homing_offset.", button_a_id_);
    RCLCPP_INFO(get_logger(), "Phase 2: Sweep joints across their allowed range. Wiggle joint id=%d for A to open save confirmation.", button_a_id_);
    RCLCPP_INFO(get_logger(), "In save confirmation, wiggle joint id=%d for A to save, or joint id=%d for B to cancel and continue calibration.", button_a_id_, button_b_id_);

    phase_ = Phase::WAIT_HOME;

    // Start timer loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / std::max(1, rate_hz_)),
                                     std::bind(&FeetechCalibrationNode::on_timer, this));
  }

 private:
  void on_timer() {
    // Sync read present position + speed (4 bytes per id)
    std::vector<uint8_t> ids_u8;
    ids_u8.reserve(ids_.size());
    for (auto id : ids_) ids_u8.push_back(static_cast<uint8_t>(id));

    std::vector<std::array<uint8_t, 4>> data;
    auto rr = protocol_->sync_read(ids_u8, SMS_STS_PRESENT_POSITION_L, &data);
    if (!rr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "sync_read failed: %s", rr.error().c_str());
      return;
    }

    last_ticks_.resize(ids_.size());
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.resize(ids_.size());
    js.velocity.resize(ids_.size());

    for (size_t i = 0; i < ids_.size(); ++i) {
      int ticks = feetech_driver::from_sts(feetech_driver::WordBytes{.low = data[i][0], .high = data[i][1]});
      int vel = feetech_driver::from_sts(feetech_driver::WordBytes{.low = data[i][2], .high = data[i][3]});
      last_ticks_[i] = ticks;
      if (!have_tick_sample_[i]) {
        unwrapped_ticks_[i] = ticks;
        have_tick_sample_[i] = true;
      } else {
        unwrapped_ticks_[i] += shortest_tick_delta(unwrapped_ticks_[i], ticks);
      }
      js.position[i] = feetech_driver::to_radians(ticks);  // raw ticks → rad (未オフセット)
      js.velocity[i] = feetech_driver::to_radians(vel);
    }
    publisher_->publish(js);

    // Phase handling
    if (phase_ == Phase::TRACK_LIMITS) {
      for (size_t i = 0; i < last_ticks_.size(); ++i) {
        min_ticks_[i] = std::min(min_ticks_[i], last_ticks_[i]);
        max_ticks_[i] = std::max(max_ticks_[i], last_ticks_[i]);
      }
    }

    const double now_sec = now().seconds();
    const bool a_pressed = detect_button_press(a_button_, now_sec);
    const bool b_pressed = detect_button_press(b_button_, now_sec);

    if (a_pressed && b_pressed) {
      RCLCPP_WARN(get_logger(), "Detected A and B simultaneously. Ignoring ambiguous gesture.");
      return;
    }

    if (phase_ == Phase::WAIT_HOME && a_pressed) {
      capture_home_pose();
      return;
    }

    if (phase_ == Phase::TRACK_LIMITS && a_pressed) {
      phase_ = Phase::CONFIRM_SAVE;
      RCLCPP_INFO(get_logger(), "Save confirmation opened.");
      RCLCPP_INFO(get_logger(), "Wiggle joint id=%d again for A to save and exit, or joint id=%d for B to cancel and continue calibration.", button_a_id_, button_b_id_);
      return;
    }

    if (phase_ == Phase::CONFIRM_SAVE) {
      if (a_pressed) {
        if (save_json()) {
          phase_ = Phase::DONE;
          rclcpp::shutdown();
        }
        return;
      }
      if (b_pressed) {
        phase_ = Phase::TRACK_LIMITS;
        RCLCPP_INFO(get_logger(), "Save canceled. Continuing range tracking.");
        return;
      }
    }
  }

  bool save_json() {
    if (!homing_captured_) {
      RCLCPP_ERROR(get_logger(), "Cannot save calibration before homing_offset is captured.");
      return false;
    }
    try {
      std::ofstream ofs(save_path_);
      ofs << "{\n";
      for (size_t i = 0; i < ids_.size(); ++i) {
        int id = ids_[i];
        int hoff = homing_ticks_[i] % 4096;
        if (hoff < 0) hoff += 4096;
        int rmin = min_ticks_[i];
        int rmax = max_ticks_[i];
        std::string name = joint_names_[i];
        ofs << "  \"joint_" << name << "\": {\n";
        ofs << "    \"id\": " << id << ",\n";
        ofs << "    \"drive_mode\": 0,\n";
        ofs << "    \"homing_offset\": " << hoff << ",\n";
        ofs << "    \"range_min\": " << rmin << ",\n";
        ofs << "    \"range_max\": " << rmax << "\n";
        ofs << "  }" << (i + 1 < ids_.size() ? ",\n" : "\n");
      }
      ofs << "}\n";
      ofs.close();
      RCLCPP_INFO(get_logger(), "Saved calibration to %s", save_path_.c_str());
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Save failed: %s", e.what());
      return false;
    }
  }

  void on_save(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    // When called, save current captured values as well (if any)
    resp->success = save_json();
    resp->message = resp->success ? std::string("Saved calibration to ") + save_path_
                                  : std::string("Calibration save failed");
  }

  int find_joint_index(const int joint_id) const {
    const auto it = std::find(ids_.begin(), ids_.end(), joint_id);
    if (it == ids_.end()) {
      throw std::runtime_error("button joint id not found in ids parameter");
    }
    return static_cast<int>(std::distance(ids_.begin(), it));
  }

  static int shortest_tick_delta(const int previous_unwrapped_ticks, const int current_raw_ticks) {
    int previous_raw_ticks = previous_unwrapped_ticks % 4096;
    if (previous_raw_ticks < 0) previous_raw_ticks += 4096;
    int delta = current_raw_ticks - previous_raw_ticks;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
  }

  bool detect_button_press(ButtonDetector &detector, const double now_sec) {
    if (detector.joint_index < 0) {
      return false;
    }

    const auto joint_index = static_cast<size_t>(detector.joint_index);
    if (!have_tick_sample_[joint_index]) {
      return false;
    }

    const int ticks = unwrapped_ticks_[joint_index];
    if (!detector.initialized) {
      detector.initialized = true;
      detector.last_ticks = ticks;
      detector.segment_start_ticks = ticks;
      return false;
    }

    const int delta = ticks - detector.last_ticks;
    detector.last_ticks = ticks;
    if (std::abs(delta) < button_min_step_ticks_) {
      return false;
    }

    const int direction = delta > 0 ? 1 : -1;
    if (detector.last_direction == 0) {
      detector.last_direction = direction;
      detector.segment_start_ticks = ticks;
      return false;
    }

    if (direction == detector.last_direction) {
      return false;
    }

    const int amplitude = std::abs(ticks - detector.segment_start_ticks);
    detector.segment_start_ticks = ticks;
    detector.last_direction = direction;

    if (amplitude < button_min_amplitude_ticks_) {
      return false;
    }

    detector.reversal_times.push_back(now_sec);
    detector.reversal_amplitudes.push_back(amplitude);
    while (!detector.reversal_times.empty() && now_sec - detector.reversal_times.front() > button_window_sec_) {
      detector.reversal_times.pop_front();
      detector.reversal_amplitudes.pop_front();
    }

    if (now_sec - detector.last_trigger_time < button_cooldown_sec_) {
      return false;
    }

    if (detector.reversal_times.size() < static_cast<size_t>(button_required_reversals_)) {
      return false;
    }

    const size_t begin = detector.reversal_times.size() - static_cast<size_t>(button_required_reversals_);
    double min_interval = std::numeric_limits<double>::max();
    double max_interval = 0.0;
    for (size_t i = begin + 1; i < detector.reversal_times.size(); ++i) {
      const double interval = detector.reversal_times[i] - detector.reversal_times[i - 1];
      min_interval = std::min(min_interval, interval);
      max_interval = std::max(max_interval, interval);
      if (interval < button_min_interval_sec_ || interval > button_max_interval_sec_) {
        return false;
      }
    }

    if (min_interval <= 0.0 || max_interval / min_interval > button_max_interval_ratio_) {
      return false;
    }

    detector.last_trigger_time = now_sec;
    detector.reversal_times.clear();
    detector.reversal_amplitudes.clear();
    RCLCPP_INFO(get_logger(), "Detected button gesture on joint id=%d.", ids_[joint_index]);
    return true;
  }

  void capture_home_pose() {
    for (size_t i = 0; i < homing_ticks_.size(); ++i) {
      int t = last_ticks_[i] % 4096;
      if (t < 0) t += 4096;
      homing_ticks_[i] = t;
      min_ticks_[i] = last_ticks_[i];
      max_ticks_[i] = last_ticks_[i];
    }
    homing_captured_ = true;
    phase_ = Phase::TRACK_LIMITS;
    RCLCPP_INFO(get_logger(), "Captured homing_offset (ticks).");
    RCLCPP_INFO(get_logger(), "Now sweep joints across allowed range. Wiggle joint id=%d for A when ready to save.", button_a_id_);
  }

  std::string usb_port_;
  std::vector<int> ids_;
  int rate_hz_;
  std::string save_path_;
  bool torque_off_;
  int button_a_id_;
  int button_b_id_;
  int button_min_step_ticks_;
  int button_min_amplitude_ticks_;
  int button_required_reversals_;
  double button_min_interval_sec_;
  double button_max_interval_sec_;
  double button_window_sec_;
  double button_cooldown_sec_;
  double button_max_interval_ratio_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
  std::vector<int> last_ticks_;
  std::vector<int> homing_ticks_;
  std::vector<int> min_ticks_;
  std::vector<int> max_ticks_;
  std::vector<int> unwrapped_ticks_;
  std::vector<bool> have_tick_sample_;

  Phase phase_{};
  bool homing_captured_{false};
  ButtonDetector a_button_;
  ButtonDetector b_button_;

  std::unique_ptr<feetech_driver::SerialPort> serial_;
  std::unique_ptr<feetech_driver::CommunicationProtocol> protocol_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeetechCalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
