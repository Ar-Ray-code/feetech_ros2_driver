// Simple calibration node: disables torque, reads positions, publishes joint_states,
// and saves current ticks as homing_offset to JSON on Trigger service.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_driver/common.hpp>

#include <chrono>
#include <fstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

enum class Phase { WAIT_HOME, TRACK_LIMITS, DONE };

class FeetechCalibrationNode : public rclcpp::Node {
 public:
  FeetechCalibrationNode() : Node("feetech_calibration") {
    usb_port_ = this->declare_parameter<std::string>("usb_port", "/dev/ttyACM0");
    auto ids64 = this->declare_parameter<std::vector<int64_t>>("ids", {1, 2, 3, 4, 5, 6});
    ids_.assign(ids64.begin(), ids64.end());
    rate_hz_ = this->declare_parameter<int>("rate_hz", 20);
    save_path_ = this->declare_parameter<std::string>("save_path", "calibration.json");
    torque_off_ = this->declare_parameter<bool>("torque_off", true);

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

    RCLCPP_INFO(get_logger(), "Calibration started.");
    RCLCPP_INFO(get_logger(), "Phase 1: Move arm to desired HOME pose by hand, then press Enter in this terminal to capture homing_offset.");

    phase_ = Phase::WAIT_HOME;
    input_thread_ = std::thread([this]() { this->stdin_loop(); });

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
  }

  void stdin_loop() {
    // Wait for first Enter
    std::string line;
    std::getline(std::cin, line);
    // Capture homing
    for (size_t i = 0; i < homing_ticks_.size(); ++i) {
      int t = last_ticks_[i] % 4096;
      if (t < 0) t += 4096;
      homing_ticks_[i] = t;
      min_ticks_[i] = last_ticks_[i];
      max_ticks_[i] = last_ticks_[i];
    }
    RCLCPP_INFO(get_logger(), "Captured homing_offset (ticks). Now Phase 2: move joints across allowed range by hand. Press Enter to finish and save JSON.");
    phase_ = Phase::TRACK_LIMITS;

    // Wait for second Enter
    std::getline(std::cin, line);
    phase_ = Phase::DONE;
    save_json();
    rclcpp::shutdown();
  }

  void save_json() {
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
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Save failed: %s", e.what());
    }
  }

  void on_save(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    // When called, save current captured values as well (if any)
    save_json();
    resp->success = true;
    resp->message = std::string("Saved calibration to ") + save_path_;
  }

  std::string usb_port_;
  std::vector<int> ids_;
  int rate_hz_;
  std::string save_path_;
  bool torque_off_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
  std::vector<int> last_ticks_;
  std::vector<int> homing_ticks_;
  std::vector<int> min_ticks_;
  std::vector<int> max_ticks_;

  Phase phase_{};
  std::thread input_thread_;

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
