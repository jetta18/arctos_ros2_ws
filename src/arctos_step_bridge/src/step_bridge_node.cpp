#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>

#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <cstdint>

class StepBridgeNode : public rclcpp::Node {
public:
  StepBridgeNode() : rclcpp::Node("step_bridge") {
    // Default axis/joint names and mechanics from firmware
    default_joint_names_ = {"X_joint","Y_joint","Z_joint","A_joint","B_joint","C_joint"};

    declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>(default_joint_names_.begin(), default_joint_names_.end()));
    declare_parameter<std::vector<double>>("steps_per_rev", {200,200,200,200,200,200});
    declare_parameter<std::vector<double>>("microstepping", {16,16,16,16,16,16});
    declare_parameter<std::vector<double>>("gear_ratio", {13.5,150.0,150.0,48.0,27.3375,10.0});

    get_parameter("joint_names", joint_names_);
    get_parameter("steps_per_rev", steps_per_rev_);
    get_parameter("microstepping", microstepping_);
    get_parameter("gear_ratio", gear_ratio_);

    if (joint_names_.size() != AXES) {
      RCLCPP_WARN(get_logger(), "joint_names has size %zu, expected %zu. Using defaults.", joint_names_.size(), static_cast<size_t>(AXES));
      joint_names_ = std::vector<std::string>(default_joint_names_.begin(), default_joint_names_.end());
    }
    resize_or_fill(steps_per_rev_, 200.0);
    resize_or_fill(microstepping_, 16.0);
    resize_or_fill(gear_ratio_, 1.0);

    for (size_t i = 0; i < AXES; ++i) {
      steps_per_rad_[i] = (steps_per_rev_[i] * microstepping_[i] * gear_ratio_[i]) / (2.0 * M_PI);
    }

    // Publishers per axis
    const std::array<std::string, AXES> axis_suffix = {"x","y","z","a","b","c"};
    rclcpp::QoS pub_qos(10);
    pub_qos.reliable();
    for (size_t i = 0; i < AXES; ++i) {
      const std::string topic = std::string("/joint_commands_steps/") + axis_suffix[i];
      pubs_[i] = this->create_publisher<std_msgs::msg::Int32>(topic, pub_qos);
    }

    // Fast subscription to radians topic
    const std::string input_topic = 
      this->declare_parameter<std::string>("input_topic", "/joint_commands_radian");

    // Use reliable to match default publishers
    rclcpp::QoS sub_qos(10);
    sub_qos.reliable();

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic, sub_qos,
      std::bind(&StepBridgeNode::on_joint_state, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "step_bridge running. Subscribing: %s", input_topic.c_str());
  }

private:
  static constexpr size_t AXES = 6;

  void resize_or_fill(std::vector<double>& v, double fill) {
    if (v.size() < AXES) v.resize(AXES, fill);
    if (v.size() > AXES) v.resize(AXES);
  }

  inline int32_t rad_to_steps(size_t axis, double rad) const {
    // llround -> int64_t then clamp to int32 range
    const long long steps_ll = llround(rad * steps_per_rad_[axis]);
    if (steps_ll > INT32_MAX) return INT32_MAX;
    if (steps_ll < INT32_MIN) return INT32_MIN;
    return static_cast<int32_t>(steps_ll);
  }

  void build_name_index_if_needed(const sensor_msgs::msg::JointState& js) {
    if (name_index_built_) return;
    if (js.name.size() != js.position.size() || js.name.empty()) {
      // Fallback: identity mapping
      for (size_t i = 0; i < AXES; ++i) index_of_axis_[i] = i;
      name_index_built_ = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "JointState lacks names; using index order mapping.");
      return;
    }
    std::unordered_map<std::string, size_t> idx;
    idx.reserve(js.name.size());
    for (size_t i = 0; i < js.name.size(); ++i) idx[js.name[i]] = i;

    bool ok = true;
    for (size_t a = 0; a < AXES; ++a) {
      auto it = idx.find(joint_names_[a]);
      if (it == idx.end()) { ok = false; break; }
      index_of_axis_[a] = it->second;
    }
    if (!ok) {
      // fallback to identity mapping
      for (size_t i = 0; i < AXES; ++i) index_of_axis_[i] = i;
      RCLCPP_WARN(get_logger(), "Joint names in message do not match expected; using index order.");
    }
    name_index_built_ = true;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr js_ptr) {
    const auto& js = *js_ptr;
    if (js.position.empty()) return;

    build_name_index_if_needed(js);

    // Convert and publish per-axis RELATIVE step pulses
    for (size_t a = 0; a < AXES; ++a) {
      const size_t idx = index_of_axis_[a];
      int32_t current_abs = 0;
      if (idx < js.position.size()) {
        current_abs = rad_to_steps(a, js.position[idx]);
      }

      long long delta_ll = 0;
      if (have_prev_) {
        delta_ll = static_cast<long long>(current_abs) - static_cast<long long>(last_abs_steps_[a]);
        if (delta_ll > INT32_MAX) delta_ll = INT32_MAX;
        if (delta_ll < INT32_MIN) delta_ll = INT32_MIN;
      } else {
        delta_ll = 0; // first message: do not move
      }

      std_msgs::msg::Int32 out;
      out.data = static_cast<int32_t>(delta_ll);
      pubs_[a]->publish(out);

      // update last absolute steps
      last_abs_steps_[a] = current_abs;
    }
    have_prev_ = true;
  }

  std::array<std::string, AXES> default_joint_names_{};
  std::vector<std::string> joint_names_;
  std::vector<double> steps_per_rev_;
  std::vector<double> microstepping_;
  std::vector<double> gear_ratio_;
  std::array<double, AXES> steps_per_rad_{};

  std::array<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr, AXES> pubs_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

  std::array<size_t, AXES> index_of_axis_{};
  bool name_index_built_ { false };
  std::array<int32_t, AXES> last_abs_steps_{};
  bool have_prev_ { false };
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<StepBridgeNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
