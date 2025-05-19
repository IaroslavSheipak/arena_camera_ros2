#pragma once

// ───────────────────────── Includes ─────────────────────────
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ArenaApi.h>

// ───────────────────────── Forward decl. ─────────────────────
class RosImageCallback;

// GenICam run‑time value (bool | int64 | double | string)
using GcValue = std::variant<bool, int64_t, double, std::string>;

// ───────────────────────── ArenaCameraNode ───────────────────
class ArenaCameraNode : public rclcpp::Node
{
public:
  explicit ArenaCameraNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());
  ~ArenaCameraNode();

  // ── tiny logging helpers ───────────────────────────────────
  void log_debug(const std::string &m) { RCLCPP_DEBUG(get_logger(), "%s", m.c_str()); }
  void log_info (const std::string &m) { RCLCPP_INFO (get_logger(), "%s", m.c_str()); }
  void log_warn (const std::string &m) { RCLCPP_WARN (get_logger(), "%s", m.c_str()); }
  void log_err  (const std::string &m) { RCLCPP_ERROR(get_logger(), "%s", m.c_str()); }

private:
  // ───────────────────── SDK handles ────────────────────────
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  // ───────────────────── ROS entities ────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr                           m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr     m_trigger_an_image_srv_;

  // ───────────────────── User parameters ─────────────────────
  // *explicit* ones that still need custom logic
  std::string serial_;
  std::string topic_;                      // final image topic
  std::string pixelformat_ros_;            // ROS encoding string
  std::string pixelformat_pfnc_;           // PFNC enum string
  std::string gain_auto_;
  std::string balance_white_auto_;
  std::string pub_qos_history_;
  std::string pub_qos_reliability_;

  size_t  width_  = 0;
  size_t  height_ = 0;
  size_t  pub_qos_history_depth_ = 0;

  double  gain_                 = -1.0;    // manual gain (dB)
  double  exposure_time_        = -1.0;    // us, < 0 ⇒ auto
  double  acquisition_frame_rate_ = -1.0;  // FPS, optional
  int64_t device_link_throughput_limit_ = -1;  // Ethernet throttle
  int64_t gev_scpd_                    = -1;  // inter‑packet delay

  bool trigger_mode_activated_              = false;

  // ───────────────────── Generic GenICam overrides ───────────
  std::unordered_map<std::string, GcValue> generic_gc_nodes_;

  // ───────────────────── Callback keeper ─────────────────────
  friend class RosImageCallback;
  std::shared_ptr<RosImageCallback> image_callback_holder_;

  // ───────────────────── Initialisation flow ─────────────────
  void parse_parameters_();
  void initialize_();
  void wait_for_device_timer_callback_();

  // ───────────────────── Capture pipeline ────────────────────
  void run_();
  Arena::IDevice *create_device_ros_();
  void msg_form_image_(Arena::IImage *, sensor_msgs::msg::Image &);

  // ───────────────────── Node‑map helpers ────────────────────
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_ethernet_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_balance_white_auto_();
  void set_nodes_trigger_mode_();
  void set_nodes_acquisition_frame_rate_();

  // NEW: apply any "genicam.*" parameter straight to nodemap
  void set_nodes_generic_genicam_();

  // ───────────────────── Trigger service ─────────────────────
  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response>);
};
