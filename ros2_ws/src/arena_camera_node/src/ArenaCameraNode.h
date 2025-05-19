#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ArenaApi.h>

class RosImageCallback;          // forward declaration

class ArenaCameraNode : public rclcpp::Node
{
public:
  ArenaCameraNode();
  ~ArenaCameraNode();

  // simple logging helpers
  void log_debug(const std::string &m) { RCLCPP_DEBUG(get_logger(), "%s", m.c_str()); }
  void log_info (const std::string &m) { RCLCPP_INFO (get_logger(), "%s", m.c_str()); }
  void log_warn (const std::string &m) { RCLCPP_WARN (get_logger(), "%s", m.c_str()); }
  void log_err  (const std::string &m) { RCLCPP_ERROR(get_logger(), "%s", m.c_str()); }

private:
  // ── SDK handles ────────────────────────────────────────────────────────
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  // ── ROS entities ───────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  m_trigger_an_image_srv_;

  // ── user‑configurable parameters (only those needed at run‑time) ───────
  std::string serial_;
  std::string topic_;
  std::string pixelformat_ros_;
  std::string pixelformat_pfnc_;
  std::string gain_auto_;
  std::string balance_white_auto_;
  std::string pub_qos_history_;
  std::string pub_qos_reliability_;

  size_t  width_  = 0;
  size_t  height_ = 0;
  size_t  pub_qos_history_depth_ = 0;

  double  gain_               = -1.0;
  double  exposure_time_      = -1.0;
  double  gamma_              = -1.0;
  double  acquisition_frame_rate_ = -1.0;
  int64_t device_link_throughput_limit_ = -1;
  int64_t gev_scpd_                    = -1;
  double  target_brightness_   = -1.0;

  bool is_passed_serial_                    = false;
  bool is_passed_pixelformat_ros_           = false;
  bool is_passed_width                      = false;
  bool is_passed_height                     = false;
  bool is_passed_gain_                      = false;
  bool is_passed_gain_auto_                 = false;
  bool is_passed_exposure_time_             = false;
  bool is_passed_gamma_                     = false;
  bool is_passed_balance_white_auto_        = false;
  bool trigger_mode_activated_              = false;
  bool is_passed_pub_qos_history_           = false;
  bool is_passed_pub_qos_history_depth_     = false;
  bool is_passed_pub_qos_reliability_       = false;
  bool is_passed_acquisition_frame_rate_    = false;
  bool is_passed_device_link_throughput_limit_ = false;
  bool is_passed_gev_scpd_                  = false;
  bool is_passed_target_brightness_         = false;

  // ── helper to keep callback alive ──────────────────────────────────────
  friend class RosImageCallback;
  std::shared_ptr<RosImageCallback> image_callback_holder_;

  // ── parameter parsing & setup ──────────────────────────────────────────
  void parse_parameters_();
  void initialize_();
  void wait_for_device_timer_callback_();

  // ── capture pipeline ───────────────────────────────────────────────────
  void run_();
  Arena::IDevice* create_device_ros_();
  void msg_form_image_(Arena::IImage*, sensor_msgs::msg::Image &);

  // ── node map helpers ───────────────────────────────────────────────────
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_ethernet_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_gamma_();
  void set_nodes_balance_white_auto_();
  void set_nodes_trigger_mode_();
  void set_nodes_acquisition_frame_rate_();
  void set_nodes_target_brightness_(); 

  // ── trigger service ───────────────────────────────────────────────────
  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response>);
};