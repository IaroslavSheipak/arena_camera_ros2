#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg)  { RCLCPP_INFO(this->get_logger(),  msg.c_str()); };
  void log_warn(std::string msg)  { RCLCPP_WARN(this->get_logger(),  msg.c_str()); };
  void log_err(std::string msg)   { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  std::string gain_auto_;
  bool is_passed_gain_auto_ = false;

  double exposure_time_; bool is_passed_exposure_time_;

  // For gamma
  double gamma_;
  bool is_passed_gamma_;

  // For optional white balance
  std::string balance_white_auto_;
  bool is_passed_balance_white_auto_;


  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  // Add the missing frame rate variable and its flag:
  double acquisition_frame_rate_;
  bool is_passed_acquisition_frame_rate_;
  int64_t device_link_throughput_limit_;
  bool is_passed_device_link_throughput_limit_;

  int64_t gev_scpd_;
  bool is_passed_gev_scpd_;




  // -------------------------------------------------------------------------
  // Member functions
  // -------------------------------------------------------------------------
  void set_nodes_ethernet_();
  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_gamma_();                    // <--- add
  void set_nodes_balance_white_auto_();        // <--- add if using white balance
  void set_nodes_trigger_mode_();
  void set_nodes_acquisition_frame_rate_(); // Called from set_nodes_()

  // Optional debugging function
  void set_nodes_test_pattern_image_();

  void publish_images_();
  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
};