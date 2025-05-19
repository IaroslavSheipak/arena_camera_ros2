#include <cstring>    // std::memcpy
#include <stdexcept>  // std::runtime_error
#include <string>

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

#include "ArenaCameraNode.h"

#include <cstring>     // memcpy
#include <sstream>
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"

// helper callback ------------------------------------------------------------
#include <array>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

class RosImageCallback final : public Arena::IImageCallback
{
public:
  explicit RosImageCallback(ArenaCameraNode * owner)
  : owner_(owner)
  {
    const std::size_t bytes = owner_->width_ * owner_->height_ * 3;

    // allocate and initialise the two buffers once
    for (auto & msg : buffers_) {
      msg.height       = owner_->height_;
      msg.width        = owner_->width_;
      msg.encoding     = owner_->pixelformat_ros_;   // e.g. "rgb8"
      msg.is_bigendian = 0;
      msg.step         = static_cast<uint32_t>(owner_->width_ * 3);
      msg.data.resize(bytes);                        // one-time 6 MB allocation
    }
  }

  void OnImage(Arena::IImage * pImage) override
  {
    try {
      auto & msg = buffers_[index_];                 // pick 0 or 1

      owner_->msg_form_image_(pImage, msg);          // copy/convert camera buffer
      owner_->m_pub_->publish(msg);                  // publish by const-ref

      index_ ^= 1;                                   // toggle for next frame
    }
    catch (const std::exception & e) {
      owner_->log_err(std::string("OnImage exception: ") + e.what());
    }
  }

private:
  ArenaCameraNode * owner_;
  std::array<sensor_msgs::msg::Image, 2> buffers_;
  uint8_t index_ {0};
};



// ── ctor / dtor --------------------------------------------------------------
ArenaCameraNode::ArenaCameraNode() : Node("arena_camera")
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  log_info(std::string("Creating \"") + get_name() + "\" node");
  parse_parameters_();
  initialize_();
  log_info(std::string("Created \"") + get_name() + "\" node");
}
ArenaCameraNode::~ArenaCameraNode()
{
  log_info(std::string("Destroying \"") + get_name() + "\" node");
}


void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    // -----------------------------------------------------------------------
    // Serial
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "serial";
    if (this->has_parameter("serial")) {
      int serial_integer;
      this->get_parameter<int>("serial", serial_integer);
      serial_ = std::to_string(serial_integer);
      is_passed_serial_ = true;
    } else {
      serial_ = "";  // Empty => not passed
      is_passed_serial_ = false;
    }

    // -----------------------------------------------------------------------
    // Pixel format
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    // -----------------------------------------------------------------------
    // Width
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    // -----------------------------------------------------------------------
    // Height
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    // -----------------------------------------------------------------------
    // Gain
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    // -----------------------------------------------------------------------
    // Exposure
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    // -----------------------------------------------------------------------
    // NEW: Gamma
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "gamma";
    gamma_ = this->declare_parameter("gamma", -1.0);
    is_passed_gamma_ = (gamma_ >= 0.0);

    // -----------------------------------------------------------------------
    // OPTIONAL: White Balance Auto
    // (Leave default empty => not passed)
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "balance_white_auto";
    balance_white_auto_ = this->declare_parameter("balance_white_auto", "");
    is_passed_balance_white_auto_ = !balance_white_auto_.empty();

    nextParameterToDeclare = "gain_auto";
    gain_auto_            = this->declare_parameter("gain_auto", "");
    is_passed_gain_auto_  = !gain_auto_.empty();

    // -----------------------------------------------------------------------
    // Target Brightness  (Auto-Exposure)
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "target_brightness";
    target_brightness_ = this->declare_parameter("target_brightness", -1.0);
    is_passed_target_brightness_ = (target_brightness_ >= 0.0);
    
    
    nextParameterToDeclare = "exposure_auto_damping";
    exposure_auto_damping_ = declare_parameter<double>(
    "exposure_auto_damping", 0.5);  // pick a sane default
    is_passed_exposure_auto_damping_ = has_parameter("exposure_auto_damping");


    // -----------------------------------------------------------------------
    // Trigger mode
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);

    // -----------------------------------------------------------------------
    // Topic
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
      "topic", std::string("/") + this->get_name() + "/images");

    // -----------------------------------------------------------------------
    // QoS: History policy
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    // -----------------------------------------------------------------------
    // QoS: History depth
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    // -----------------------------------------------------------------------
    // QoS: Reliability
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

    // -----------------------------------------------------------------------
    // Acquisition Frame Rate
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "acquisition_frame_rate";
    acquisition_frame_rate_ = this->declare_parameter("acquisition_frame_rate", -1.0);
    is_passed_acquisition_frame_rate_ = (acquisition_frame_rate_ >= 0.0);

    // -----------------------------------------------------------------------
    // Device Link Throughput Limit (Fancy Ethernet Parameter)
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "device_link_throughput_limit";
    device_link_throughput_limit_ =
      this->declare_parameter<int64_t>("device_link_throughput_limit", -1);
    is_passed_device_link_throughput_limit_ = (device_link_throughput_limit_ >= 0);

    // -----------------------------------------------------------------------
    // GevSCPD (Inter-Packet Delay)
    // -----------------------------------------------------------------------
    nextParameterToDeclare = "gev_scpd";
    gev_scpd_ = this->declare_parameter<int64_t>("gev_scpd", -1);
    is_passed_gev_scpd_ = (gev_scpd_ >= 0);

  } catch (rclcpp::ParameterTypeException & e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::set_nodes_gain_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // 1. Automatic gain has priority over manual gain
  if (is_passed_gain_auto_) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap,
                                           "GainAuto",
                                           gain_auto_.c_str());
    log_info("\tGainAuto set to " + gain_auto_);
    return;                       // nothing else to do
  }

  // 2. Fallback to fixed gain if user gave one
  if (is_passed_gain_) {
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info("\tGain set to " + std::to_string(gain_));
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;

  // -------------------------------------------------------------------------
  // ARENASDK Setup
  // -------------------------------------------------------------------------
  m_pSystem = std::shared_ptr<Arena::ISystem>(
      nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  m_pDevice = std::shared_ptr<Arena::IDevice>(
      nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  // -------------------------------------------------------------------------
  // CHECK DEVICE CONNECTION (Timer)
  // -------------------------------------------------------------------------
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  // -------------------------------------------------------------------------
  // Trigger service
  // -------------------------------------------------------------------------
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  // -------------------------------------------------------------------------
  // Publisher
  // -------------------------------------------------------------------------
  rclcpp::SensorDataQoS pub_qos_;

  // QoS: History
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      throw;
    }
  }

  // QoS: Depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST)
  {
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // QoS: Reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  // Print the final QoS settings
  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info << "\tQoS history     = "
               << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
               << '\n';
  pub_qos_info << "\t\t\t\tQoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\tQoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.reliability]
               << '\n';
  log_info(pub_qos_info.str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
    return;
  }

  // Discover cameras
  m_pSystem->UpdateDevices(100);
  auto device_infos = m_pSystem->GetDevices();

  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  } else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) discovered.");
    run_();
  }
}

// ── run_ --------------------------------------------------------------------
void ArenaCameraNode::run_()
{
  // 1. open device and apply settings
  m_pDevice.reset(create_device_ros_());
  set_nodes_();

  // 3. register the image callback (no extra argument now)
  auto cb = std::make_shared<RosImageCallback>(this);
  m_pDevice->RegisterImageCallback(cb.get());
  image_callback_holder_ = cb;          // keep it alive

  // 4. start streaming
  m_pDevice->StartStream();

  if (trigger_mode_activated_)
    log_warn("\tStream idle – waiting for /trigger_image service");
}


void ArenaCameraNode::msg_form_image_(Arena::IImage * pImage,
                                      sensor_msgs::msg::Image & msg)
{
  // ────────────────────────────────────
  // 1. header – minimal per-frame work
  // ────────────────────────────────────
  msg.header.stamp = rclcpp::Time(pImage->GetTimestampNs(), RCL_ROS_TIME);

  { // int → string with no allocation
    char buf[24];
    auto id = static_cast<std::uint64_t>(pImage->GetFrameId());
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), id);
    if (ec == std::errc{}) {
      msg.header.frame_id.assign(buf, static_cast<std::size_t>(ptr - buf));
    } else {
      msg.header.frame_id = "0";         // fallback, should never happen
    }
  }

  // ────────────────────────────────────
  // 2. pixel copy – one memcpy when strides match
  // ────────────────────────────────────
  const std::size_t bpp        = pImage->GetBitsPerPixel() / 8; // 1 or 3
  const std::size_t dst_stride = msg.width * bpp;
  const std::size_t src_stride =
      static_cast<std::size_t>(pImage->GetSizeFilled()) / msg.height;

  const std::uint8_t * src = static_cast<const std::uint8_t *>(pImage->GetData());
  std::uint8_t       * dst = msg.data.data();

  if (src_stride == dst_stride) {
    // common path: camera delivers tightly packed rows
    std::memcpy(dst, src, dst_stride * msg.height);
  } else {
    // padded rows: fall back to per-row copy
    for (std::uint32_t y = 0; y < msg.height; ++y)
      std::memcpy(dst + y * dst_stride, src + y * src_stride, dst_stride);
  }
}


void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg = "Failed to trigger image because the device is not in trigger mode. "
                      "Please set 'trigger_mode' parameter to true in your launch file.";
    log_warn(msg);
    response->message = msg;
    response->success = false;
    return;
  }

  log_info("A client triggered an image request");
  Arena::IImage* pImage = nullptr;

  try {
    // Wait for trigger to be armed
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      triggerArmed = Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");
      if (!triggerArmed && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }
    } while (!triggerArmed);

    log_debug("Trigger is armed; triggering an image...");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // Acquire image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
    log_debug("Grabbing an image...");
    pImage = m_pDevice->GetImage(1000);

    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);
    pImage = nullptr;
  } catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg = std::string("Exception occurred while triggering an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  } catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg = std::string("GenICam Exception occurred\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    throw std::runtime_error(
        "Camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  // Configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  // NEW: call set_nodes_ethernet_() to handle device link throughput limit, etc.
  set_nodes_ethernet_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_gamma_();
  set_nodes_balance_white_auto_();
  set_nodes_trigger_mode_();
  set_nodes_acquisition_frame_rate_();
  set_nodes_target_brightness_();
  set_nodes_exposure_auto_damping_();

}

// ---------------------------------------------------------------------------
// Load default user set
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

// ---------------------------------------------------------------------------
// ROI, Width, Height
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  log_info(std::string("\tROI set to ") + std::to_string(width_) +
           " X " + std::to_string(height_));
}


// ---------------------------------------------------------------------------
// Pixel Format
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(
          nodemap, "PixelFormat", pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);
    } catch (GenICam::GenericException& e) {
      auto x = std::string("Pixelformat is not supported by this camera: ");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    // If user didn't specify, read from camera
    pixelformat_pfnc_ = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_   = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];
    if (pixelformat_ros_.empty()) {
      log_warn("The current camera pixelformat is not recognized by ROS2. "
               "Please set 'pixelformat' in your launch file to a supported format.");
    }
  }
}

// ---------------------------------------------------------------------------
// Exposure - Now clamped to camera's valid range
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();

    // Turn off auto-exposure
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");

    // Get camera's valid range
    double min_exposure = Arena::GetNodeMin<double>(nodemap, "ExposureTime");
    double max_exposure = Arena::GetNodeMax<double>(nodemap, "ExposureTime");

    double requested_exposure = exposure_time_;

    // Clamp if below min
    if (requested_exposure < min_exposure) {
      log_warn("Requested exposure " + std::to_string(requested_exposure) +
               " us < camera min " + std::to_string(min_exposure) +
               " us. Clamping to min.");
      requested_exposure = min_exposure;
    }

    // Clamp if above max
    if (requested_exposure > max_exposure) {
      log_warn("Requested exposure " + std::to_string(requested_exposure) +
               " us > camera max " + std::to_string(max_exposure) +
               " us. Clamping to max.");
      requested_exposure = max_exposure;
    }

    // Set exposure
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", requested_exposure);
    exposure_time_ = requested_exposure;

    log_info("\tExposureTime set to " + std::to_string(exposure_time_) + " us");
  }
}

// ---------------------------------------------------------------------------
// NEW: set_nodes_gamma_() with clamping
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_gamma_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (is_passed_gamma_) {
    // Camera's valid range for Gamma
    double min_gamma = Arena::GetNodeMin<double>(nodemap, "Gamma");
    double max_gamma = Arena::GetNodeMax<double>(nodemap, "Gamma");

    double requested_gamma = gamma_;

    if (requested_gamma < min_gamma) {
      log_warn("Requested gamma " + std::to_string(requested_gamma) +
               " < camera min " + std::to_string(min_gamma) +
               ", clamping to min.");
      requested_gamma = min_gamma;
    }
    if (requested_gamma > max_gamma) {
      log_warn("Requested gamma " + std::to_string(requested_gamma) +
               " > camera max " + std::to_string(max_gamma) +
               ", clamping to max.");
      requested_gamma = max_gamma;
    }

    // Enable gamma
    Arena::SetNodeValue<bool>(nodemap, "GammaEnable", true);
    Arena::SetNodeValue<double>(nodemap, "Gamma", requested_gamma);

    gamma_ = requested_gamma;
    log_info("\tGamma enabled and set to " + std::to_string(gamma_));
  } else {
    // If user didn't pass gamma >= 0, disable it
    Arena::SetNodeValue<bool>(nodemap, "GammaEnable", false);
    log_info("\tGamma disabled");
  }
}

void ArenaCameraNode::set_nodes_exposure_auto_damping_()
{
  if (!is_passed_exposure_auto_damping_) {
    return;            // user didn't specify the parameter
  }

  auto nodemap = m_pDevice->GetNodeMap();

  try {
    // GenICam float nodes accept a double here
    Arena::SetNodeValue<double>(
        nodemap, "ExposureAutoDamping", exposure_auto_damping_);

    log_info("\tExposureAutoDamping set to " +
             std::to_string(exposure_auto_damping_));
  } catch (GenICam::GenericException &e) {
    log_warn("Failed to set ExposureAutoDamping to " +
             std::to_string(exposure_auto_damping_) +
             ". Exception: " + std::string(e.what()));
  }
}


// ---------------------------------------------------------------------------
// OPTIONAL: White Balance Auto
// If user sets e.g. 'balance_white_auto=Continuous' or 'Off', etc.
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_balance_white_auto_()
{
  if (!is_passed_balance_white_auto_) {
    return; // user didn't specify
  }

  auto nodemap = m_pDevice->GetNodeMap();
  try {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "BalanceWhiteAuto",
                                           balance_white_auto_.c_str());
    log_info("\tBalanceWhiteAuto set to " + balance_white_auto_);
  } catch (GenICam::GenericException& e) {
    log_warn("Failed to set BalanceWhiteAuto to " + balance_white_auto_ +
             ". Exception: " + std::string(e.what()));
  }
}

// ---------------------------------------------------------------------------
// Trigger Mode
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn("\tWarning: trigger mode with negative or unspecified exposure_time can cause long waits.");
    }
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameStart");

    log_warn("\tTrigger mode is activated. You can call the 'trigger_image' service to capture images.");
  } else {
    // Turn TriggerMode off if the user did not enable it
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

// ---------------------------------------------------------------------------
// AcquisitionFrameRate
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_acquisition_frame_rate_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // ── 1.  Write only if the user asked for a specific FPS ────────────────
  if (is_passed_acquisition_frame_rate_) {
    // some cameras require this to be enabled before setting/reading
    Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", acquisition_frame_rate_);

    log_info("\tAcquisitionFrameRate set to "
             + std::to_string(acquisition_frame_rate_));
  }

  // ── 2.  Always read back & log the value the camera is using ───────────
  double current_fps =
      Arena::GetNodeValue<double>(nodemap, "AcquisitionFrameRate");

  log_info("\tAcquisitionFrameRate (effective) = "
           + std::to_string(current_fps));
}


// ---------------------------------------------------------------------------
// NEW: Ethernet Packet Tuning
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_ethernet_()
{
  auto device_nodemap = m_pDevice->GetNodeMap();

  // DeviceLinkThroughputLimit
  if (is_passed_device_link_throughput_limit_) {
    try {
      Arena::SetNodeValue<GenICam::gcstring>(device_nodemap,
        "DeviceLinkThroughputLimitMode", "On");
      // This is typically in bits/second. e.g., 125000000 = 1 Gbps
      Arena::SetNodeValue<int64_t>(
          device_nodemap,
          "DeviceLinkThroughputLimit",
          device_link_throughput_limit_
      );
      log_info("\tDeviceLinkThroughputLimit set to " +
               std::to_string(device_link_throughput_limit_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("Failed to set DeviceLinkThroughputLimit. ") + e.what());
    }
  }

  // GevSCPD (Inter-Packet Delay)
  if (is_passed_gev_scpd_) {
    try {
      // Typically in timestamp ticks, might be 1 tick = 8 ns or 1 us, depending on the camera
      Arena::SetNodeValue<int64_t>(device_nodemap, "GevSCPD", gev_scpd_);
      log_info("\tGevSCPD set to " + std::to_string(gev_scpd_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("Failed to set GevSCPD. ") + e.what());
    }
  }
}


// ---------------------------------------------------------------------------
// NEW: Auto-Exposure Target Brightness (clamped to camera range)
// ---------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_target_brightness_()
{
  if (!is_passed_target_brightness_)
    return;                                // nothing to do

  auto nodemap = m_pDevice->GetNodeMap();

  // Ensure automatic exposure is on; otherwise the camera ignores the target
  try {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous");
  } catch (GenICam::GenericException& e) {
    log_warn("Failed to force ExposureAuto=Continuous: " + std::string(e.what()));
  }

  // Clamp to camera limits
  int64_t min_tb = Arena::GetNodeMin<int64_t>(nodemap, "TargetBrightness");
  int64_t max_tb = Arena::GetNodeMax<int64_t>(nodemap, "TargetBrightness");

  int64_t requested_tb = static_cast<int64_t>(target_brightness_);

  if (requested_tb <  min_tb) { log_warn("TargetBrightness clamped up to min"); requested_tb = min_tb; }
  if (requested_tb >  max_tb) { log_warn("TargetBrightness clamped down to max"); requested_tb = max_tb; }

  Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness", requested_tb);

  target_brightness_ = requested_tb;   // store back actual value
  log_info("\tTargetBrightness set to " + std::to_string(target_brightness_));
}
