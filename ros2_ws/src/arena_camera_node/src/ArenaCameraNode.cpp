// ───────────────────────── ArenaCameraNode.cpp ──────────────────────────
// Streamlined implementation that keeps all original functionality while
// letting users write any plain GenICam node through `genicam.*` parameters.
//
// Requires C++17 (std::variant) and Arena SDK ≥ 1.2
// -------------------------------------------------------------------------

#include "ArenaCameraNode.h"

#include <cstring>           // std::memcpy
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

// ROS helpers
#include "rmw/types.h"
#include "rclcpp_adapter/pixelformat_translation.h"          // K_* maps
#include "rclcpp_adapter/quilty_of_service_translation.cpp"  // QoS maps

// Local helpers
#include "light_arena/deviceinfo_helper.h"

// ───────────────────────── Image callback helper ─────────────────────────
class RosImageCallback : public Arena::IImageCallback
{
public:
  RosImageCallback(ArenaCameraNode *owner,
                   std::unique_ptr<sensor_msgs::msg::Image> first)
      : owner_(owner), msg_(std::move(first)) {}

  void OnImage(Arena::IImage *pImage) override
  {
    try {
      // next buffer
      auto next = std::make_unique<sensor_msgs::msg::Image>();
      next->data.resize(owner_->width_ * owner_->height_ * 3);

      // convert & publish current image
      owner_->msg_form_image_(pImage, *msg_);
      owner_->m_pub_->publish(std::move(msg_));

      // keep next container (Arena re-queues buffer automatically)
      msg_ = std::move(next);
    } catch (const std::exception &e) {
      owner_->log_err(std::string("OnImage exception: ") + e.what());
    }
  }

private:
  ArenaCameraNode *owner_;
  std::unique_ptr<sensor_msgs::msg::Image> msg_;
};

// ───────────────────────── ctor / dtor ───────────────────────────────────
ArenaCameraNode::ArenaCameraNode(const rclcpp::NodeOptions &opts)
    : Node("arena_camera", opts)
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

// ───────────────────────── Parameter parsing ─────────────────────────────
void ArenaCameraNode::parse_parameters_()
{
  std::string current;
  try {
    // ── explicit parameters ────────────────────────────────────────────
    current = "serial";
    if (this->has_parameter("serial")) {
      int serial_int{};
      this->get_parameter("serial", serial_int);
      serial_ = std::to_string(serial_int);
    }

    current = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");

    current = "width";
    width_  = this->declare_parameter("width", 0);
    current = "height";
    height_ = this->declare_parameter("height", 0);

    current = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    current = "gain_auto";
    gain_auto_ = this->declare_parameter("gain_auto", "");

    current = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);

    current = "balance_white_auto";
    balance_white_auto_ =
        this->declare_parameter("balance_white_auto", "");

    current = "trigger_mode";
    trigger_mode_activated_ =
        this->declare_parameter("trigger_mode", false);

    current = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");

    // ── QoS ─────────────────────────────────────────────────────────────
    current = "qos_history";
    pub_qos_history_       = this->declare_parameter("qos_history", "");
    current = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    current = "qos_reliability";
    pub_qos_reliability_   = this->declare_parameter("qos_reliability", "");

    // ── performance tuning ─────────────────────────────────────────────
    current = "acquisition_frame_rate";
    acquisition_frame_rate_ =
        this->declare_parameter("acquisition_frame_rate", -1.0);

    current = "device_link_throughput_limit";
    device_link_throughput_limit_ =
        this->declare_parameter<int64_t>("device_link_throughput_limit", -1);

    current = "gev_scpd";
    gev_scpd_ =
        this->declare_parameter<int64_t>("gev_scpd", -1);

    // ── generic GenICam overrides (all parameters that start with “genicam.”)
    const auto &over =
        this->get_node_parameters_interface()->get_parameter_overrides();

    constexpr std::string_view prefix = "genicam.";
    for (const auto &kv : over) {
      if (kv.first.rfind(prefix, 0) != 0)           // not our prefix
        continue;

      std::string node = kv.first.substr(prefix.size());  // strip prefix
      rclcpp::Parameter param(node, kv.second);           // wrap value

      switch (param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          generic_gc_nodes_[node] = param.as_bool(); break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          generic_gc_nodes_[node] = static_cast<int64_t>(param.as_int()); break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          generic_gc_nodes_[node] = param.as_double(); break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          generic_gc_nodes_[node] = param.as_string(); break;
        default:
          log_warn("genicam." + node + " ignored: unsupported type");
      }
    }

  } catch (const rclcpp::ParameterTypeException &) {
    log_err("Parameter error at " + current);
    throw;
  }
}



// ───────────────────────── Gain helper ───────────────────────────────────
void ArenaCameraNode::set_nodes_gain_()
{
  auto nm = m_pDevice->GetNodeMap();
  if (!gain_auto_.empty()) {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "GainAuto", gain_auto_.c_str());
    log_info("\tGainAuto set to " + gain_auto_);
  } else if (gain_ >= 0.0) {
    Arena::SetNodeValue<double>(nm, "Gain", gain_);
    log_info("\tGain set to " + std::to_string(gain_));
  }
}

// ───────────────────────── Initialisation ────────────────────────────────
void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;

  m_pDevice = std::shared_ptr<Arena::IDevice>(nullptr,
        [this](Arena::IDevice *d) {
          if (m_pSystem && d) m_pSystem->DestroyDevice(d);
        });

  // poll timer -------------------------------------------------------------
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  // trigger service --------------------------------------------------------
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      get_name() + std::string("/trigger_image"),
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  // publisher QoS ----------------------------------------------------------
  rclcpp::SensorDataQoS qos;
  if (!pub_qos_history_.empty()) {
    if (!is_supported_qos_histroy_policy(pub_qos_history_))
      throw std::invalid_argument("invalid qos_history");
    qos.history(K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.at(pub_qos_history_));
  }
  if (pub_qos_history_depth_ > 0 &&
      qos.get_rmw_qos_profile().history == RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    qos.keep_last(pub_qos_history_depth_);
  if (!pub_qos_reliability_.empty()) {
    if (!is_supported_qos_reliability_policy(pub_qos_reliability_))
      throw std::invalid_argument("invalid qos_reliability");
    qos.reliability(
        K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY.at(pub_qos_reliability_));
  }

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, qos);
}

// ───────────────────────── Device polling ────────────────────────────────
void ArenaCameraNode::wait_for_device_timer_callback_()
{
  if (!rclcpp::ok()) { rclcpp::shutdown(); return; }

  m_pSystem->UpdateDevices(100);
  auto devs = m_pSystem->GetDevices();
  if (devs.empty()) {
    log_info("No arena camera connected. Waiting…");
    return;
  }
  m_wait_for_device_timer_callback_->cancel();
  log_info(std::to_string(devs.size()) + " arena device(s) discovered.");
  run_();
}

// ───────────────────────── run_: start stream ────────────────────────────
void ArenaCameraNode::run_()
{
  m_pDevice.reset(create_device_ros_());
  set_nodes_();

  auto first = std::make_unique<sensor_msgs::msg::Image>();
  first->data.resize(width_ * height_ * 3);

  auto cb = std::make_shared<RosImageCallback>(this, std::move(first));
  m_pDevice->RegisterImageCallback(cb.get());
  image_callback_holder_ = cb;

  m_pDevice->StartStream();
  if (trigger_mode_activated_)
    log_warn("\tStream idle – waiting for /trigger_image service");
}

// ───────────────────────── Image → ROS message ───────────────────────────
void ArenaCameraNode::msg_form_image_(Arena::IImage *pImage,
                                      sensor_msgs::msg::Image &msg)
{
  msg.header.stamp    = rclcpp::Time(pImage->GetTimestampNs(), RCL_ROS_TIME);
  msg.header.frame_id = std::to_string(pImage->GetFrameId());

  msg.height = static_cast<uint32_t>(pImage->GetHeight());
  msg.width  = static_cast<uint32_t>(pImage->GetWidth());

  msg.encoding     = pixelformat_ros_;
  msg.is_bigendian = (pImage->GetPixelEndianness() ==
                      Arena::EPixelEndianness::PixelEndiannessBig);

  const size_t bpp        = pImage->GetBitsPerPixel() / 8;
  const size_t dst_stride = msg.width * bpp;
  size_t       src_stride = dst_stride;
  const size_t filled     = pImage->GetSizeFilled();

  if (msg.height && filled >= dst_stride * msg.height) {
    const size_t maybe = filled / msg.height;
    if (maybe >= dst_stride) src_stride = maybe;
  }
  msg.step = static_cast<uint32_t>(dst_stride);

  const uint8_t *src = static_cast<const uint8_t *>(pImage->GetData());
  uint8_t       *dst = msg.data.data();
  for (uint32_t y = 0; y < msg.height; ++y)
    std::memcpy(dst + y * dst_stride, src + y * src_stride, dst_stride);
}

// ───────────────────────── Trigger service ───────────────────────────────
void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    const std::string err =
        "Device is not in trigger mode. Set 'trigger_mode=true'";
    log_warn(err);
    response->message = err;
    response->success = false;
    return;
  }

  log_info("A client triggered an image request");
  Arena::IImage *pImage = nullptr;

  try {
    while (!Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed"))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
    pImage       = m_pDevice->GetImage(1000);

    msg_form_image_(pImage, *img_msg);
    m_pub_->publish(std::move(img_msg));

    response->message = "image " + std::to_string(pImage->GetFrameId()) +
                        " published to " + topic_;
    response->success = true;
    log_info(response->message);

    m_pDevice->RequeueBuffer(pImage); pImage = nullptr;
  } catch (const std::exception &e) {
    if (pImage) m_pDevice->RequeueBuffer(pImage);
    const std::string err = "Trigger exception: " + std::string(e.what());
    log_warn(err);
    response->message = err;
    response->success = false;
  }
}

// ───────────────────────── Device create helper ─────────────────────────
Arena::IDevice *ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);
  auto infos = m_pSystem->GetDevices();
  if (infos.empty())
    throw std::runtime_error("Camera(s) disconnected");

  size_t idx = 0;
  if (!serial_.empty())
    idx = DeviceInfoHelper::get_index_of_serial(infos, serial_);

  auto *dev = m_pSystem->CreateDevice(infos.at(idx));
  log_info("device created " + DeviceInfoHelper::info(infos.at(idx)));
  return dev;
}

// ───────────────────────── Node map helpers ──────────────────────────────
void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();

  // Enable jumbo-frame tuning helpers on the stream layer
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(),
                            "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(),
                            "StreamPacketResendEnable", true);

  set_nodes_ethernet_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_balance_white_auto_();
  set_nodes_trigger_mode_();
  set_nodes_acquisition_frame_rate_();
  set_nodes_generic_genicam_();   // last: user overrides always win
}

// -------------------------------------------------------------------------
// Load default user set
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nm = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nm, "UserSetSelector", "Default");
  Arena::ExecuteNode(nm, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

// -------------------------------------------------------------------------
// Ethernet tuning (throughput + inter-packet delay)
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_ethernet_()
{
  auto nm = m_pDevice->GetNodeMap();
  if (device_link_throughput_limit_ >= 0) {
    try {
      Arena::SetNodeValue<GenICam::gcstring>(nm,
          "DeviceLinkThroughputLimitMode", "On");
      Arena::SetNodeValue<int64_t>(nm,
          "DeviceLinkThroughputLimit", device_link_throughput_limit_);
      log_info("\tDeviceLinkThroughputLimit set to "
               + std::to_string(device_link_throughput_limit_));
    } catch (GenICam::GenericException &e) {
      log_warn("Failed to set DeviceLinkThroughputLimit: " + std::string(e.what()));
    }
  }

  if (gev_scpd_ >= 0) {
    try {
      Arena::SetNodeValue<int64_t>(nm, "GevSCPD", gev_scpd_);
      log_info("\tGevSCPD set to " + std::to_string(gev_scpd_));
    } catch (GenICam::GenericException &e) {
      log_warn("Failed to set GevSCPD: " + std::string(e.what()));
    }
  }
}

// -------------------------------------------------------------------------
// ROI
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_roi_()
{
  auto nm = m_pDevice->GetNodeMap();

  if (width_  > 0) Arena::SetNodeValue<int64_t>(nm, "Width",  width_);
  if (height_ > 0) Arena::SetNodeValue<int64_t>(nm, "Height", height_);

  // read-back for buffer allocations
  width_  = Arena::GetNodeValue<int64_t>(nm, "Width");
  height_ = Arena::GetNodeValue<int64_t>(nm, "Height");
  log_info("\tROI set to " + std::to_string(width_) + " × " + std::to_string(height_));
}

// -------------------------------------------------------------------------
// Pixel format (ROS ↔ PFNC translation)
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nm = m_pDevice->GetNodeMap();

  if (!pixelformat_ros_.empty()) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty())
      throw std::invalid_argument("pixelformat is not supported");

    Arena::SetNodeValue<GenICam::gcstring>(nm,
                                           "PixelFormat",
                                           pixelformat_pfnc_.c_str());
    log_info("\tPixelFormat set to " + pixelformat_pfnc_);
  } else {
    // read from camera → convert to ROS string for msgs
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nm, "PixelFormat");
    pixelformat_ros_ =
        K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];
    if (pixelformat_ros_.empty())
      log_warn("Camera pixel format not recognised by ROS2; set 'pixelformat' parameter");
  }
}

// -------------------------------------------------------------------------
// Exposure (manual with clamping, or leave auto)
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_exposure_()
{
  if (exposure_time_ < 0) return;              // leave auto-exposure as is

  auto nm = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nm, "ExposureAuto", "Off");

  const double min_exp = Arena::GetNodeMin<double>(nm, "ExposureTime");
  const double max_exp = Arena::GetNodeMax<double>(nm, "ExposureTime");

  if (exposure_time_ < min_exp) {
    log_warn("Exposure " + std::to_string(exposure_time_)
             + " < min; clamped up");
    exposure_time_ = min_exp;
  } else if (exposure_time_ > max_exp) {
    log_warn("Exposure " + std::to_string(exposure_time_)
             + " > max; clamped down");
    exposure_time_ = max_exp;
  }

  Arena::SetNodeValue<double>(nm, "ExposureTime", exposure_time_);
  log_info("\tExposureTime set to " + std::to_string(exposure_time_) + " µs");
}

// -------------------------------------------------------------------------
// White-balance auto
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_balance_white_auto_()
{
  if (balance_white_auto_.empty()) return;

  auto nm = m_pDevice->GetNodeMap();
  try {
    Arena::SetNodeValue<GenICam::gcstring>(nm,
                                           "BalanceWhiteAuto",
                                           balance_white_auto_.c_str());
    log_info("\tBalanceWhiteAuto set to " + balance_white_auto_);
  } catch (GenICam::GenericException &e) {
    log_warn("Failed to set BalanceWhiteAuto: " + std::string(e.what()));
  }
}

// -------------------------------------------------------------------------
// Trigger mode
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nm = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerMode",  "On");
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerSource","Software");
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerSelector","FrameStart");
    log_warn("\tTrigger mode activated – use /trigger_image service");
  } else {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerMode", "Off");
  }
}

// -------------------------------------------------------------------------
// Acquisition FPS
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_acquisition_frame_rate_()
{
  auto nm = m_pDevice->GetNodeMap();
  if (acquisition_frame_rate_ >= 0.0) {
    Arena::SetNodeValue<bool>(nm, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(nm,
                                "AcquisitionFrameRate",
                                acquisition_frame_rate_);
    log_info("\tAcquisitionFrameRate requested "
             + std::to_string(acquisition_frame_rate_));
  }

  double fps = Arena::GetNodeValue<double>(nm, "AcquisitionFrameRate");
  log_info("\tAcquisitionFrameRate (effective) = " + std::to_string(fps));
}

// -------------------------------------------------------------------------
// Generic GenICam overrides (genicam.* parameters)
// -------------------------------------------------------------------------
void ArenaCameraNode::set_nodes_generic_genicam_()
{
  auto nm = m_pDevice->GetNodeMap();

  for (const auto &[name, value] : generic_gc_nodes_) {
    try {
      std::visit([&](auto &&v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, bool>)
          Arena::SetNodeValue<bool>(nm, name.c_str(), v);
        else if constexpr (std::is_same_v<T, int64_t>)
          Arena::SetNodeValue<int64_t>(nm, name.c_str(), v);
        else if constexpr (std::is_same_v<T, double>)
          Arena::SetNodeValue<double>(nm, name.c_str(), v);
        else if constexpr (std::is_same_v<T, std::string>)
          Arena::SetNodeValue<GenICam::gcstring>(nm, name.c_str(), v.c_str());
      }, value);

      log_info("\t" + name + " <- genicam override");
    } catch (GenICam::GenericException &e) {
      log_warn("Failed to set " + name + ": " + std::string(e.what()));
    }
  }
}
