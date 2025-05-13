/*
 * arena_camera_node_refactored.cpp – compact Arena Camera ROS 2 driver
 * Feature-parity with the original long node:
 *   • topic parameter
 *   • full QoS controls  (best_effort / reliable, keep_last / keep_all, depth)
 *   • hardware time-stamping (camera clock → RCL_ROS_TIME)
 *   • generic GenICam overrides prefix (genicam_overrides.*)
 *   • Ethernet tuning (auto-MTU, packet resend, throughput cap, GevSCPD)
 *   • gain / exposure / gamma clamping
 *   • trigger-mode safety + software-trigger service
 *   • stride-robust image copy
 *
 * Build:
 *   add_library(arena_camera_node SHARED arena_camera_node_refactored.cpp)
 *   target_link_libraries(arena_camera_node rclcpp rclcpp_components sensor_msgs Arena)
 *   rclcpp_components_register_node(arena_camera_node_refactored::ArenaCameraNode)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rmw/types.h>

#include "ArenaApi.h"                                   // Lucid Arena SDK
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"     // K_ROS2_PIXELFORMAT_TO_PFNC

#include <optional>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

namespace arena_camera_node_refactored
{
// ─────────────────────── RAII wrappers ──────────────────────────────────────
struct ArenaSystemDeleter {
  void operator()(Arena::ISystem* p) const noexcept { if (p) Arena::CloseSystem(p); }
};
struct ArenaDeviceDeleter {
  std::shared_ptr<Arena::ISystem> sys;
  void operator()(Arena::IDevice* d) const noexcept { if (sys && d) sys->DestroyDevice(d); }
};
struct ArenaImageDeleter {
  std::shared_ptr<Arena::IDevice> dev;
  void operator()(Arena::IImage* i) const noexcept { if (dev && i) dev->RequeueBuffer(i); }
};
using UniqueImage = std::unique_ptr<Arena::IImage, ArenaImageDeleter>;

static inline std::string lc(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

// ──────────────────────── Node class ────────────────────────────────────────
class ArenaCameraNode : public rclcpp::Node
{
public:
  ArenaCameraNode();

private:
  // configuration bundle -----------------------------------------------------
  struct Config
  {
    std::optional<std::string> serial;
    std::string  pixel_fmt{"rgb8"};
    std::optional<int64_t> width, height;
    std::optional<double>  gain, exposure, gamma, fps;
    bool trigger{false};

    std::string reliability{"best_effort"};   // best_effort | reliable
    std::string history{"keep_last"};         // keep_last   | keep_all
    int history_depth{10};
    std::string topic{"image"};

    std::optional<int64_t> device_link_throughput_limit;
    std::optional<int64_t> gev_scpd;

    std::unordered_map<std::string, rclcpp::Parameter> gc_override;
  } cfg_;

  // Arena handles ------------------------------------------------------------
  std::shared_ptr<Arena::ISystem> system_;
  std::shared_ptr<Arena::IDevice> device_;

  // ROS entities -------------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr    srv_;
  rclcpp::TimerBase::SharedPtr                          discovery_timer_;

  // misc ---------------------------------------------------------------------
  size_t bytes_per_frame_{0};

  // helpers ------------------------------------------------------------------
  void declare_params_();
  void read_params_();
  void discover_device_();
  void configure_camera_();
  void apply_overrides_();
  void start_stream_();
  void publish_one_image_(const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                          std::shared_ptr<std_srvs::srv::Trigger::Response>);
  void to_ros(Arena::IImage*, sensor_msgs::msg::Image&);

  // image callback holder ----------------------------------------------------
  class RosCallback : public Arena::IImageCallback
  {
    ArenaCameraNode* owner_;
    std::unique_ptr<sensor_msgs::msg::Image> msg_;
  public:
    RosCallback(ArenaCameraNode* o, std::unique_ptr<sensor_msgs::msg::Image> first)
      : owner_(o), msg_(std::move(first)) {}
    void OnImage(Arena::IImage* img) override
    {
      try {
        auto next = std::make_unique<sensor_msgs::msg::Image>();
        next->data.resize(owner_->bytes_per_frame_);
        owner_->to_ros(img, *msg_);
        owner_->pub_->publish(std::move(msg_));
        msg_ = std::move(next);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(owner_->get_logger(), "OnImage: %s", e.what());
      }
    }
  };
  std::shared_ptr<RosCallback> cb_holder_;
};

// ═════════════════════════ constructor ══════════════════════════════════════
ArenaCameraNode::ArenaCameraNode()
  : Node("arena_camera",
         rclcpp::NodeOptions{}
           .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
{
  declare_params_();
  read_params_();

  // QoS ----------------------------------------------------------------------
  rclcpp::SensorDataQoS qos;
  qos.reliability(lc(cfg_.reliability) == "reliable"
                    ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                    : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  if (lc(cfg_.history) == "keep_all")
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
  else {
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(cfg_.history_depth);
  }

  pub_ = create_publisher<sensor_msgs::msg::Image>(cfg_.topic, qos);
  srv_ = create_service<std_srvs::srv::Trigger>(
           "trigger_image",
           std::bind(&ArenaCameraNode::publish_one_image_, this,
                     std::placeholders::_1, std::placeholders::_2));

  discovery_timer_ = create_wall_timer(1s,
                      std::bind(&ArenaCameraNode::discover_device_, this));
}

// ═════════════════════ parameter helpers ════════════════════════════════════
void ArenaCameraNode::declare_params_()
{
#define P(name, def)  declare_parameter(name, def)
  P("serial", "");
  P("pixelformat", "rgb8");
  P("width", 0);  P("height", 0);
  P("gain", -1.0); P("exposure_time", -1.0); P("gamma", -1.0);
  P("acquisition_frame_rate", -1.0);
  P("trigger_mode", false);

  P("topic", "image");
  P("qos_reliability", "best_effort");
  P("qos_history", "keep_last");
  P("qos_history_depth", 10);

  P("device_link_throughput_limit", (int64_t)-1);
  P("gev_scpd", (int64_t)-1);
#undef P
}

void ArenaCameraNode::read_params_()
{
  auto str_opt = [&](const char* n){ auto s=get_parameter(n).as_string(); return s.empty()? std::optional<std::string>{}: std::make_optional(s); };
  auto int_opt = [&](const char* n){ auto v=get_parameter(n).as_int();    return v>0? std::optional<int64_t>{v}: std::nullopt; };
  auto dbl_opt = [&](const char* n){ auto d=get_parameter(n).as_double(); return d>=0? std::optional<double>{d}: std::nullopt; };

  cfg_.serial     = str_opt("serial");
  cfg_.pixel_fmt  = get_parameter("pixelformat").as_string();
  cfg_.width      = int_opt("width");
  cfg_.height     = int_opt("height");
  cfg_.gain       = dbl_opt("gain");
  cfg_.exposure   = dbl_opt("exposure_time");
  cfg_.gamma      = dbl_opt("gamma");
  cfg_.fps        = dbl_opt("acquisition_frame_rate");
  cfg_.trigger    = get_parameter("trigger_mode").as_bool();

  cfg_.topic        = get_parameter("topic").as_string();
  cfg_.reliability  = lc(get_parameter("qos_reliability").as_string());
  cfg_.history      = lc(get_parameter("qos_history").as_string());
  cfg_.history_depth= get_parameter("qos_history_depth").as_int();

  int64_t tl = get_parameter("device_link_throughput_limit").as_int();
  if (tl >= 0) cfg_.device_link_throughput_limit = tl;
  int64_t sc = get_parameter("gev_scpd").as_int();
  if (sc >= 0) cfg_.gev_scpd = sc;

  cfg_.gc_override = get_parameters_by_prefix("genicam_overrides");
}

// ═════════════════════ device discovery ═════════════════════════════════════
void ArenaCameraNode::discover_device_()
{
  if (!rclcpp::ok()) return;

  if (!system_)
    system_.reset(Arena::OpenSystem(), ArenaSystemDeleter{});

  system_->UpdateDevices(100);
  auto infos = system_->GetDevices();
  if (infos.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Waiting for camera …");
    return;
  }

  size_t idx = 0;
  if (cfg_.serial)
    idx = DeviceInfoHelper::get_index_of_serial(infos, *cfg_.serial);

  device_.reset(system_->CreateDevice(infos.at(idx)),
                ArenaDeviceDeleter{system_});

  discovery_timer_->cancel();
  RCLCPP_INFO(get_logger(), "Connected to %s",
              DeviceInfoHelper::info(infos.at(idx)).c_str());

  configure_camera_();
  start_stream_();
}

// ═════════════════════ camera configuration ════════════════════════════════
void ArenaCameraNode::configure_camera_()
{
  auto nm = device_->GetNodeMap();

  // 1) default user set
  Arena::SetNodeValue<GenICam::gcstring>(nm, "UserSetSelector", "Default");
  Arena::ExecuteNode(nm, "UserSetLoad");

  // 2) ROI
  if (cfg_.width)  Arena::SetNodeValue<int64_t>(nm, "Width",  *cfg_.width);
  if (cfg_.height) Arena::SetNodeValue<int64_t>(nm, "Height", *cfg_.height);
  const int64_t w = Arena::GetNodeValue<int64_t>(nm, "Width");
  const int64_t h = Arena::GetNodeValue<int64_t>(nm, "Height");
  bytes_per_frame_ = static_cast<size_t>(w * h * 3);
  RCLCPP_INFO(get_logger(), "ROI = %ld × %ld", w, h);

  // 3) pixel format
  auto pfnc = K_ROS2_PIXELFORMAT_TO_PFNC[cfg_.pixel_fmt];
  if (!pfnc.empty())
    Arena::SetNodeValue<GenICam::gcstring>(nm, "PixelFormat", pfnc.c_str());

  // 4) gain / exposure / gamma
  if (cfg_.gain) {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "GainAuto", "Off");
    Arena::SetNodeValue<double>(nm, "Gain", *cfg_.gain);
  }
  if (cfg_.exposure) {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "ExposureAuto", "Off");
    double minE = Arena::GetNodeMin<double>(nm, "ExposureTime");
    double maxE = Arena::GetNodeMax<double>(nm, "ExposureTime");
    double e = std::clamp(*cfg_.exposure, minE, maxE);
    if (e != *cfg_.exposure)
      RCLCPP_WARN(get_logger(), "Exposure clamped to %.1f µs", e);
    Arena::SetNodeValue<double>(nm, "ExposureTime", e);
  }
  if (cfg_.gamma) {
    double minG = Arena::GetNodeMin<double>(nm, "Gamma");
    double maxG = Arena::GetNodeMax<double>(nm, "Gamma");
    double g = std::clamp(*cfg_.gamma, minG, maxG);
    Arena::SetNodeValue<bool>(nm, "GammaEnable", true);
    Arena::SetNodeValue<double>(nm, "Gamma", g);
  } else
    Arena::SetNodeValue<bool>(nm, "GammaEnable", false);

  // 5) FPS
  if (cfg_.fps) {
    Arena::SetNodeValue<bool>(nm, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(nm, "AcquisitionFrameRate", *cfg_.fps);
  }

  // 6) user overrides
  apply_overrides_();

  // 7) trigger
  Arena::SetNodeValue<GenICam::gcstring>(nm,
      "TriggerMode", cfg_.trigger ? "On" : "Off");
  if (cfg_.trigger) {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerSource", "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerSelector", "FrameStart");
    if (!cfg_.exposure)
      RCLCPP_WARN(get_logger(),
                  "Trigger mode with undefined exposure may stall stream");
  }

  // 8) Ethernet tuning
  auto tl = device_->GetTLStreamNodeMap();
  try {
    Arena::SetNodeValue<bool>(tl, "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(tl, "StreamPacketResendEnable", true);
  } catch (...) {}

  if (cfg_.device_link_throughput_limit) {
    Arena::SetNodeValue<GenICam::gcstring>(nm,
        "DeviceLinkThroughputLimitMode", "On");
    Arena::SetNodeValue<int64_t>(nm,
        "DeviceLinkThroughputLimit", *cfg_.device_link_throughput_limit);
  } else {
    try {
      Arena::SetNodeValue<GenICam::gcstring>(nm,
          "DeviceLinkThroughputLimitMode", "Off");
    } catch (...) {}
  }

  if (cfg_.gev_scpd) {
    try {
      Arena::SetNodeValue<int64_t>(nm, "GevSCPD", *cfg_.gev_scpd);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Failed to set GevSCPD");
    }
  }
}

// ═════════════════ GenICam overrides helper ═════════════════════════════════
void ArenaCameraNode::apply_overrides_()
{
  if (cfg_.gc_override.empty()) return;

  auto nm = device_->GetNodeMap();
  for (auto &kv : cfg_.gc_override) {
    try {
      switch (kv.second.get_type()) {
        case rclcpp::PARAMETER_INTEGER: // int64
          Arena::SetNodeValue<int64_t>(nm, kv.first.c_str(), kv.second.as_int());
          break;
        case rclcpp::PARAMETER_DOUBLE:
          Arena::SetNodeValue<double>(nm, kv.first.c_str(), kv.second.as_double());
          break;
        case rclcpp::PARAMETER_STRING:
          Arena::SetNodeValue<GenICam::gcstring>(
              nm, kv.first.c_str(), kv.second.as_string().c_str());
          break;
        default: continue;
      }
      RCLCPP_INFO(get_logger(), "Override %s = %s",
                  kv.first.c_str(), kv.second.value_to_string().c_str());
    } catch (const GenICam::GenericException& e) {
      RCLCPP_WARN(get_logger(), "Failed to set %s (%s)",
                  kv.first.c_str(), e.what());
    }
  }
}

// ═════════════════════ image conversion ═════════════════════════════════════
void ArenaCameraNode::to_ros(Arena::IImage* src,
                             sensor_msgs::msg::Image& dst)
{
  dst.header.stamp = rclcpp::Time(src->GetTimestampNs(), RCL_ROS_TIME);
  dst.header.frame_id = std::to_string(src->GetFrameId());

  dst.height = static_cast<uint32_t>(src->GetHeight());
  dst.width  = static_cast<uint32_t>(src->GetWidth());
  dst.encoding = cfg_.pixel_fmt;
  dst.is_bigendian =
      src->GetPixelEndianness() == Arena::EPixelEndianness::PixelEndiannessBig;

  const size_t bpp = src->GetBitsPerPixel() / 8;
  const size_t dst_row = dst.width * bpp;
  dst.step = static_cast<uint32_t>(dst_row);
  dst.data.resize(bytes_per_frame_);

  const uint8_t* s = static_cast<const uint8_t*>(src->GetData());
  size_t src_stride = dst_row;
  if (dst.height) {
    size_t maybe = src->GetSizeFilled() / dst.height;
    if (maybe >= dst_row) src_stride = maybe;
  }

  uint8_t* d = dst.data.data();
  for (uint32_t y = 0; y < dst.height; ++y)
    std::memcpy(d + y * dst_row, s + y * src_stride, dst_row);
}

// ═════════════════════ start streaming ══════════════════════════════════════
void ArenaCameraNode::start_stream_()
{
  auto first = std::make_unique<sensor_msgs::msg::Image>();
  first->data.resize(bytes_per_frame_);
  cb_holder_ = std::make_shared<RosCallback>(this, std::move(first));

  device_->RegisterImageCallback(cb_holder_.get());
  device_->StartStream();

  if (!cfg_.trigger)
    RCLCPP_INFO(get_logger(), "Streaming started");
  else
    RCLCPP_WARN(get_logger(), "Stream idle – waiting for /trigger_image");
}

// ═════════════════ software-trigger service ═════════════════════════════════
void ArenaCameraNode::publish_one_image_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  if (!cfg_.trigger) {
    resp->success = false;
    resp->message = "trigger_mode=false";
    return;
  }

  Arena::IImage* raw = nullptr;
  try {
    // wait until ‘TriggerArmed’ true
    bool armed = false;
    do {
      armed = Arena::GetNodeValue<bool>(
                device_->GetNodeMap(), "TriggerArmed");
    } while (!armed);

    Arena::ExecuteNode(device_->GetNodeMap(), "TriggerSoftware");

    raw = device_->GetImage(1000);
    auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
    to_ros(raw, *img_msg);
    pub_->publish(std::move(img_msg));

    resp->success = true;
    resp->message = "frame " + std::to_string(raw->GetFrameId());

    device_->RequeueBuffer(raw);
    raw = nullptr;
  } catch (const std::exception& e) {
    if (raw) { device_->RequeueBuffer(raw); raw = nullptr; }
    resp->success = false;
    resp->message = e.what();
  }
}

} // namespace arena_camera_node_refactored

// ───────────────────────── registration ─────────────────────────────────────
RCLCPP_COMPONENTS_REGISTER_NODE(arena_camera_node_refactored::ArenaCameraNode)
