/*
 * arena_camera_node_refactored.cpp – concise rewrite of ArenaCameraNode
 * C++17, generic GenICam overrides, sane auto‑function defaults
 *
 * Build (colcon):
 *   add_library(arena_camera_node SHARED arena_camera_node_refactored.cpp)
 *   target_link_libraries(arena_camera_node rclcpp rclcpp_components sensor_msgs Arena)
 *   rclcpp_components_register_node(arena_camera_node_refactored::ArenaCameraNode)
 *
 * Launch YAML example:
 *   arena_camera:
 *     ros__parameters:
 *       pixelformat: rgb8
 *       acquisition_frame_rate: 10.0
 *       trigger_mode: false
 *       genicam_overrides.AutoFunctionsTargetGreyValue: 90
 *       genicam_overrides.AutoGainUpperLimit: 6
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ArenaApi.h"                                   // Lucid Arena SDK
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"     // K_ROS2_PIXELFORMAT_TO_PFNC

#include <optional>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <cstring>
#include <algorithm>

using namespace std::chrono_literals;

namespace arena_camera_node_refactored {

// ── RAII wrappers for Arena -------------------------------------------------------------
struct ArenaSystemDeleter {
  void operator()(Arena::ISystem* p) const noexcept { if (p) Arena::CloseSystem(p); }
};
struct ArenaDeviceDeleter {
  std::shared_ptr<Arena::ISystem> sys;  // must out‑live device
  void operator()(Arena::IDevice* d) const noexcept { if (sys && d) sys->DestroyDevice(d); }
};
struct ArenaImageDeleter {
  std::shared_ptr<Arena::IDevice> dev;
  void operator()(Arena::IImage* img) const noexcept { if (dev && img) dev->RequeueBuffer(img); }
};
using UniqueImage = std::unique_ptr<Arena::IImage, ArenaImageDeleter>;

// ── Main node ---------------------------------------------------------------------------
class ArenaCameraNode : public rclcpp::Node {
public:
  ArenaCameraNode();

private:
  // ------------- parameter bundle -------------------------------------------------------
  struct Config {
    std::optional<std::string> serial;
    std::string pixel_fmt{"rgb8"};
    std::optional<int64_t> width, height;
    std::optional<double> gain, exposure, gamma, fps;
    bool trigger{false};
    std::unordered_map<std::string, rclcpp::Parameter> gc_override; // name -> value
  } cfg_;

  // ------------- Arena handles ----------------------------------------------------------
  std::shared_ptr<Arena::ISystem> system_;
  std::shared_ptr<Arena::IDevice> device_;

  // ------------- ROS entities -----------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr    srv_;
  rclcpp::TimerBase::SharedPtr                          discovery_timer_;

  // ------------- misc -------------------------------------------------------------------
  size_t bytes_per_frame_{0};

  // helpers
  void declare_params_();
  void read_params_();
  void discover_device_();
  void configure_camera_();
  void apply_overrides_();
  void start_stream_();

  void publish_one_image_(const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                          std::shared_ptr<std_srvs::srv::Trigger::Response>);

  // image callback wrapper ---------------------------------------------------------------
  class RosCallback : public Arena::IImageCallback {
  public:
    RosCallback(ArenaCameraNode* owner, std::unique_ptr<sensor_msgs::msg::Image> first)
      : owner_(owner), msg_(std::move(first)) {}
    void OnImage(Arena::IImage* img) override {
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
  private:
    ArenaCameraNode* owner_;
    std::unique_ptr<sensor_msgs::msg::Image> msg_;
  };

  void to_ros(Arena::IImage*, sensor_msgs::msg::Image&);
};

// ── ctor ---------------------------------------------------------------------------------
ArenaCameraNode::ArenaCameraNode()
  : Node("arena_camera",
         rclcpp::NodeOptions()
           .allow_undeclared_parameters(true)  // for overrides
           .automatically_declare_parameters_from_overrides(true))
{
  declare_params_();
  read_params_();

  rclcpp::SensorDataQoS qos;
  pub_ = create_publisher<sensor_msgs::msg::Image>("image", qos);

  srv_ = create_service<std_srvs::srv::Trigger>(
           "trigger_image",
           std::bind(&ArenaCameraNode::publish_one_image_, this, std::placeholders::_1, std::placeholders::_2));

  discovery_timer_ = create_wall_timer(1s, std::bind(&ArenaCameraNode::discover_device_, this));
}

// ── parameter helpers -------------------------------------------------------------------
void ArenaCameraNode::declare_params_() {
  declare_parameter("serial", "");
  declare_parameter("pixelformat", "rgb8");
  declare_parameter("width", 0);
  declare_parameter("height", 0);
  declare_parameter("gain", -1.0);
  declare_parameter("exposure_time", -1.0);
  declare_parameter("gamma", -1.0);
  declare_parameter("acquisition_frame_rate", -1.0);
  declare_parameter("trigger_mode", false);
}

void ArenaCameraNode::read_params_() {
  auto str_opt  = [&](const char* n){ auto s=get_parameter(n).as_string(); return s.empty()?std::optional<std::string>{}:std::make_optional(s); };
  auto int_opt  = [&](const char* n){ auto v=get_parameter(n).as_int();    return v>0?std::optional<int64_t>{v}:std::nullopt; };
  auto dbl_opt  = [&](const char* n){ auto d=get_parameter(n).as_double(); return d>=0?std::optional<double>{d}:std::nullopt; };

  cfg_.serial   = str_opt("serial");
  cfg_.pixel_fmt= get_parameter("pixelformat").as_string();
  cfg_.width    = int_opt("width");
  cfg_.height   = int_opt("height");
  cfg_.gain     = dbl_opt("gain");
  cfg_.exposure = dbl_opt("exposure_time");
  cfg_.gamma    = dbl_opt("gamma");
  cfg_.fps      = dbl_opt("acquisition_frame_rate");
  cfg_.trigger  = get_parameter("trigger_mode").as_bool();

  cfg_.gc_override = get_parameters_by_prefix("genicam_overrides");
}

// ── discovery ---------------------------------------------------------------------------
void ArenaCameraNode::discover_device_() {
  if (!rclcpp::ok()) return;
  if (!system_) system_.reset(Arena::OpenSystem(), ArenaSystemDeleter{});

  system_->UpdateDevices(100);
  auto infos = system_->GetDevices();
  if (infos.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera …");
    return;
  }
  size_t idx = 0;
  if (cfg_.serial) idx = DeviceInfoHelper::get_index_of_serial(infos, *cfg_.serial);
  device_.reset(system_->CreateDevice(infos.at(idx)), ArenaDeviceDeleter{system_});
  discovery_timer_->cancel();
  RCLCPP_INFO(get_logger(), "Connected to %s", DeviceInfoHelper::info(infos.at(idx)).c_str());

  configure_camera_();
  start_stream_();
}

// ── camera configuration ----------------------------------------------------------------
void ArenaCameraNode::configure_camera_() {
  auto nm = device_->GetNodeMap();
  // 1. load user set "Default"
  Arena::SetNodeValue<GenICam::gcstring>(nm, "UserSetSelector", "Default");
  Arena::ExecuteNode(nm, "UserSetLoad");

  // 2. ROI
  if (cfg_.width)  Arena::SetNodeValue<int64_t>(nm, "Width",  *cfg_.width);
  if (cfg_.height) Arena::SetNodeValue<int64_t>(nm, "Height", *cfg_.height);
  const int64_t w = Arena::GetNodeValue<int64_t>(nm, "Width");
  const int64_t h = Arena::GetNodeValue<int64_t>(nm, "Height");
  bytes_per_frame_ = static_cast<size_t>(w*h*3); // assumes rgb8

  // 3. pixel format
  auto pfnc = K_ROS2_PIXELFORMAT_TO_PFNC[cfg_.pixel_fmt];
  if (!pfnc.empty()) Arena::SetNodeValue<GenICam::gcstring>(nm, "PixelFormat", pfnc.c_str());

  // 4. manual params
  if (cfg_.gain)     { Arena::SetNodeValue<GenICam::gcstring>(nm, "GainAuto", "Off"); Arena::SetNodeValue<double>(nm, "Gain", *cfg_.gain);}  
  if (cfg_.exposure) { Arena::SetNodeValue<GenICam::gcstring>(nm, "ExposureAuto", "Off"); Arena::SetNodeValue<double>(nm, "ExposureTime", *cfg_.exposure);} // µs
  if (cfg_.gamma)    { Arena::SetNodeValue<bool>(nm, "GammaEnable", true); Arena::SetNodeValue<double>(nm, "Gamma", *cfg_.gamma);} else { Arena::SetNodeValue<bool>(nm, "GammaEnable", false);}  

  // 5. FPS
  if (cfg_.fps) {
    Arena::SetNodeValue<bool>(nm, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(nm, "AcquisitionFrameRate", *cfg_.fps);
  }

  // 6. default safe auto limits if user didn't supply
  if (!cfg_.gc_override.count("AutoFunctionsTargetGreyValue"))
    RCLCPP_WARN(get_logger(), "AutoFunctionsTargetGreyValue not specified – camera default 128 DN may over‑expose in low light");
  if (!cfg_.gc_override.count("AutoGainUpperLimit"))
    RCLCPP_WARN(get_logger(), "AutoGainUpperLimit not specified – camera may raise gain to noisy levels");
  if (cfg_.fps && !cfg_.gc_override.count("AutoExposureTimeUpperLimit")) {
    RCLCPP_WARN(get_logger(), "AutoExposureTimeUpperLimit missing – exposure can climb to frame period (%.0f µs)", 1e6/ *cfg_.fps);
}
  }
  apply_overrides_();

  // 7. trigger
  Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerMode", cfg_.trigger?"On":"Off");
  if (cfg_.trigger) {
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerSource", "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nm, "TriggerSelector", "FrameStart");
    RCLCPP_WARN(get_logger(), "Trigger mode active – waiting for service call");
  }
}

void ArenaCameraNode::apply_overrides_() {
  if (cfg_.gc_override.empty()) return;
  auto nm = device_->GetNodeMap();
  for (auto &kv : cfg_.gc_override) {
    const std::string &name = kv.first;
    const auto &param = kv.second;
    try {
      switch (param.get_type()) {
        case rclcpp::PARAMETER_INTEGER: Arena::SetNodeValue<int64_t>(nm, name.c_str(), param.as_int()); break;
        case rclcpp::PARAMETER_DOUBLE:  Arena::SetNodeValue<double>(nm,  name.c_str(), param.as_double()); break;
        case rclcpp::PARAMETER_STRING:  Arena::SetNodeValue<GenICam::gcstring>(nm, name.c_str(), param.as_string().c_str()); break;
        default: continue;
      }
      RCLCPP_INFO(get_logger(), "\t%s = %s", name.c_str(), param.value_to_string().c_str());
    } catch (const GenICam::GenericException &e) {
      RCLCPP_WARN(get_logger(), "Failed to set %s (%s)", name.c_str(), e.what());
    }
  }
}

// ── streaming ---------------------------------------------------------------------------
void ArenaCameraNode::to_ros(Arena::IImage* src, sensor_msgs::msg::Image &dst) {
  dst.header.stamp = now();
  dst.header.frame_id = std::to_string(src->GetFrameId());
  dst.height = static_cast<uint32_t>(src->GetHeight());
  dst.width  = static_cast<uint32_t>(src->GetWidth());
  dst.encoding = cfg_.pixel_fmt;
  dst.is_bigendian = src->GetPixelEndianness()==Arena::EPixelEndianness::PixelEndiannessBig;
  const size_t bpp = src->GetBitsPerPixel()/8;
  const size_t row = dst.width*bpp;
  dst.step = static_cast<uint32_t>(row);
  dst.data.resize(bytes_per_frame_);
  const uint8_t* s = static_cast<const uint8_t*>(src->GetData());
  const size_t src_stride = src->GetSizeFilled()/dst.height;
  uint8_t* d = dst.data.data();
  for (uint32_t y=0; y<dst.height; ++y) std::memcpy(d+y*row, s+y*src_stride, row);
}

void ArenaCameraNode::start_stream_() {
  auto first = std::make_unique<sensor_msgs::msg::Image>();
  first->data.resize(bytes_per_frame_);
  auto cb = std::make_shared<RosCallback>(this, std::move(first));
  device_->RegisterImageCallback(cb.get());
  device_->StartStream();
  if (!cfg_.trigger) RCLCPP_INFO(get_logger(), "Streaming started");
}

// ── software‑trigger service -------------------------------------------------------------
void ArenaCameraNode::publish_one_image_(const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
  if (!cfg_.trigger) {
    resp->success = false;
    resp->message = "trigger_mode=false";
    return;
  }
  try {
    Arena::ExecuteNode(device_->GetNodeMap(), "TriggerSoftware");
    UniqueImage img(device_->GetImage(1000), ArenaImageDeleter{device_});
    auto m = std::make_unique<sensor_msgs::msg::Image>();
    to_ros(img.get(), *m);
    pub_->publish(std::move(m));
    resp->success = true;
    resp->message = "frame " + std::to_string(img->GetFrameId());
  } catch (const std::exception &e) {
    resp->success = false;
    resp->message = e.what();
  }
}

} // namespace arena_camera_node_refactored

RCLCPP_COMPONENTS_REGISTER_NODE(arena_camera_node_refactored::ArenaCameraNode)
