#pragma once
/**
 * arena_camera_node_refactored.hpp – public interface for the refactored Arena camera ROS 2 node
 *
 * Matches arena_camera_node_refactored.cpp (same namespace, same members).
 * Kept minimal: only forward‑declarations & member lists needed by external units.
 */

#include <optional>
#include <unordered_map>
#include <memory>
#include <string>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ArenaApi.h"   // Lucid Arena SDK C++ wrapper

namespace arena_camera_node_refactored {

// ── RAII helpers (definitions in cpp) ----------------------------------------------------
struct ArenaSystemDeleter;
struct ArenaDeviceDeleter;
struct ArenaImageDeleter;
using UniqueImage = std::unique_ptr<Arena::IImage, ArenaImageDeleter>;

// ── Main node class ---------------------------------------------------------------------
class ArenaCameraNode : public rclcpp::Node {
public:
  explicit ArenaCameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ArenaCameraNode() override = default;

private:  // ———————————————————————————————— internal types & data
  struct Config {
    std::optional<std::string> serial;
    std::string pixel_fmt{"rgb8"};
    std::optional<int64_t> width, height;
    std::optional<double> gain, exposure, gamma, fps;
    bool trigger{false};
    std::unordered_map<std::string, rclcpp::Parameter> gc_override;
  } cfg_;

  // Arena handles (owning) ---------------------------------------------------------------
  std::shared_ptr<Arena::ISystem> system_;
  std::shared_ptr<Arena::IDevice> device_;

  // ROS entities -------------------------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr    srv_;
  rclcpp::TimerBase::SharedPtr                          discovery_timer_;

  // other state --------------------------------------------------------------------------
  size_t bytes_per_frame_{0};

  // ——— helper methods (implemented in .cpp) ——————————————————————————
  void declare_params_();
  void read_params_();
  void discover_device_();
  void configure_camera_();
  void apply_overrides_();
  void start_stream_();

  void publish_one_image_(const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                          std::shared_ptr<std_srvs::srv::Trigger::Response>);

  void to_ros(Arena::IImage*, sensor_msgs::msg::Image&);

  // image callback wrapper ---------------------------------------------------------------
  class RosCallback : public Arena::IImageCallback {
  public:
    RosCallback(ArenaCameraNode* owner, std::unique_ptr<sensor_msgs::msg::Image> first);
    void OnImage(Arena::IImage* img) override;
  private:
    ArenaCameraNode* owner_;
    std::unique_ptr<sensor_msgs::msg::Image> msg_;
  };
};

} // namespace arena_camera_node_refactored
