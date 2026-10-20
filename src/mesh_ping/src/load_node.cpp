// Background load for mesh_ping tests: publishes and/or subscribes to `load_<i>`
// topics, adding DDS endpoints (discovery load) and traffic (bandwidth load).
// Run several processes with tools/spawn_load.sh to add DDS participants.

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mesh_ping/msg/ping.hpp"
#include "rclcpp/rclcpp.hpp"

using mesh_ping::msg::Ping;

class LoadNode : public rclcpp::Node
{
public:
  LoadNode()
  : Node("load_node")
  {
    const int topics = declare_parameter("topics", 1);
    const double rate_hz = declare_parameter("rate_hz", 10.0);
    payload_size_ = declare_parameter<int64_t>("payload_size", 64);
    const bool publish = declare_parameter("publish", true);
    const bool subscribe = declare_parameter("subscribe", true);
    const bool reliable = declare_parameter("reliable", false);
    const int depth = declare_parameter("depth", 10);

    if (topics < 1 || payload_size_ < 0 || (publish && rate_hz <= 0.0)) {
      throw std::invalid_argument("topics must be >= 1, payload_size >= 0, rate_hz > 0");
    }

    rclcpp::QoS qos(static_cast<size_t>(depth));
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    for (int i = 0; i < topics; ++i) {
      const std::string topic = "load_" + std::to_string(i);
      if (publish) {
        pubs_.push_back(create_publisher<Ping>(topic, qos));
      }
      if (subscribe) {
        subs_.push_back(
          create_subscription<Ping>(
            topic, qos, [this](Ping::ConstSharedPtr msg) {
              received_++;
              received_bytes_ += msg->payload.size();
            }));
      }
    }

    if (publish) {
      send_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz), [this] {send();});
    }
    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] {report();});

    RCLCPP_INFO(
      get_logger(), "load start: topics=%d publish=%s subscribe=%s rate=%.1fHz payload=%ldB "
      "(offered %.1f kbit/s)", topics, publish ? "true" : "false", subscribe ? "true" : "false",
      rate_hz, static_cast<long>(payload_size_),
      publish ? topics * rate_hz * static_cast<double>(payload_size_) * 8.0 / 1000.0 : 0.0);
  }

private:
  void send()
  {
    for (auto & pub : pubs_) {
      auto msg = std::make_unique<Ping>();
      msg->seq = seq_;
      msg->payload.resize(static_cast<size_t>(payload_size_));
      pub->publish(std::move(msg));
    }
    seq_++;
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(), "last 5s: sent=%lu recv=%lu (%.1f kbit/s in)",
      static_cast<unsigned long>(seq_ - last_seq_) * pubs_.size(),
      static_cast<unsigned long>(received_), static_cast<double>(received_bytes_) * 8.0 / 5000.0);
    last_seq_ = seq_;
    received_ = 0;
    received_bytes_ = 0;
  }

  int64_t payload_size_;
  uint64_t seq_ = 0;
  uint64_t last_seq_ = 0;
  uint64_t received_ = 0;
  uint64_t received_bytes_ = 0;

  std::vector<rclcpp::Publisher<Ping>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<Ping>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadNode>());
  rclcpp::shutdown();
  return 0;
}
