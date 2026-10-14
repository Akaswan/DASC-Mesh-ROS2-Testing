// Echoes every message received on `ping` straight back on `pong`.

#include <memory>
#include <utility>

#include "mesh_ping/msg/ping.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"

using mesh_ping::msg::Ping;

class PongNode : public rclcpp::Node
{
public:
  PongNode()
  : Node("pong_node")
  {
    const bool reliable = declare_parameter("reliable", false);
    const int depth = declare_parameter("depth", 10);
    echo_payload_ = declare_parameter("echo_payload", true);

    // QoS must match ping_node's, otherwise the two sides never connect.
    rclcpp::QoS qos(static_cast<size_t>(depth));
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    pub_ = create_publisher<Ping>("pong", qos);
    sub_ = create_subscription<Ping>(
      "ping", qos, [this](Ping::UniquePtr msg) {
        if (!echo_payload_) {
          msg->payload.clear();
        }
        pub_->publish(std::move(msg));
      });

    RCLCPP_INFO(
      get_logger(), "pong ready: rmw=%s qos=%s depth=%d echo_payload=%s",
      rmw_get_implementation_identifier(), reliable ? "reliable" : "best_effort",
      depth, echo_payload_ ? "true" : "false");
  }

private:
  bool echo_payload_;
  rclcpp::Publisher<Ping>::SharedPtr pub_;
  rclcpp::Subscription<Ping>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PongNode>());
  rclcpp::shutdown();
  return 0;
}
