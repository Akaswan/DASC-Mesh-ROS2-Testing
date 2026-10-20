// Publishes sequenced, timestamped messages on `ping`, listens for the echoes
// on `pong`, and reports round-trip latency, loss, reordering and jitter.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mesh_ping/msg/ping.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"

using mesh_ping::msg::Ping;

namespace
{

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Stats
{
  uint64_t sent = 0;
  uint64_t received = 0;
  uint64_t lost = 0;       // no echo within timeout_s
  uint64_t late = 0;       // echo arrived after being counted lost (or duplicate)
  uint64_t reordered = 0;  // echo arrived after one with a higher seq
  std::vector<double> rtts_ms;
  double jitter_sum_ms = 0.0;
  uint64_t jitter_n = 0;

  void clear() {*this = Stats{};}
};

double percentile(const std::vector<double> & sorted, double q)
{
  const auto idx = static_cast<size_t>(q * static_cast<double>(sorted.size() - 1) + 0.5);
  return sorted[std::min(idx, sorted.size() - 1)];
}

std::string format_stats(const Stats & s)
{
  const uint64_t resolved = s.received + s.lost;
  const double loss_pct = resolved ? 100.0 * static_cast<double>(s.lost) / resolved : 0.0;

  char buf[512];
  int n = std::snprintf(
    buf, sizeof(buf), "sent=%lu recv=%lu lost=%lu (%.2f%%) late=%lu reord=%lu",
    static_cast<unsigned long>(s.sent), static_cast<unsigned long>(s.received),
    static_cast<unsigned long>(s.lost), loss_pct, static_cast<unsigned long>(s.late),
    static_cast<unsigned long>(s.reordered));

  if (!s.rtts_ms.empty()) {
    std::vector<double> sorted = s.rtts_ms;
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;
    for (double v : sorted) {
      sum += v;
    }
    const double jitter = s.jitter_n ? s.jitter_sum_ms / static_cast<double>(s.jitter_n) : 0.0;
    std::snprintf(
      buf + n, sizeof(buf) - n,
      " | rtt ms min=%.2f avg=%.2f p50=%.2f p99=%.2f max=%.2f jitter=%.2f",
      sorted.front(), sum / static_cast<double>(sorted.size()), percentile(sorted, 0.50),
      percentile(sorted, 0.99), sorted.back(), jitter);
  }
  return buf;
}

}  // namespace

class PingNode : public rclcpp::Node
{
public:
  PingNode()
  : Node("ping_node")
  {
    const double rate_hz = declare_parameter("rate_hz", 10.0);
    payload_size_ = declare_parameter<int64_t>("payload_size", 64);
    count_ = declare_parameter<int64_t>("count", 0);  // 0 = run until Ctrl-C
    const double timeout_s = declare_parameter("timeout_s", 2.0);
    const double report_period_s = declare_parameter("report_period_s", 1.0);
    const bool reliable = declare_parameter("reliable", false);
    const int depth = declare_parameter("depth", 10);
    const std::string csv_path = declare_parameter("csv_path", std::string());

    if (rate_hz <= 0.0 || payload_size_ < 0 || report_period_s <= 0.0) {
      throw std::invalid_argument("rate_hz and report_period_s must be > 0, payload_size >= 0");
    }
    timeout_ns_ = static_cast<int64_t>(timeout_s * 1e9);

    // One row per message outcome, for offline analysis (see tools/mesh_test.py).
    if (!csv_path.empty()) {
      csv_ = std::fopen(csv_path.c_str(), "w");
      if (!csv_) {
        throw std::runtime_error("cannot open csv_path: " + csv_path);
      }
      std::fprintf(csv_, "seq,send_ns,rtt_ms,status\n");
    }

    // QoS must match pong_node's, otherwise the two sides never connect.
    rclcpp::QoS qos(static_cast<size_t>(depth));
    if (reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    pub_ = create_publisher<Ping>("ping", qos);
    sub_ = create_subscription<Ping>(
      "pong", qos, [this](Ping::ConstSharedPtr msg) {on_pong(*msg);});

    // Messages sent before DDS discovery completes are silently dropped, which
    // would show up as loss. Poll for the match quickly (independent of rate_hz)
    // so the discovery time is measured to ~5 ms, and only send once matched.
    match_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] {check_matched();});
    send_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this] {send();});
    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period_s), [this] {report();});

    RCLCPP_INFO(
      get_logger(),
      "ping start: rmw=%s qos=%s depth=%d rate=%.1fHz payload=%ldB count=%ld timeout=%.1fs",
      rmw_get_implementation_identifier(), reliable ? "reliable" : "best_effort", depth,
      rate_hz, static_cast<long>(payload_size_), static_cast<long>(count_), timeout_s);
  }

  ~PingNode() override
  {
    if (csv_) {
      std::fclose(csv_);
    }
  }

  void print_summary()
  {
    expire_outstanding();
    std::printf(
      "\n=== mesh_ping summary ===\n%s\nin_flight=%zu\n", format_stats(total_).c_str(),
      outstanding_.size());
  }

private:
  void check_matched()
  {
    if (pub_->get_subscription_count() == 0 || sub_->get_publisher_count() == 0) {
      return;
    }
    matched_ = true;
    match_timer_->cancel();
    // Parsed by tools/mesh_test.py as the discovery time.
    RCLCPP_INFO(
      get_logger(), "pong matched after %.0f ms, sending",
      static_cast<double>(now_ns() - start_ns_) / 1e6);
  }

  void send()
  {
    if (!matched_) {
      return;
    }

    if (count_ > 0 && next_seq_ >= static_cast<uint64_t>(count_)) {
      send_timer_->cancel();
      done_sending_ = true;
      return;
    }

    auto msg = std::make_unique<Ping>();
    msg->seq = next_seq_++;
    msg->payload.resize(static_cast<size_t>(payload_size_));
    msg->send_time_ns = now_ns();
    outstanding_.emplace(msg->seq, msg->send_time_ns);
    pub_->publish(std::move(msg));

    window_.sent++;
    total_.sent++;
  }

  void on_pong(const Ping & msg)
  {
    const int64_t now = now_ns();

    auto it = outstanding_.find(msg.seq);
    const double rtt_ms = static_cast<double>(now - msg.send_time_ns) / 1e6;
    if (it == outstanding_.end()) {
      window_.late++;
      total_.late++;
      write_row(msg.seq, msg.send_time_ns, "late", rtt_ms);
      return;
    }
    outstanding_.erase(it);

    if (have_highest_seq_ && msg.seq < highest_seq_) {
      window_.reordered++;
      total_.reordered++;
      write_row(msg.seq, msg.send_time_ns, "reordered", rtt_ms);
    } else {
      highest_seq_ = msg.seq;
      have_highest_seq_ = true;
      write_row(msg.seq, msg.send_time_ns, "ok", rtt_ms);
    }

    for (Stats * s : {&window_, &total_}) {
      s->received++;
      s->rtts_ms.push_back(rtt_ms);
      if (have_last_rtt_) {
        s->jitter_sum_ms += std::abs(rtt_ms - last_rtt_ms_);
        s->jitter_n++;
      }
    }
    last_rtt_ms_ = rtt_ms;
    have_last_rtt_ = true;
  }

  // Counts every message older than timeout_s that hasn't been echoed as lost.
  // seq and send time increase together, so the oldest are at the front.
  void expire_outstanding()
  {
    const int64_t cutoff = now_ns() - timeout_ns_;
    while (!outstanding_.empty() && outstanding_.begin()->second < cutoff) {
      write_row(outstanding_.begin()->first, outstanding_.begin()->second, "lost");
      outstanding_.erase(outstanding_.begin());
      window_.lost++;
      total_.lost++;
    }
  }

  void write_row(
    uint64_t seq, int64_t send_ns, const char * status,
    double rtt_ms = std::numeric_limits<double>::quiet_NaN())
  {
    if (!csv_) {
      return;
    }
    if (std::isnan(rtt_ms)) {
      std::fprintf(
        csv_, "%lu,%ld,,%s\n", static_cast<unsigned long>(seq), static_cast<long>(send_ns), status);
    } else {
      std::fprintf(
        csv_, "%lu,%ld,%.3f,%s\n", static_cast<unsigned long>(seq), static_cast<long>(send_ns),
        rtt_ms, status);
    }
  }

  void report()
  {
    if (!matched_) {
      RCLCPP_INFO(get_logger(), "waiting for pong_node...");
      return;
    }
    expire_outstanding();
    RCLCPP_INFO(get_logger(), "%s", format_stats(window_).c_str());
    window_.clear();

    if (done_sending_ && outstanding_.empty()) {
      rclcpp::shutdown();
    }
  }

  int64_t payload_size_;
  int64_t count_;
  int64_t timeout_ns_;

  std::FILE * csv_ = nullptr;
  const int64_t start_ns_ = now_ns();

  bool matched_ = false;
  uint64_t next_seq_ = 0;
  bool done_sending_ = false;
  std::map<uint64_t, int64_t> outstanding_;  // seq -> send_time_ns

  uint64_t highest_seq_ = 0;
  bool have_highest_seq_ = false;
  double last_rtt_ms_ = 0.0;
  bool have_last_rtt_ = false;

  Stats window_;
  Stats total_;

  rclcpp::Publisher<Ping>::SharedPtr pub_;
  rclcpp::Subscription<Ping>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr match_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PingNode>();
  rclcpp::spin(node);
  node->print_summary();
  rclcpp::shutdown();
  return 0;
}
