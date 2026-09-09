// D435i 뎁스 영상으로 전방 장애물을 감지해 속도 명령을 제한하는 안전 필터.
//
//   /cmd_vel_raw  (TwistStamped)  ──┐
//                                   ├─→ /cmd_vel (TwistStamped)
//   /camera/depth/image_rect_raw  ──┘
//
// 상위 노드(텔레오퍼레이션, Nav2 등)는 /cmd_vel_raw 로 명령을 내고,
// 이 노드가 전방이 막혔을 때 전진 성분만 잘라냅니다. 회전은 항상 허용해서
// 장애물 앞에서도 제자리 회전으로 빠져나올 수 있게 합니다.
//
// 뎁스 인코딩은 시뮬(32FC1, m)과 실제 D435i(16UC1, mm) 를 모두 처리합니다.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

namespace
{
constexpr double kNoObstacle = std::numeric_limits<double>::infinity();
}  // namespace

class DepthSafetyFilter : public rclcpp::Node
{
public:
  DepthSafetyFilter()
  : rclcpp::Node("depth_safety_filter")
  {
    stop_distance_ = declare_parameter<double>("stop_distance", 0.45);
    slow_distance_ = declare_parameter<double>("slow_distance", 1.00);
    roi_width_ratio_ = declare_parameter<double>("roi_width_ratio", 0.4);
    roi_height_ratio_ = declare_parameter<double>("roi_height_ratio", 0.5);
    min_valid_depth_ = declare_parameter<double>("min_valid_depth", 0.15);
    depth_timeout_ = declare_parameter<double>("depth_timeout", 1.0);

    if (slow_distance_ <= stop_distance_) {
      RCLCPP_WARN(
        get_logger(),
        "slow_distance(%.2f) 가 stop_distance(%.2f) 이하입니다 — 감속 구간 없이 동작합니다.",
        slow_distance_, stop_distance_);
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    distance_pub_ = create_publisher<std_msgs::msg::Float32>("~/obstacle_distance", 10);

    // 뎁스 영상은 센서 데이터이므로 best effort
    auto sensor_qos = rclcpp::SensorDataQoS();
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_rect_raw", sensor_qos,
      std::bind(&DepthSafetyFilter::onDepth, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_raw", 10,
      std::bind(&DepthSafetyFilter::onCmd, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "depth_safety_filter 시작 (stop=%.2fm, slow=%.2fm)",
      stop_distance_, slow_distance_);
  }

private:
  // ----------------------------------------------------------------
  // 영상 중앙 ROI 안에서 가장 가까운 유효 거리 [m] 를 찾는다.
  double nearestDepth(const sensor_msgs::msg::Image & img) const
  {
    const bool is_float = (img.encoding == "32FC1");
    const bool is_uint16 = (img.encoding == "16UC1" || img.encoding == "mono16");
    if (!is_float && !is_uint16) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "지원하지 않는 뎁스 인코딩: %s", img.encoding.c_str());
      return kNoObstacle;
    }

    const uint32_t w = img.width;
    const uint32_t h = img.height;
    if (w == 0 || h == 0) {
      return kNoObstacle;
    }

    const uint32_t roi_w = std::max<uint32_t>(1, static_cast<uint32_t>(w * roi_width_ratio_));
    const uint32_t roi_h = std::max<uint32_t>(1, static_cast<uint32_t>(h * roi_height_ratio_));
    const uint32_t x0 = (w - roi_w) / 2;
    const uint32_t y0 = (h - roi_h) / 2;

    double nearest = kNoObstacle;

    for (uint32_t v = y0; v < y0 + roi_h; ++v) {
      const uint8_t * row = img.data.data() + static_cast<size_t>(v) * img.step;
      for (uint32_t u = x0; u < x0 + roi_w; ++u) {
        double d;
        if (is_float) {
          float raw;
          std::memcpy(&raw, row + static_cast<size_t>(u) * sizeof(float), sizeof(float));
          d = static_cast<double>(raw);
        } else {
          uint16_t raw;
          std::memcpy(&raw, row + static_cast<size_t>(u) * sizeof(uint16_t), sizeof(uint16_t));
          d = static_cast<double>(raw) * 0.001;  // mm -> m
        }
        // 0 / NaN / inf 는 무효 픽셀 (측정 실패 또는 far clip)
        if (!std::isfinite(d) || d < min_valid_depth_) {
          continue;
        }
        nearest = std::min(nearest, d);
      }
    }
    return nearest;
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    nearest_ = nearestDepth(*msg);
    last_depth_time_ = now();

    std_msgs::msg::Float32 out;
    out.data = std::isfinite(nearest_) ? static_cast<float>(nearest_) :
      std::numeric_limits<float>::infinity();
    distance_pub_->publish(out);
  }

  void onCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    auto out = *msg;

    const bool have_depth =
      last_depth_time_.nanoseconds() > 0 &&
      (now() - last_depth_time_).seconds() < depth_timeout_;

    if (!have_depth) {
      // 뎁스가 끊긴 상태에서 전진을 허용하면 위험하므로 막는다.
      if (out.twist.linear.x > 0.0) {
        out.twist.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "뎁스 영상 없음 — 전진 명령을 차단합니다.");
      }
    } else if (out.twist.linear.x > 0.0) {
      const double scale = speedScale(nearest_);
      if (scale <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "전방 %.2f m 에 장애물 — 정지", nearest_);
      }
      out.twist.linear.x *= scale;
    }

    out.header.stamp = now();
    cmd_pub_->publish(out);
  }

  // 거리에 따른 전진 속도 배율: stop 이하 0, slow 이상 1, 사이는 선형
  double speedScale(double distance) const
  {
    if (!std::isfinite(distance)) {
      return 1.0;
    }
    if (distance <= stop_distance_) {
      return 0.0;
    }
    if (distance >= slow_distance_ || slow_distance_ <= stop_distance_) {
      return 1.0;
    }
    return (distance - stop_distance_) / (slow_distance_ - stop_distance_);
  }

  // ----------------------------------------------------------------
  double stop_distance_;
  double slow_distance_;
  double roi_width_ratio_;
  double roi_height_ratio_;
  double min_valid_depth_;
  double depth_timeout_;

  double nearest_{kNoObstacle};
  rclcpp::Time last_depth_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthSafetyFilter>());
  rclcpp::shutdown();
  return 0;
}
