// Corrects motion smear in Livox point clouds by using each point's
// per-point capture time (CustomMsg.timebase + CustomPoint.offset_time)
// to interpolate the platform's pose from a GPS odometry stream, and
// re-projecting every point into a single reference pose for the frame.
//
// Subscribes:
//   <lidar_topic>  livox_interfaces/msg/CustomMsg   (per-point offset_time)
//   <gps_topic>    nav_msgs/msg/Odometry            (platform pose @ ~100Hz)
//
// Publishes:
//   <lidar_topic>/deskewed   sensor_msgs/msg/PointCloud2  (motion-corrected)
//   <lidar_topic>/raw_xyzi   sensor_msgs/msg/PointCloud2  (same points,
//                                                          uncorrected, for
//                                                          side-by-side
//                                                          comparison)
//
// Method
// ------
// For each point i captured at absolute time t_i = timebase + offset_time_i:
//   1. Interpolate the platform pose (position + orientation) at t_i from
//      the GPS odometry buffer -> (R_i, p_i_world)
//   2. Interpolate the platform pose at a single reference time for the
//      whole frame (default: the first point's time, i.e. `timebase`)
//      -> (R_ref, p_ref_world)
//   3. Re-express the point in the reference pose's frame:
//        corrected_point = R_ref^-1 * R_i * point + R_ref^-1 * (p_i - p_ref)
//      This assumes the LiDAR's own sensor frame is rigidly fixed to the
//      platform frame the odometry is published in (a fixed lever arm is
//      NOT separately compensated -- if your GPS/IMU origin and the LiDAR
//      are not co-located, add that static offset for higher accuracy).
//
// Orientation interpolation uses normalized linear interpolation (nlerp)
// rather than full slerp, matching the Python version -- a good
// approximation for the small angular change typical within one LiDAR
// frame period, and cheap to evaluate per point.
//
// Odometry times outside the buffer's current range are CLAMPED to the
// nearest edge sample rather than dropping the whole frame, since the
// last few points of a frame are timestamped close to "now" and the
// matching odometry sample may not have arrived yet (LiDAR and GPS are
// independent, async streams).
//
// Parameters:
//   lidar_topic            (string)  default: /livox/lidar
//   gps_topic              (string)  default: /gps/odometry
//   lidar_qos_reliability  (string)  default: best_effort
//   lidar_qos_depth        (int)     default: 10
//   gps_qos_reliability    (string)  default: best_effort
//   gps_qos_depth          (int)     default: 10
//   odom_buffer_size       (int)     default: 500
//   reference_point        (string)  default: first   ["first" | "last"]
//   publish_raw            (bool)    default: true

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using nav_msgs::msg::Odometry;
using livox_interfaces::msg::CustomMsg;

namespace {

struct OdomSample {
  double t;
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;  // stored normalized
};

double StampToSec(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

rclcpp::QoS BuildQos(const std::string & reliability, int depth) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(depth)));
  if (reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  qos.durability_volatile();
  return qos;
}

}  // namespace

class LidarDeskewNode : public rclcpp::Node
{
public:
  LidarDeskewNode()
  : Node("lidar_deskew_node")
  {
    lidar_topic_ = this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    gps_topic_ = this->declare_parameter<std::string>("gps_topic", "/gps/odometry");
    std::string lidar_qos_reliability =
      this->declare_parameter<std::string>("lidar_qos_reliability", "best_effort");
    int lidar_qos_depth = this->declare_parameter<int>("lidar_qos_depth", 10);
    std::string gps_qos_reliability =
      this->declare_parameter<std::string>("gps_qos_reliability", "best_effort");
    int gps_qos_depth = this->declare_parameter<int>("gps_qos_depth", 10);
    odom_buffer_size_ = this->declare_parameter<int>("odom_buffer_size", 500);
    reference_point_ = this->declare_parameter<std::string>("reference_point", "first");
    publish_raw_ = this->declare_parameter<bool>("publish_raw", true);

    auto lidar_qos = BuildQos(lidar_qos_reliability, lidar_qos_depth);
    auto gps_qos = BuildQos(gps_qos_reliability, gps_qos_depth);

    sub_odom_ = this->create_subscription<Odometry>(
      gps_topic_, gps_qos,
      std::bind(&LidarDeskewNode::OdomCallback, this, std::placeholders::_1));

    sub_lidar_ = this->create_subscription<CustomMsg>(
      lidar_topic_, lidar_qos,
      std::bind(&LidarDeskewNode::LidarCallback, this, std::placeholders::_1));

    pub_deskewed_ = this->create_publisher<PointCloud2>(
      lidar_topic_ + "/deskewed", rclcpp::SensorDataQoS());
    if (publish_raw_) {
      pub_raw_ = this->create_publisher<PointCloud2>(
        lidar_topic_ + "/raw", rclcpp::SensorDataQoS());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Deskewing \"%s\" using odometry from \"%s\" (reference_point=%s)",
      lidar_topic_.c_str(), gps_topic_.c_str(), reference_point_.c_str());
  }

private:
  void OdomCallback(const Odometry::SharedPtr msg)
  {
    OdomSample s;
    s.t = StampToSec(msg->header.stamp);
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    s.pos = Eigen::Vector3d(p.x, p.y, p.z);
    s.quat = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();

    odom_buffer_.push_back(s);
    while (static_cast<int>(odom_buffer_.size()) > odom_buffer_size_) {
      odom_buffer_.pop_front();
    }
  }

  // Returns false only if the buffer has fewer than 2 samples (startup).
  // Otherwise clamps t_query into [buffer front, buffer back] and
  // linearly interpolates position + nlerp's orientation.
  bool InterpolatePose(double t_query, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out)
  {
    if (odom_buffer_.size() < 2) {
      return false;
    }

    const double t_front = odom_buffer_.front().t;
    const double t_back = odom_buffer_.back().t;
    double clamped = std::min(std::max(t_query, t_front), t_back);

    double clamp_amount = std::abs(t_query - clamped);
    if (clamp_amount > 0.02) {  // 20ms -- worth surfacing, not just edge jitter
      max_clamp_seen_ = std::max(max_clamp_seen_, clamp_amount);
      clamp_warn_counter_++;
      if (clamp_warn_counter_ % 20 == 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "Odometry buffer lagging point times by up to %.1f ms -- those "
          "points used the nearest available pose instead of true "
          "interpolation. If this grows, check `ros2 topic hz %s` and "
          "consider increasing odom_buffer_size.",
          clamp_amount * 1e3, gps_topic_.c_str());
      }
    }

    // Binary search for the first sample with t >= clamped.
    auto it = std::lower_bound(
      odom_buffer_.begin(), odom_buffer_.end(), clamped,
      [](const OdomSample & sample, double val) {return sample.t < val;});

    size_t idx1 = static_cast<size_t>(std::distance(odom_buffer_.begin(), it));
    idx1 = std::min(std::max(idx1, static_cast<size_t>(1)), odom_buffer_.size() - 1);
    size_t idx0 = idx1 - 1;

    const OdomSample & s0 = odom_buffer_[idx0];
    const OdomSample & s1 = odom_buffer_[idx1];

    double denom = (s1.t - s0.t);
    double alpha = (denom == 0.0) ? 0.0 : (clamped - s0.t) / denom;

    pos_out = s0.pos + alpha * (s1.pos - s0.pos);

    Eigen::Quaterniond q0 = s0.quat;
    Eigen::Quaterniond q1 = s1.quat;
    // keep quaternions on the same hemisphere before lerp
    if (q0.dot(q1) < 0.0) {
      q1.coeffs() = -q1.coeffs();
    }
    Eigen::Vector4d lerp = (1.0 - alpha) * q0.coeffs() + alpha * q1.coeffs();
    quat_out = Eigen::Quaterniond(lerp).normalized();

    return true;
  }

  void LidarCallback(const CustomMsg::SharedPtr msg)
  {
    const size_t n = msg->points.size();
    if (n == 0) {
      return;
    }

    const double timebase_sec = static_cast<double>(msg->timebase) * 1e-9;

    // First pass: interpolate the per-frame reference pose.
    double t_ref;
    if (reference_point_ == "last") {
      t_ref = timebase_sec + static_cast<double>(msg->points.back().offset_time) * 1e-9;
    } else {
      t_ref = timebase_sec + static_cast<double>(msg->points.front().offset_time) * 1e-9;
    }

    Eigen::Vector3d pos_ref;
    Eigen::Quaterniond quat_ref;
    if (!InterpolatePose(t_ref, pos_ref, quat_ref)) {
      dropped_frames_++;
      if (dropped_frames_ % 20 == 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "Odometry buffer has fewer than 2 samples yet -- dropping frame "
          "(total dropped: %d). Expected only briefly at startup; if this "
          "continues, check that odometry is actually being received "
          "(`ros2 topic hz %s`).",
          dropped_frames_, gps_topic_.c_str());
      }
      return;
    }
    const Eigen::Quaterniond quat_ref_conj = quat_ref.conjugate();

    std::vector<float> deskewed_buf;
    std::vector<float> raw_buf;
    deskewed_buf.reserve(n * 4);
    if (publish_raw_) {
      raw_buf.reserve(n * 4);
    }

    for (const auto & pt : msg->points) {
      const double t_i = timebase_sec + static_cast<double>(pt.offset_time) * 1e-9;

      Eigen::Vector3d pos_i;
      Eigen::Quaterniond quat_i;
      // Buffer already validated to have >=2 samples above; this can only
      // fail to bracket (never fails outright), it clamps instead.
      InterpolatePose(t_i, pos_i, quat_i);

      Eigen::Vector3d point(pt.x, pt.y, pt.z);

      Eigen::Quaterniond q_rel = quat_ref_conj * quat_i;
      Eigen::Vector3d rotated_point = q_rel * point;
      Eigen::Vector3d trans_diff = pos_i - pos_ref;
      Eigen::Vector3d rotated_trans = quat_ref_conj * trans_diff;
      Eigen::Vector3d corrected = rotated_point + rotated_trans;

      deskewed_buf.push_back(static_cast<float>(corrected.x()));
      deskewed_buf.push_back(static_cast<float>(corrected.y()));
      deskewed_buf.push_back(static_cast<float>(corrected.z()));
      deskewed_buf.push_back(static_cast<float>(pt.reflectivity));

      if (publish_raw_) {
        raw_buf.push_back(pt.x);
        raw_buf.push_back(pt.y);
        raw_buf.push_back(pt.z);
        raw_buf.push_back(static_cast<float>(pt.reflectivity));
      }
    }

    PublishCloud(pub_deskewed_, msg->header, deskewed_buf, n);
    if (publish_raw_) {
      PublishCloud(pub_raw_, msg->header, raw_buf, n);
    }
  }

  void PublishCloud(
    const rclcpp::Publisher<PointCloud2>::SharedPtr & publisher,
    const std_msgs::msg::Header & header,
    const std::vector<float> & xyzi_interleaved,
    size_t n)
  {
    PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(n);
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;  // 4x float32: x, y, z, intensity
    cloud.row_step = cloud.point_step * cloud.width;

    PointField fx, fy, fz, fi;
    fx.name = "x"; fx.offset = 0; fx.datatype = PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4; fy.datatype = PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8; fz.datatype = PointField::FLOAT32; fz.count = 1;
    fi.name = "intensity"; fi.offset = 12; fi.datatype = PointField::FLOAT32; fi.count = 1;
    cloud.fields = {fx, fy, fz, fi};

    cloud.data.resize(xyzi_interleaved.size() * sizeof(float));
    std::memcpy(cloud.data.data(), xyzi_interleaved.data(), cloud.data.size());

    publisher->publish(cloud);
  }

  std::string lidar_topic_;
  std::string gps_topic_;
  int odom_buffer_size_;
  std::string reference_point_;
  bool publish_raw_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_deskewed_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_raw_;

  std::deque<OdomSample> odom_buffer_;

  int dropped_frames_ = 0;
  int clamp_warn_counter_ = 0;
  double max_clamp_seen_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
