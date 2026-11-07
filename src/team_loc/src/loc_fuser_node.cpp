// loc_fuser: cuVSLAM odometry -> course frame. Milestone 1 (anchor + republish + status).
//
//   in   /visual_slam/tracking/odometry  nav_msgs/Odometry (odom -> base_link)
//        /visual_slam/status             VisualSlamStatus (vo_state)
//        /fmu/out/vehicle_attitude       roll/pitch for the anchor
//        /fmu/out/vehicle_land_detected  landed (anchor refresh / freeze)
//        /fmu/out/vehicle_status_v1      arming edge (re-anchor for a new run)
//   out  /team/odom_map                  nav_msgs/Odometry, frame `course`, every VIO message
//        TF course -> map                (parent of cuVSLAM's map -> odom; see CONTRACTS.md)
//        /team/loc/status                diagnostic_msgs/DiagnosticStatus, 10 Hz
//        /team/loc/correction            geometry_msgs/PoseWithCovarianceStamped: T_course_odom
//
// Reads only /fmu/out/*. Never publishes setpoints or anything to PX4.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "team_loc/course_config.hpp"
#include "team_loc/fuser_core.hpp"

using namespace std::chrono_literals;

namespace team_loc {

namespace {
double stamp_s(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

Iso3 to_iso(const geometry_msgs::msg::Pose& p) {
  Iso3 T;
  tf2::fromMsg(p, T);
  return T;
}

std::string fmt(double v, int prec = 3) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}
}  // namespace

class LocFuser : public rclcpp::Node {
 public:
  LocFuser() : Node("loc_fuser"), anchor_(AnchorParams{}), vio_(VioParams{}) {
    // ---- parameters
    const auto course_config = declare_parameter<std::string>("course_config", "");
    AnchorParams ap;
    ap.base_height_m = declare_parameter<double>("base_height_on_pad_m", 0.0);
    ap.still_speed_mps = declare_parameter<double>("still_speed_mps", ap.still_speed_mps);
    ap.freeze_motion_m = declare_parameter<double>("freeze_motion_m", ap.freeze_motion_m);
    ap.px4_timeout_s = declare_parameter<double>("px4_timeout_s", ap.px4_timeout_s);
    if (course_config.empty()) throw std::runtime_error("parameter course_config is required");
    const CourseConfig cfg = load_course_config(course_config);
    const auto sp = cfg.get("StartPos", 3);
    ap.start_pos = Vec3(sp[0], sp[1], sp[2]);
    ap.start_heading_rad = cfg.get("StartHeadingDeg", 1)[0] * M_PI / 180.0;
    anchor_ = AnchorTracker(ap);

    VioParams vp;
    vp.jump_dist_m = declare_parameter<double>("jump_dist_m", vp.jump_dist_m);
    vp.jump_speed_mps = declare_parameter<double>("jump_speed_mps", vp.jump_speed_mps);
    vp.jump_yaw_rad = declare_parameter<double>("jump_yaw_deg", 20.0) * M_PI / 180.0;
    vio_ = VioMonitor(vp);

    sp_.vio_timeout_s = declare_parameter<double>("vio_timeout_s", sp_.vio_timeout_s);
    sp_.min_rate_hz = declare_parameter<double>("min_rate_hz", sp_.min_rate_hz);
    sp_.warn_sigma_xy_m = declare_parameter<double>("warn_sigma_xy_m", sp_.warn_sigma_xy_m);
    sp_.error_sigma_xy_m = declare_parameter<double>("error_sigma_xy_m", sp_.error_sigma_xy_m);
    sp_.warn_sigma_yaw_rad = declare_parameter<double>("warn_sigma_yaw_deg", 5.0) * M_PI / 180.0;
    sp_.warn_fix_age_s = declare_parameter<double>("warn_fix_age_s", sp_.warn_fix_age_s);

    cp_.sigma_xy0_m = declare_parameter<double>("anchor_sigma_xy_m", cp_.sigma_xy0_m);
    cp_.sigma_z0_m = declare_parameter<double>("anchor_sigma_z_m", cp_.sigma_z0_m);
    cp_.sigma_yaw0_rad = declare_parameter<double>("anchor_sigma_yaw_deg", 2.0) * M_PI / 180.0;
    cp_.drift_frac = declare_parameter<double>("drift_frac", cp_.drift_frac);
    cp_.drift_yaw_rad_per_m = declare_parameter<double>("drift_yaw_deg_per_m", 0.5) * M_PI / 180.0;

    correction_enabled_ = declare_parameter<bool>("enable_correction", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    course_frame_ = declare_parameter<std::string>("course_frame", "course");
    tf_child_frame_ = declare_parameter<std::string>("tf_child_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    const double status_rate = declare_parameter<double>("status_rate_hz", 10.0);

    const auto t_odom = declare_parameter<std::string>("odom_topic", "/visual_slam/tracking/odometry");
    const auto t_vo = declare_parameter<std::string>("vo_status_topic", "/visual_slam/status");
    const auto t_att = declare_parameter<std::string>("attitude_topic", "/fmu/out/vehicle_attitude");
    const auto t_land = declare_parameter<std::string>("land_detected_topic", "/fmu/out/vehicle_land_detected");
    const auto t_vs = declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    const auto t_out = declare_parameter<std::string>("odom_map_topic", "/team/odom_map");
    const auto t_status = declare_parameter<std::string>("status_topic", "/team/loc/status");
    const auto t_corr = declare_parameter<std::string>("correction_topic", "/team/loc/correction");

    // ---- I/O
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(t_out, rclcpp::QoS(10));
    pub_status_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(t_status, rclcpp::QoS(10));
    pub_corr_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(t_corr, rclcpp::QoS(10));
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        t_odom, sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    sub_vo_ = create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(
        t_vo, sensor_qos, [this](isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::ConstSharedPtr m) {
          vo_state_ = m->vo_state;
          t_vo_ = now_s();
        });
    sub_att_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        t_att, sensor_qos, [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr m) {
          anchor_.on_attitude(now_s(), px4_q_to_enu_flu(m->q[0], m->q[1], m->q[2], m->q[3]));
        });
    sub_land_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        t_land, sensor_qos, [this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr m) {
          anchor_.on_landed(now_s(), m->landed);
        });
    sub_vs_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        t_vs, sensor_qos, [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m) {
          const bool armed = m->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
          if (armed && !armed_) vio_.clear_jump();  // a new run starts clean
          armed_ = armed;
          anchor_.on_armed(now_s(), armed);
        });

    status_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / status_rate),
                                      [this] { publish_status(); });
    RCLCPP_INFO(get_logger(), "course %s: start (%.3f, %.3f, %.3f), heading %.1f deg, base height %.3f m",
                course_config.c_str(), ap.start_pos.x(), ap.start_pos.y(), ap.start_pos.z(),
                ap.start_heading_rad * 180.0 / M_PI, ap.base_height_m);
  }

 private:
  double now_s() { return get_clock()->now().seconds(); }

  bool vo_ok() const {
    // Unknown vo_state (no status topic) does not block anchoring; "lost" does.
    return vo_state_ != 2;
  }

  void on_odom(const nav_msgs::msg::Odometry& m) {
    const auto t0 = std::chrono::steady_clock::now();
    const double t = now_s();
    const Iso3 T_odom_base = to_iso(m.pose.pose);

    if (vio_.on_odom(t, stamp_s(m.header.stamp), T_odom_base))
      RCLCPP_ERROR(get_logger(), "VIO jump detected: status ERROR until the next arming");
    const bool was_anchored = anchor_.anchored();
    if (anchor_.on_odom(t, T_odom_base, vio_.speed(), vo_ok())) {
      path_at_anchor_ = vio_.path_length();
      if (!was_anchored)
        RCLCPP_INFO(get_logger(), "anchored on the pad (tilt from %s)", anchor_.tilt_source().c_str());
    }
    if (!anchor_.anchored()) return;  // nothing trustworthy to publish yet

    const Iso3 T_course_odom = applied_correction_.iso() * anchor_.T_course_odom();
    const Iso3 T_course_base = T_course_odom * T_odom_base;
    const double dist = vio_.path_length() - path_at_anchor_;
    const auto cov = pose_covariance(cp_, dist, &sigma_xy_, &sigma_yaw_);

    nav_msgs::msg::Odometry out;
    out.header.stamp = m.header.stamp;
    out.header.frame_id = course_frame_;
    out.child_frame_id = m.child_frame_id.empty() ? "base_link" : m.child_frame_id;
    out.pose.pose = tf2::toMsg(T_course_base);
    std::copy(cov.begin(), cov.end(), out.pose.covariance.begin());
    out.twist = m.twist;  // body FLU, unchanged
    pub_odom_->publish(out);

    if (publish_tf_) publish_tf(m.header.stamp, T_course_odom);

    // Debug: the full correction (where odom's origin sits in the course frame).
    geometry_msgs::msg::PoseWithCovarianceStamped c;
    c.header.stamp = m.header.stamp;
    c.header.frame_id = course_frame_;
    c.pose.pose = tf2::toMsg(T_course_odom);
    pub_corr_->publish(c);

    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    proc_us_.push_back(us);
    if (proc_us_.size() > 300) proc_us_.erase(proc_us_.begin());
  }

  void publish_tf(const builtin_interfaces::msg::Time& stamp, const Iso3& T_course_odom) {
    // TF is a tree: odom already has cuVSLAM's map as parent, so we publish course -> map with
    // T_course_map = T_course_odom * T_map_odom^-1 (T_map_odom live from TF, identity if absent).
    Iso3 T_course_child = T_course_odom;
    if (tf_child_frame_ != odom_frame_) {
      Iso3 T_map_odom = Iso3::Identity();
      try {
        const auto tr = tf_buffer_->lookupTransform(tf_child_frame_, odom_frame_, tf2::TimePointZero);
        T_map_odom = tf2::transformToEigen(tr);
        map_odom_tf_ok_ = true;
      } catch (const tf2::TransformException&) {
        map_odom_tf_ok_ = false;
      }
      T_course_child = T_course_odom * T_map_odom.inverse();
    }
    geometry_msgs::msg::TransformStamped tr = tf2::eigenToTransform(T_course_child);
    tr.header.stamp = stamp;
    tr.header.frame_id = course_frame_;
    tr.child_frame_id = tf_child_frame_;
    tf_pub_->sendTransform(tr);
  }

  void publish_status() {
    const double t = now_s();
    StatusInputs in;
    in.vio_ever = vio_.ever();
    in.vio_age_s = vio_.age(t);
    in.vio_rate_hz = vio_.rate_hz(t);
    in.vo_state = (t_vo_ >= 0 && t - t_vo_ < 1.0) ? vo_state_ : -1;
    in.jump_latched = vio_.jump_latched();
    in.anchored = anchor_.anchored();
    in.correction_enabled = correction_enabled_;
    in.sigma_xy_m = sigma_xy_;
    in.sigma_yaw_rad = sigma_yaw_;
    in.moving = vio_.speed() > 0.1;
    const StatusResult r = evaluate_status(in, sp_);

    diagnostic_msgs::msg::DiagnosticStatus s;
    s.level = r.level;
    s.name = "team_loc";
    s.hardware_id = "loc_fuser";
    s.message = r.message;
    auto kv = [&s](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue p;
      p.key = k;
      p.value = v;
      s.values.push_back(p);
    };
    const char* st[] = {"waiting", "live", "frozen"};
    kv("anchor", st[static_cast<int>(anchor_.state())]);
    kv("anchor_tilt_source", anchor_.tilt_source());
    kv("anchor_count", std::to_string(anchor_.anchor_count()));
    kv("land_known", anchor_.land_known(t) ? "true" : "false");
    kv("vo_state", std::to_string(in.vo_state));
    kv("vio_rate_hz", fmt(in.vio_rate_hz, 1));
    kv("vio_age_s", fmt(std::min(in.vio_age_s, 999.0)));
    kv("vio_jump", in.jump_latched ? "true" : "false");
    kv("dist_since_anchor_m", fmt(vio_.path_length() - path_at_anchor_, 2));
    kv("sigma_xy_m", fmt(sigma_xy_));
    kv("sigma_yaw_deg", fmt(sigma_yaw_ * 180.0 / M_PI, 2));
    kv("correction_enabled", correction_enabled_ ? "true" : "false");
    kv("map_odom_tf", tf_child_frame_ == odom_frame_ ? "unused" : (map_odom_tf_ok_ ? "ok" : "missing"));
    if (!proc_us_.empty()) {
      std::vector<double> v = proc_us_;
      std::sort(v.begin(), v.end());
      kv("proc_us_p50", fmt(v[v.size() / 2], 0));
      kv("proc_us_p95", fmt(v[std::min(v.size() - 1, v.size() * 95 / 100)], 0));
    }
    pub_status_->publish(s);
  }

  AnchorTracker anchor_;
  VioMonitor vio_;
  StatusParams sp_;
  CovParams cp_;
  Correction4 applied_correction_;  // identity until milestone 3
  bool correction_enabled_ = false, publish_tf_ = true, armed_ = false, map_odom_tf_ok_ = false;
  std::string course_frame_, tf_child_frame_, odom_frame_;
  int vo_state_ = -1;
  double t_vo_ = -1, path_at_anchor_ = 0, sigma_xy_ = 0, sigma_yaw_ = 0;
  std::vector<double> proc_us_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_corr_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr sub_vo_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_att_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr sub_land_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_vs_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace team_loc

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<team_loc::LocFuser>());
  rclcpp::shutdown();
  return 0;
}
