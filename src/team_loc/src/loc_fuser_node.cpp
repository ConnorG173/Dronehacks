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
#include <deque>
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
#include <px4_msgs/msg/vehicle_local_position.hpp>
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
    vp.jump_yaw_rate_radps = declare_parameter<double>("jump_yaw_rate_deg_per_s", 170.0) * M_PI / 180.0;
    jump_sigma_xy_m_ = declare_parameter<double>("jump_sigma_xy_m", 0.5);
    jump_sigma_yaw_rad_ = declare_parameter<double>("jump_sigma_yaw_deg", 15.0) * M_PI / 180.0;
    jump_recover_fixes_ = declare_parameter<int>("jump_recover_fixes", 3);
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

    FilterParams fp;
    fp.q_xy_per_sqrt_m = declare_parameter<double>("q_xy_per_sqrt_m", fp.q_xy_per_sqrt_m);
    fp.q_yaw_per_sqrt_m = declare_parameter<double>("q_yaw_deg_per_sqrt_m", 1.0) * M_PI / 180.0;
    fp.max_mahal = declare_parameter<double>("fix_max_mahal", fp.max_mahal);
    fp.reacq_n = declare_parameter<int>("reacq_n", fp.reacq_n);
    fp.reacq_xy_m = declare_parameter<double>("reacq_xy_m", fp.reacq_xy_m);
    fp.slew_mps = declare_parameter<double>("slew_mps", fp.slew_mps);
    fp.slew_radps = declare_parameter<double>("slew_deg_per_s", 5.0) * M_PI / 180.0;
    fp.sigma0_xy_m = cp_.sigma_xy0_m;
    fp.sigma0_z_m = cp_.sigma_z0_m;
    fp.sigma0_yaw_rad = cp_.sigma_yaw0_rad;
    filter_ = std::make_unique<CorrectionFilter>(fp);
    vio_floor_xy_m_ = declare_parameter<double>("vio_floor_xy_m", 0.02);
    max_fix_lag_s_ = declare_parameter<double>("max_fix_lag_s", 1.0);

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
    const auto t_lpos = declare_parameter<std::string>("local_position_topic", "/fmu/out/vehicle_local_position");
    SanityParams snp;
    snp.identical_s = declare_parameter<double>("frozen_identical_s", snp.identical_s);
    snp.max_z_disagree_m = declare_parameter<double>("max_z_disagree_m", snp.max_z_disagree_m);
    max_z_disagree_m_ = snp.max_z_disagree_m;
    sanity_ = std::make_unique<VioSanity>(snp);
    const auto t_out = declare_parameter<std::string>("odom_map_topic", "/team/odom_map");
    const auto t_status = declare_parameter<std::string>("status_topic", "/team/loc/status");
    const auto t_corr = declare_parameter<std::string>("correction_topic", "/team/loc/correction");
    const auto t_fix = declare_parameter<std::string>("fix_topic", "/team/loc/fix");

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
    sub_fix_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        t_fix, rclcpp::QoS(10), [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { on_fix(*m); });
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
    sub_lpos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        t_lpos, sensor_qos, [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m) {
          if (m->dist_bottom_valid && std::isfinite(m->dist_bottom)) sanity_->on_range(now_s(), m->dist_bottom);
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

    if (vio_.on_odom(t, stamp_s(m.header.stamp), T_odom_base)) {
      // A VIO reset/teleport: the correction must absorb it. Widen it and let fixes pull it back;
      // ERROR stays until enough fixes agree (or the next arming).
      filter_->inflate(jump_sigma_xy_m_, jump_sigma_yaw_rad_);
      fixes_since_jump_ = 0;
      RCLCPP_ERROR(get_logger(), "VIO jump detected: status ERROR until %d fixes re-acquire%s",
                   jump_recover_fixes_, correction_enabled_ ? "" : " (correction off: until the next arming)");
    }
    const bool was_anchored = anchor_.anchored();
    if (anchor_.on_odom(t, T_odom_base, vio_.speed(), vo_ok())) {
      path_at_anchor_ = vio_.path_length();
      if (!was_anchored)
        RCLCPP_INFO(get_logger(), "anchored on the pad (tilt from %s)", anchor_.tilt_source().c_str());
    }
    const double ts = stamp_s(m.header.stamp);
    if (!odom_buf_.empty() && ts < odom_buf_.back().first - 1.0) odom_buf_.clear();  // time went back
    if (odom_buf_.empty() || ts > odom_buf_.back().first) odom_buf_.emplace_back(ts, T_odom_base);
    while (!odom_buf_.empty() && ts - odom_buf_.front().first > 3.0) odom_buf_.pop_front();
    if (!anchor_.anchored()) return;  // nothing trustworthy to publish yet

    // The correction only runs in flight. On the pad the anchor is the truth: reset it.
    const bool flying = anchor_.state() == AnchorTracker::State::Frozen;
    if (!flying) {
      filter_->reset();
      applied_correction_ = Correction4{};
    } else if (correction_enabled_ && have_prev_) {
      const double ds = (T_odom_base.translation() - prev_odom_.translation()).norm();
      const double dt = ts - prev_stamp_;
      if (dt > 0) {
        filter_->predict(ds, dt);
        filter_->slew(applied_correction_, dt);
      }
    }
    prev_odom_ = T_odom_base;
    prev_stamp_ = ts;
    have_prev_ = true;

    const Iso3 T_course_odom = applied_correction_.iso() * anchor_.T_course_odom();
    const Iso3 T_course_base = T_course_odom * T_odom_base;
    sanity_->on_odom(t, T_odom_base, T_course_base.translation().z());
    const double dist = vio_.path_length() - path_at_anchor_;
    std::array<double, 36> cov = pose_covariance(cp_, dist, &sigma_xy_, &sigma_yaw_);
    if (correction_enabled_) {
      // Filter covariance at this pose + what the slew has not applied yet + a short-term VIO floor.
      const Iso3 q = anchor_.T_course_odom() * T_odom_base;
      const auto C = filter_->pose_cov(q);
      const Vec3 res_t = filter_->est().t - applied_correction_.t;
      const double res_yaw = wrap_pi(filter_->est().yaw - applied_correction_.yaw);
      const double fl = vio_floor_xy_m_ * vio_floor_xy_m_;
      cov.fill(0.0);
      cov[0] = C(0, 0) + res_t.x() * res_t.x() + fl;
      cov[1] = cov[6] = C(0, 1);
      cov[7] = C(1, 1) + res_t.y() * res_t.y() + fl;
      cov[14] = C(2, 2) + res_t.z() * res_t.z() + fl;
      cov[21] = cov[28] = cp_.sigma_rp_rad * cp_.sigma_rp_rad;
      cov[35] = C(3, 3) + res_yaw * res_yaw;
      cov[5] = cov[30] = C(0, 3);
      cov[11] = cov[31] = C(1, 3);
      const double a = cov[0], b = cov[1], d = cov[7];
      sigma_xy_ = std::sqrt(0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + b * b));  // major axis
      sigma_yaw_ = std::sqrt(cov[35]);
      slew_residual_m_ = std::hypot(res_t.x(), res_t.y());
    }

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

  // T_odom_base at stamp t, interpolated from the VIO buffer (false if outside it).
  bool odom_at(double t, Iso3& out) const {
    if (odom_buf_.size() < 2 || t < odom_buf_.front().first || t > odom_buf_.back().first + 0.05) return false;
    if (t >= odom_buf_.back().first) {
      out = odom_buf_.back().second;
      return true;
    }
    auto hi = std::lower_bound(odom_buf_.begin(), odom_buf_.end(), t,
                               [](const auto& e, double v) { return e.first < v; });
    if (hi == odom_buf_.begin()) {
      out = hi->second;
      return true;
    }
    auto lo = hi - 1;
    const double span = hi->first - lo->first;
    if (span > 0.2) return false;
    const double f = span > 0 ? (t - lo->first) / span : 0.0;
    out = Iso3::Identity();
    out.translation() = (1 - f) * lo->second.translation() + f * hi->second.translation();
    out.linear() = Quat(lo->second.linear()).slerp(f, Quat(hi->second.linear())).toRotationMatrix();
    return true;
  }

  void on_fix(const geometry_msgs::msg::PoseWithCovarianceStamped& m) {
    ++n_fix_in_;
    if (!correction_enabled_ || anchor_.state() != AnchorTracker::State::Frozen) return;
    const double t = stamp_s(m.header.stamp);
    if (!odom_buf_.empty() && odom_buf_.back().first - t > max_fix_lag_s_) {
      ++n_fix_stale_;
      return;
    }
    Iso3 T_odom_base;
    if (!odom_at(t, T_odom_base)) {
      ++n_fix_stale_;
      return;
    }
    const Iso3 q = anchor_.T_course_odom() * T_odom_base;
    const auto& o = m.pose.pose.orientation;
    const double yaw = std::atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z));
    const Vec3 z(m.pose.pose.position.x, m.pose.pose.position.y, yaw);
    const auto& c = m.pose.covariance;
    Mat3 R;
    R << c[0], c[1], c[5], c[6], c[7], c[11], c[30], c[31], c[35];
    double d2 = 0;
    const auto r = filter_->update(q, z, R, &d2);
    last_fix_d2_ = d2;
    if (r == CorrectionFilter::Result::Accepted || r == CorrectionFilter::Result::Reacquired) {
      t_last_fix_ = now_s();
      if (vio_.jump_latched() && ++fixes_since_jump_ >= jump_recover_fixes_) {
        vio_.clear_jump();
        RCLCPP_WARN(get_logger(), "recovered from the VIO jump: %d fixes accepted", fixes_since_jump_);
      }
    }
    if (r == CorrectionFilter::Result::Reacquired)
      RCLCPP_WARN(get_logger(), "re-acquired from %d consistent rejected fixes: correction (%.2f, %.2f) m, %.1f deg",
                  3, filter_->est().t.x(), filter_->est().t.y(), filter_->est().yaw * 180.0 / M_PI);
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
    in.moving = vio_.speed() > 0.1 && anchor_.state() == AnchorTracker::State::Frozen;
    in.fix_age_s = t_last_fix_ < 0 ? -1.0 : t - t_last_fix_;
    in.slew_residual_m = slew_residual_m_;
    in.vio_frozen = sanity_->frozen(t, anchor_.state() == AnchorTracker::State::Frozen);
    in.z_disagree_m = sanity_->z_disagreement(t);
    in.max_z_disagree_m = max_z_disagree_m_;
    in.consecutive_rejected = filter_->consecutive_rejected();
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
    kv("vio_frozen", in.vio_frozen ? "true" : "false");
    kv("z_vio_vs_range_m", in.z_disagree_m < 0 ? "unknown" : fmt(in.z_disagree_m));
    kv("dist_since_anchor_m", fmt(vio_.path_length() - path_at_anchor_, 2));
    kv("sigma_xy_m", fmt(sigma_xy_));
    kv("sigma_yaw_deg", fmt(sigma_yaw_ * 180.0 / M_PI, 2));
    kv("correction_enabled", correction_enabled_ ? "true" : "false");
    if (correction_enabled_) {
      kv("fix_age_s", in.fix_age_s < 0 ? "never" : fmt(in.fix_age_s, 2));
      kv("fixes_in", std::to_string(n_fix_in_));
      kv("fixes_accepted", std::to_string(filter_->accepted()));
      kv("fixes_rejected", std::to_string(filter_->rejected()));
      kv("fixes_stale", std::to_string(n_fix_stale_));
      kv("reacquired", std::to_string(filter_->reacquired()));
      kv("last_fix_mahal", fmt(std::sqrt(last_fix_d2_), 2));
      kv("corr_est_xy_yaw", fmt(filter_->est().t.x()) + " " + fmt(filter_->est().t.y()) + " " +
                                fmt(filter_->est().yaw * 180.0 / M_PI, 2));
      kv("corr_applied_xy_yaw", fmt(applied_correction_.t.x()) + " " + fmt(applied_correction_.t.y()) + " " +
                                    fmt(applied_correction_.yaw * 180.0 / M_PI, 2));
      kv("slew_residual_m", fmt(slew_residual_m_));
    }
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
  Correction4 applied_correction_;  // slews toward filter_->est()
  std::unique_ptr<CorrectionFilter> filter_;
  std::deque<std::pair<double, Iso3>> odom_buf_;
  Iso3 prev_odom_ = Iso3::Identity();
  double prev_stamp_ = 0, t_last_fix_ = -1, last_fix_d2_ = 0, vio_floor_xy_m_ = 0.02, max_fix_lag_s_ = 1.0;
  double slew_residual_m_ = 0;
  bool have_prev_ = false;
  double jump_sigma_xy_m_ = 0.5, jump_sigma_yaw_rad_ = 0.26;
  int jump_recover_fixes_ = 3, fixes_since_jump_ = 0;
  long n_fix_in_ = 0, n_fix_stale_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_fix_;
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
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_lpos_;
  std::unique_ptr<VioSanity> sanity_;
  double max_z_disagree_m_ = 0.35;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace team_loc

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<team_loc::LocFuser>());
  rclcpp::shutdown();
  return 0;
}
