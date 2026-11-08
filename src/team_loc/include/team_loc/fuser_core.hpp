// Pure logic of loc_fuser (no ROS): the pad anchor, VIO health, status levels, covariance.
// Times are seconds on one clock (the node clock). See docs/proposals/localization.md, sections 2-3.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "team_loc/pose_math.hpp"

namespace team_loc {

// ---------------------------------------------------------------------------------------------
// Pad anchor. T_course_odom = launch_pose * T_odom_base(now)^-1, refreshed while the drone sits on
// the pad before a flight, frozen at takeoff = PX4 not landed AND moved freeze_motion_m since the last
// refresh (PX4's land detector can report not-landed on the pad after arming, e.g. throttle centred). Re-enabled by an arming edge (a new run starts with
// the drone placed on the launch mark and armed there). Never re-anchors after a flight without a
// new arming, because after landing the drone is on the landing pad, not the launch mark.
struct AnchorParams {
  Vec3 start_pos = Vec3::Zero();
  double start_heading_rad = 0.0;
  double base_height_m = 0.0;     // base_link above the floor while on the pad (vehicle value)
  double still_speed_mps = 0.05;  // refresh only while slower than this
  double freeze_motion_m = 0.10;  // without PX4 landed info: freeze once moved this far
  double px4_timeout_s = 1.0;     // PX4 topic older than this counts as unknown
};

class AnchorTracker {
 public:
  enum class State { Waiting, Live, Frozen };

  explicit AnchorTracker(const AnchorParams& p) : p_(p) {}

  void on_landed(double t, bool landed) { t_landed_ = t; landed_ = landed; }
  void on_armed(double t, bool armed);
  // A VIO discontinuity (teleport): keep the course pose continuous across it by folding the step
  // into the anchor, T_course_odom <- T_course_odom * T_pre * T_post^-1.
  void absorb(const Iso3& T_odom_base_pre, const Iso3& T_odom_base_post) {
    T_course_odom_ = T_course_odom_ * T_odom_base_pre * T_odom_base_post.inverse();
    p_ref_ = T_odom_base_post.translation();
  }
  void on_attitude(double t, const Mat3& R_enu_flu) { t_att_ = t; R_att_ = R_enu_flu; }

  // Call on every VIO message. `speed` = smoothed VIO speed (m/s). Returns true if the anchor
  // was (re)computed from this message.
  bool on_odom(double t, const Iso3& T_odom_base, double speed, bool vo_ok);

  State state() const { return state_; }
  bool anchored() const { return state_ != State::Waiting; }
  const Iso3& T_course_odom() const { return T_course_odom_; }
  const std::string& tilt_source() const { return tilt_source_; }
  bool land_known(double t) const { return t_landed_ >= 0 && t - t_landed_ < p_.px4_timeout_s; }
  // In the air: took off (frozen anchor) and PX4 does not say landed (after landing it does).
  bool airborne(double t) const { return state_ == State::Frozen && !(land_known(t) && landed_); }
  int anchor_count() const { return anchor_count_; }

 private:
  void anchor(double t, const Iso3& T_odom_base);

  AnchorParams p_;
  State state_ = State::Waiting;
  Iso3 T_course_odom_ = Iso3::Identity();
  std::string tilt_source_ = "none";
  double t_landed_ = -1, t_att_ = -1;
  bool landed_ = false, flown_ = false, armed_ = false, armed_known_ = false;
  Mat3 R_att_ = Mat3::Identity();
  Vec3 p_first_ = Vec3::Zero();
  Vec3 p_ref_ = Vec3::Zero();   // odom position at the last anchor refresh
  int anchor_count_ = 0;
};

// ---------------------------------------------------------------------------------------------
// VIO health: receive rate, staleness, path length, speed, and jump detection (a VIO reset or
// teleport). A jump latches until clear_jump().
struct VioParams {
  double jump_dist_m = 0.30;     // a step larger than max(this, jump_speed_mps * dt) is a jump
  double jump_speed_mps = 4.0;
  double jump_yaw_rad = 20.0 * M_PI / 180.0;   // a rotation larger than max(this, rate * dt) is a jump
  double jump_yaw_rate_radps = 3.0;
  double rate_window_s = 1.0;
  double speed_alpha = 0.3;      // EMA weight of the newest speed sample
};

class VioMonitor {
 public:
  explicit VioMonitor(const VioParams& p) : p_(p) {}

  // Returns true if this message is a jump.
  bool on_odom(double t_recv, double t_stamp, const Iso3& T_odom_base);

  bool ever() const { return n_ > 0; }
  double age(double now) const { return n_ ? now - t_recv_last_ : 1e9; }
  double rate_hz(double now) const;
  double speed() const { return speed_; }
  // Last good (non-jump) velocity in odom, m/s (EMA): the motion to keep across a jump.
  const Vec3& velocity() const { return vel_; }
  double path_length() const { return path_len_; }
  bool jump_latched() const { return jump_latched_; }
  void clear_jump() { jump_latched_ = false; }
  long count() const { return n_; }

 private:
  VioParams p_;
  long n_ = 0;
  double t_recv_last_ = 0, t_stamp_last_ = 0;
  Iso3 T_last_ = Iso3::Identity();
  std::deque<double> recv_times_;
  double speed_ = 0, path_len_ = 0;
  Vec3 vel_ = Vec3::Zero();
  bool jump_latched_ = false;
};

// ---------------------------------------------------------------------------------------------
// VIO sanity against things vision cannot fake. cuVSLAM can report vo_state 1 with a frozen pose
// (seen in sim on blank images): bit-identical poses while airborne -> frozen. And the rangefinder
// (PX4 dist_bottom) is independent of vision: its height change over a window should match VIO's.
struct SanityParams {
  double identical_s = 1.0;      // identical poses this long while airborne = frozen
  double window_s = 1.0;         // height-change comparison window
  double max_z_disagree_m = 0.35;
  double frozen_range_m = 0.15;  // rangefinder change over the window that proves motion
};

class VioSanity {
 public:
  explicit VioSanity(const SanityParams& p) : p_(p) {}
  void on_odom(double t, const Iso3& T_odom_base, double z_course);
  void on_range(double t, double dist_bottom_m);
  // Bit-identical poses for identical_s while airborne (a noiseless sim can do this when still).
  bool static_pose(double t, bool airborne) const {
    return airborne && have_ && (t - identical_since_) >= p_.identical_s;
  }
  // Static VIO while the rangefinder says the drone moved: frozen.
  bool frozen(double t, bool airborne) const {
    const double r = range_change(t);
    return static_pose(t, airborne) && r > p_.frozen_range_m;
  }
  // |range change| over the last window; -1 if unknown.
  double range_change(double t) const;
  // |range change - VIO z change| over the last window; -1 if either series is too short.
  double z_disagreement(double t) const;

 private:
  static bool at(const std::deque<std::pair<double, double>>& d, double t, double& v);
  SanityParams p_;
  bool have_ = false;
  Iso3 last_ = Iso3::Identity();
  double identical_since_ = 0;
  std::deque<std::pair<double, double>> zvio_, rng_;
};

// ---------------------------------------------------------------------------------------------
// Status levels (diagnostic_msgs/DiagnosticStatus OK = 0, WARN = 1, ERROR = 2).
struct StatusParams {
  double vio_timeout_s = 0.2;
  double min_rate_hz = 20.0;
  double warn_sigma_xy_m = 0.15;   // sigma thresholds apply only with correction enabled
  double warn_sigma_yaw_rad = 5.0 * M_PI / 180.0;
  double error_sigma_xy_m = 0.5;
  double warn_fix_age_s = 5.0;
  double warn_slew_residual_m = 0.10;
  int warn_consecutive_rejected = 3;
};

struct StatusInputs {
  bool vio_ever = false;
  double vio_age_s = 1e9;
  double vio_rate_hz = 0;
  int vo_state = -1;               // -1 = no status message seen
  bool jump_latched = false;
  bool anchored = false;
  bool correction_enabled = false;
  double sigma_xy_m = 0, sigma_yaw_rad = 0;
  double fix_age_s = -1;           // -1 = no fix ever (only checked with correction enabled)
  bool moving = false;
  double slew_residual_m = 0;      // estimated minus applied correction (correction enabled only)
  int consecutive_rejected = 0;
  bool vio_frozen = false;
  bool vio_static = false;
  bool jump_absorbed = false;      // a VIO jump was folded into the anchor recently
  double z_disagree_m = -1;        // VIO vs rangefinder height change; -1 = unknown
  double max_z_disagree_m = 0.35;
};

struct StatusResult {
  uint8_t level = 0;
  std::string message;
  std::vector<std::string> reasons;
};

StatusResult evaluate_status(const StatusInputs& in, const StatusParams& p);

// ---------------------------------------------------------------------------------------------
// Pose covariance (row-major 6x6: x, y, z, roll, pitch, yaw) of the course pose: anchor
// uncertainty plus VIO drift growing linearly with distance flown since the anchor/last fix.
struct CovParams {
  double sigma_xy0_m = 0.05;
  double sigma_z0_m = 0.03;
  double sigma_yaw0_rad = 2.0 * M_PI / 180.0;
  double sigma_rp_rad = 1.0 * M_PI / 180.0;
  double drift_frac = 0.02;                     // m of 1-sigma drift per m flown
  double drift_yaw_rad_per_m = 0.5 * M_PI / 180.0;
};

std::array<double, 36> pose_covariance(const CovParams& p, double dist_m,
                                       double* sigma_xy = nullptr, double* sigma_yaw = nullptr);

// ---------------------------------------------------------------------------------------------
// EKF on the 4-DOF correction C = (t, yaw), applied on the left of the anchor A:
//   T_course_base = C * A * T_odom_base.
// The state is a slowly varying offset, so a fix stamped at the image time (100-300 ms old) can be
// applied to the current state without replaying VIO: the correction barely changes over that
// interval (process noise only). Fixes measure (x, y, yaw) of base_link in the course frame.
struct FilterParams {
  double sigma0_xy_m = 0.05, sigma0_z_m = 0.03, sigma0_yaw_rad = 2.0 * M_PI / 180.0;
  double q_xy_per_sqrt_m = 0.04;                    // random-walk growth with distance flown
  double q_z_per_sqrt_m = 0.02;
  double q_yaw_per_sqrt_m = 1.0 * M_PI / 180.0;
  double q_xy_per_sqrt_s = 0.005;                   // and with time
  double q_yaw_per_sqrt_s = 0.1 * M_PI / 180.0;
  double max_mahal = 3.5;                           // 3-dof gate (chi^2 12.25)
  int reacq_n = 3;                                  // consecutive rejected fixes that agree...
  double reacq_xy_m = 0.15;                         // ...within this of their mean
  double reacq_yaw_rad = 5.0 * M_PI / 180.0;
  double slew_mps = 0.20;                           // applied correction rate limits
  double slew_radps = 5.0 * M_PI / 180.0;
};

class CorrectionFilter {
 public:
  enum class Result { Accepted, Rejected, Reacquired, Invalid };
  using Mat4 = Eigen::Matrix4d;

  explicit CorrectionFilter(const FilterParams& p) : p_(p) { reset(); }

  void reset();
  void predict(double ds_m, double dt_s);
  // q = A * T_odom_base(t_fix): the anchor-only course pose at the fix stamp.
  // z = (x, y, yaw) measured; R = its 3x3 covariance. d2_out = Mahalanobis^2 of the innovation.
  Result update(const Iso3& q, const Vec3& z, const Mat3& R, double* d2_out = nullptr);
  // Move `applied` toward the estimate by at most the slew limits over dt.
  void slew(Correction4& applied, double dt_s) const;
  // After a VIO jump: widen the state uncertainty so fixes can pull it back.
  void inflate(double sigma_xy_m, double sigma_yaw_rad);

  // Covariance (x, y, z, yaw) of the course pose C * q for a current anchor-only pose q.
  Mat4 pose_cov(const Iso3& q) const;

  const Correction4& est() const { return est_; }
  const Mat4& P() const { return P_; }
  int accepted() const { return n_acc_; }
  int rejected() const { return n_rej_; }
  int consecutive_rejected() const { return static_cast<int>(rej_.size()); }
  int reacquired() const { return n_reacq_; }

 private:
  // The correction that would make the fix exact: (t, yaw) with z = C * q.
  Correction4 implied(const Iso3& q, const Vec3& z) const;

  FilterParams p_;
  Correction4 est_;
  Mat4 P_ = Mat4::Identity();
  std::deque<Correction4> rej_;
  int n_acc_ = 0, n_rej_ = 0, n_reacq_ = 0;
};

}  // namespace team_loc
