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
// the pad before a flight, frozen at takeoff. Re-enabled by an arming edge (a new run starts with
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
  void on_attitude(double t, const Mat3& R_enu_flu) { t_att_ = t; R_att_ = R_enu_flu; }

  // Call on every VIO message. `speed` = smoothed VIO speed (m/s). Returns true if the anchor
  // was (re)computed from this message.
  bool on_odom(double t, const Iso3& T_odom_base, double speed, bool vo_ok);

  State state() const { return state_; }
  bool anchored() const { return state_ != State::Waiting; }
  const Iso3& T_course_odom() const { return T_course_odom_; }
  const std::string& tilt_source() const { return tilt_source_; }
  bool land_known(double t) const { return t_landed_ >= 0 && t - t_landed_ < p_.px4_timeout_s; }
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
  int anchor_count_ = 0;
};

// ---------------------------------------------------------------------------------------------
// VIO health: receive rate, staleness, path length, speed, and jump detection (a VIO reset or
// teleport). A jump latches until clear_jump().
struct VioParams {
  double jump_dist_m = 0.30;     // a step larger than max(this, jump_speed_mps * dt) is a jump
  double jump_speed_mps = 4.0;
  double jump_yaw_rad = 20.0 * M_PI / 180.0;
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
  bool jump_latched_ = false;
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

}  // namespace team_loc
