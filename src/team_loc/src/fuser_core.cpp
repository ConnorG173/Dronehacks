#include "team_loc/fuser_core.hpp"

#include <algorithm>
#include <cmath>

namespace team_loc {

// ------------------------------------------------------------------------------------ anchor
void AnchorTracker::on_armed(double, bool armed) {
  if (armed && (!armed_known_ || !armed_)) {
    // Arming edge: a new run. Allow re-anchoring while still landed on the launch mark.
    flown_ = false;
    if (state_ == State::Frozen) state_ = State::Live;
    p_first_ = Vec3::Constant(NAN);  // re-arm the no-PX4 fallback too
  }
  armed_ = armed;
  armed_known_ = true;
}

void AnchorTracker::anchor(double t, const Iso3& T_odom_base) {
  Mat3 tilt;
  if (t_att_ >= 0 && t - t_att_ < p_.px4_timeout_s) {
    tilt = R_att_;
    tilt_source_ = "px4_attitude";
  } else {
    tilt = T_odom_base.linear();  // assumes odom's z is up (stack started level)
    tilt_source_ = "odom";
  }
  Iso3 T_course_base = launch_pose(p_.start_pos, p_.start_heading_rad, p_.base_height_m, tilt);
  T_course_odom_ = anchor_course_odom(T_course_base, T_odom_base);
  ++anchor_count_;
}

bool AnchorTracker::on_odom(double t, const Iso3& T_odom_base, double speed, bool vo_ok) {
  if (!vo_ok) return false;
  const bool still = speed < p_.still_speed_mps;

  if (land_known(t)) {
    if (landed_ && !flown_) {
      if (state_ == State::Waiting || still) {
        anchor(t, T_odom_base);
        state_ = State::Live;
        return true;
      }
      return false;
    }
    if (!landed_ && state_ == State::Live) {  // takeoff
      state_ = State::Frozen;
      flown_ = true;
    }
    return false;
  }

  // No PX4 landed info: anchor at the first good message, refresh while it stays put, and freeze
  // at the first real motion.
  const Vec3 p = T_odom_base.translation();
  if (state_ == State::Waiting || (state_ == State::Live && !p_first_.allFinite())) {
    p_first_ = p;
    anchor(t, T_odom_base);
    state_ = State::Live;
    return true;
  }
  if (state_ == State::Live) {
    if ((p - p_first_).norm() >= p_.freeze_motion_m) {
      state_ = State::Frozen;
      flown_ = true;
      return false;
    }
    if (still) {
      anchor(t, T_odom_base);
      return true;
    }
  }
  return false;
}

// ------------------------------------------------------------------------------------ VIO
bool VioMonitor::on_odom(double t_recv, double t_stamp, const Iso3& T) {
  bool jump = false;
  if (n_ > 0) {
    const double dt = t_stamp - t_stamp_last_;
    const Vec3 dp = T.translation() - T_last_.translation();
    const double dist = dp.norm();
    const double ang = Eigen::AngleAxisd(T_last_.linear().transpose() * T.linear()).angle();
    if (dt > 0) {
      if (dist > std::max(p_.jump_dist_m, p_.jump_speed_mps * dt) || ang > p_.jump_yaw_rad) {
        jump = true;
        jump_latched_ = true;
      } else {
        path_len_ += dist;
        speed_ = p_.speed_alpha * (dist / dt) + (1.0 - p_.speed_alpha) * speed_;
      }
    }
  }
  ++n_;
  t_recv_last_ = t_recv;
  t_stamp_last_ = t_stamp;
  T_last_ = T;
  recv_times_.push_back(t_recv);
  while (!recv_times_.empty() && t_recv - recv_times_.front() > p_.rate_window_s)
    recv_times_.pop_front();
  return jump;
}

double VioMonitor::rate_hz(double now) const {
  long k = 0;
  for (double t : recv_times_)
    if (now - t <= p_.rate_window_s) ++k;
  return k / p_.rate_window_s;
}

// ------------------------------------------------------------------------------------ status
StatusResult evaluate_status(const StatusInputs& in, const StatusParams& p) {
  StatusResult r;
  std::vector<std::string> err, warn;
  if (!in.vio_ever) err.push_back("no VIO yet");
  else if (in.vio_age_s > p.vio_timeout_s) err.push_back("VIO stale");
  if (in.vo_state == 2) err.push_back("VIO lost (vo_state 2)");
  if (in.jump_latched) err.push_back("VIO jump");
  if (!in.anchored) err.push_back("not anchored");
  if (in.correction_enabled && in.sigma_xy_m > p.error_sigma_xy_m) err.push_back("sigma_xy too large");

  if (in.vio_ever && in.vio_age_s <= p.vio_timeout_s && in.vio_rate_hz < p.min_rate_hz)
    warn.push_back("VIO rate low");
  if (in.vo_state == 3) warn.push_back("VIO heuristic (vo_state 3)");
  if (in.vo_state == 0) warn.push_back("VIO state unknown (vo_state 0)");
  if (in.correction_enabled) {
    if (in.sigma_xy_m > p.warn_sigma_xy_m) warn.push_back("sigma_xy high");
    if (in.sigma_yaw_rad > p.warn_sigma_yaw_rad) warn.push_back("sigma_yaw high");
    if (in.moving && (in.fix_age_s < 0 || in.fix_age_s > p.warn_fix_age_s)) warn.push_back("no recent fix");
  }

  if (!err.empty()) {
    r.level = 2;
    r.reasons = err;
  } else if (!warn.empty()) {
    r.level = 1;
    r.reasons = warn;
  } else {
    r.level = 0;
    r.reasons = {"ok"};
  }
  for (size_t i = 0; i < r.reasons.size(); ++i) r.message += (i ? "; " : "") + r.reasons[i];
  // Lower-level reasons are still useful to see.
  if (r.level == 2 && !warn.empty()) {
    r.message += " (also: ";
    for (size_t i = 0; i < warn.size(); ++i) r.message += (i ? "; " : "") + warn[i];
    r.message += ")";
  }
  return r;
}

// ------------------------------------------------------------------------------------ cov
std::array<double, 36> pose_covariance(const CovParams& p, double d, double* sxy, double* syaw) {
  d = std::max(0.0, d);
  const double s_xy = std::hypot(p.sigma_xy0_m, p.drift_frac * d);
  const double s_z = std::hypot(p.sigma_z0_m, p.drift_frac * d);
  const double s_yaw = std::hypot(p.sigma_yaw0_rad, p.drift_yaw_rad_per_m * d);
  std::array<double, 36> c{};
  c[0] = c[7] = s_xy * s_xy;
  c[14] = s_z * s_z;
  c[21] = c[28] = p.sigma_rp_rad * p.sigma_rp_rad;
  c[35] = s_yaw * s_yaw;
  if (sxy) *sxy = s_xy;
  if (syaw) *syaw = s_yaw;
  return c;
}

}  // namespace team_loc
