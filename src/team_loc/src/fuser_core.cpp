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
  p_ref_ = T_odom_base.translation();
  ++anchor_count_;
}

bool AnchorTracker::on_odom(double t, const Iso3& T_odom_base, double speed, bool vo_ok) {
  if (!vo_ok) return false;
  const bool still = speed < p_.still_speed_mps;

  if (land_known(t)) {
    if (flown_) return false;
    if (landed_) {
      if (state_ == State::Waiting || still) {
        anchor(t, T_odom_base);
        state_ = State::Live;
        return true;
      }
      return false;
    }
    // PX4 says not landed. Takeoff only once the drone has really moved since the last refresh.
    if (state_ == State::Live) {
      if ((T_odom_base.translation() - p_ref_).norm() >= p_.freeze_motion_m) {
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
      if (dist > std::max(p_.jump_dist_m, p_.jump_speed_mps * dt) ||
          ang > std::max(p_.jump_yaw_rad, p_.jump_yaw_rate_radps * dt)) {
        jump = true;
        jump_latched_ = true;
      } else {
        path_len_ += dist;
        speed_ = p_.speed_alpha * (dist / dt) + (1.0 - p_.speed_alpha) * speed_;
        vel_ = p_.speed_alpha * (dp / dt) + (1.0 - p_.speed_alpha) * vel_;
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

// ------------------------------------------------------------------------------------ sanity
void VioSanity::on_odom(double t, const Iso3& T, double z_course) {
  const bool same = have_ && T.translation() == last_.translation() && T.linear() == last_.linear();
  if (!same) identical_since_ = t;
  last_ = T;
  have_ = true;
  zvio_.emplace_back(t, z_course);
  while (!zvio_.empty() && t - zvio_.front().first > 3 * p_.window_s) zvio_.pop_front();
}

void VioSanity::on_range(double t, double r) {
  rng_.emplace_back(t, r);
  while (!rng_.empty() && t - rng_.front().first > 3 * p_.window_s) rng_.pop_front();
}

bool VioSanity::at(const std::deque<std::pair<double, double>>& d, double t, double& v) {
  if (d.empty() || t < d.front().first) return false;
  for (auto it = d.rbegin(); it != d.rend(); ++it)
    if (it->first <= t) {
      v = it->second;
      return true;
    }
  return false;
}

double VioSanity::range_change(double t) const {
  if (rng_.empty() || t - rng_.back().first > 0.3) return -1;
  double r0;
  if (!at(rng_, t - p_.window_s, r0)) return -1;
  return std::abs(rng_.back().second - r0);
}

double VioSanity::z_disagreement(double t) const {
  if (zvio_.empty() || rng_.empty()) return -1;
  if (t - zvio_.back().first > 0.3 || t - rng_.back().first > 0.3) return -1;  // stale input
  double z0, r0;
  if (!at(zvio_, t - p_.window_s, z0) || !at(rng_, t - p_.window_s, r0)) return -1;
  return std::abs((rng_.back().second - r0) - (zvio_.back().second - z0));
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
  if (in.vio_frozen) err.push_back("VIO frozen (static pose, rangefinder moving)");

  if (in.vio_ever && in.vio_age_s <= p.vio_timeout_s && in.vio_rate_hz < p.min_rate_hz)
    warn.push_back("VIO rate low");
  if (in.vo_state == 3) warn.push_back("VIO heuristic (vo_state 3)");
  if (in.jump_absorbed) warn.push_back("VIO jump absorbed");
  if (in.vio_static && !in.vio_frozen) warn.push_back("VIO pose static in flight");
  if (in.z_disagree_m > in.max_z_disagree_m) warn.push_back("VIO height disagrees with rangefinder");
  if (in.vo_state == 0) warn.push_back("VIO state unknown (vo_state 0)");
  if (in.correction_enabled) {
    if (in.sigma_xy_m > p.warn_sigma_xy_m) warn.push_back("sigma_xy high");
    if (in.sigma_yaw_rad > p.warn_sigma_yaw_rad) warn.push_back("sigma_yaw high");
    if (in.moving && (in.fix_age_s < 0 || in.fix_age_s > p.warn_fix_age_s)) warn.push_back("no recent fix");
    if (in.slew_residual_m > p.warn_slew_residual_m) warn.push_back("correction slewing");
    if (in.consecutive_rejected > p.warn_consecutive_rejected) warn.push_back("fixes rejected");
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

// ------------------------------------------------------------------------------------ filter
void CorrectionFilter::reset() {
  est_ = Correction4{};
  P_.setZero();
  P_(0, 0) = P_(1, 1) = p_.sigma0_xy_m * p_.sigma0_xy_m;
  P_(2, 2) = p_.sigma0_z_m * p_.sigma0_z_m;
  P_(3, 3) = p_.sigma0_yaw_rad * p_.sigma0_yaw_rad;
  rej_.clear();
}

void CorrectionFilter::predict(double ds, double dt) {
  ds = std::max(0.0, ds);
  dt = std::max(0.0, dt);
  const double qxy = p_.q_xy_per_sqrt_m * p_.q_xy_per_sqrt_m * ds + p_.q_xy_per_sqrt_s * p_.q_xy_per_sqrt_s * dt;
  P_(0, 0) += qxy;
  P_(1, 1) += qxy;
  P_(2, 2) += p_.q_z_per_sqrt_m * p_.q_z_per_sqrt_m * ds;
  P_(3, 3) += p_.q_yaw_per_sqrt_m * p_.q_yaw_per_sqrt_m * ds + p_.q_yaw_per_sqrt_s * p_.q_yaw_per_sqrt_s * dt;
}

Correction4 CorrectionFilter::implied(const Iso3& q, const Vec3& z) const {
  Correction4 c;
  c.yaw = wrap_pi(z.z() - yaw_of(q.linear()));
  const Vec3 rq = rot_z(c.yaw) * q.translation();
  c.t = Vec3(z.x() - rq.x(), z.y() - rq.y(), est_.t.z());
  return c;
}

CorrectionFilter::Result CorrectionFilter::update(const Iso3& q, const Vec3& z, const Mat3& R, double* d2_out) {
  if (!z.allFinite() || !R.allFinite()) return Result::Invalid;
  const double c = std::cos(est_.yaw), s = std::sin(est_.yaw);
  const Vec3 qp = q.translation();
  // Prediction h(C) = (Rz(yaw) q_xy + t_xy, yaw_q + yaw).
  const double hx = c * qp.x() - s * qp.y() + est_.t.x();
  const double hy = s * qp.x() + c * qp.y() + est_.t.y();
  const double hyaw = wrap_pi(yaw_of(q.linear()) + est_.yaw);
  Eigen::Matrix<double, 3, 4> H = Eigen::Matrix<double, 3, 4>::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(0, 3) = -s * qp.x() - c * qp.y();
  H(1, 3) = c * qp.x() - s * qp.y();
  H(2, 3) = 1;
  Vec3 nu(z.x() - hx, z.y() - hy, wrap_pi(z.z() - hyaw));
  const Mat3 S = H * P_ * H.transpose() + R;
  const double d2 = nu.dot(S.ldlt().solve(nu));
  if (d2_out) *d2_out = d2;

  if (d2 > p_.max_mahal * p_.max_mahal) {
    ++n_rej_;
    rej_.push_back(implied(q, z));
    while (static_cast<int>(rej_.size()) > p_.reacq_n) rej_.pop_front();
    if (static_cast<int>(rej_.size()) == p_.reacq_n) {
      // Re-acquire if the rejected fixes agree with each other: VIO jumped or drifted beyond the gate.
      Vec3 mt = Vec3::Zero();
      double sy = 0, cy = 0;
      for (const auto& r : rej_) {
        mt += r.t;
        sy += std::sin(r.yaw);
        cy += std::cos(r.yaw);
      }
      mt /= rej_.size();
      const double my = std::atan2(sy, cy);
      double max_dxy = 0, max_dyaw = 0;
      for (const auto& r : rej_) {
        max_dxy = std::max(max_dxy, std::hypot(r.t.x() - mt.x(), r.t.y() - mt.y()));
        max_dyaw = std::max(max_dyaw, std::abs(wrap_pi(r.yaw - my)));
      }
      if (max_dxy < p_.reacq_xy_m && max_dyaw < p_.reacq_yaw_rad) {
        est_.t.x() = mt.x();
        est_.t.y() = mt.y();
        est_.yaw = my;
        const double vxy = std::max(R(0, 0), R(1, 1)) + max_dxy * max_dxy;
        P_(0, 0) = P_(1, 1) = std::max(vxy, p_.sigma0_xy_m * p_.sigma0_xy_m);
        P_(0, 1) = P_(1, 0) = 0;
        P_(0, 3) = P_(3, 0) = P_(1, 3) = P_(3, 1) = 0;
        P_(3, 3) = std::max(R(2, 2) + max_dyaw * max_dyaw, p_.sigma0_yaw_rad * p_.sigma0_yaw_rad);
        rej_.clear();
        ++n_reacq_;
        return Result::Reacquired;
      }
    }
    return Result::Rejected;
  }

  const Eigen::Matrix<double, 4, 3> K = P_ * H.transpose() * S.inverse();
  const Eigen::Vector4d dx = K * nu;
  est_.t.x() += dx(0);
  est_.t.y() += dx(1);
  est_.t.z() += dx(2);
  est_.yaw = wrap_pi(est_.yaw + dx(3));
  const Mat4 I_KH = Mat4::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();  // Joseph form
  rej_.clear();
  ++n_acc_;
  return Result::Accepted;
}

void CorrectionFilter::slew(Correction4& a, double dt) const {
  dt = std::max(0.0, dt);
  const Vec3 d = est_.t - a.t;
  const double n = d.norm(), lim = p_.slew_mps * dt;
  a.t += (n > lim && n > 0) ? Vec3(d * (lim / n)) : d;
  const double dy = wrap_pi(est_.yaw - a.yaw), ly = p_.slew_radps * dt;
  a.yaw = wrap_pi(a.yaw + std::clamp(dy, -ly, ly));
}

void CorrectionFilter::inflate(double sxy, double syaw) {
  P_(0, 0) = std::max(P_(0, 0), sxy * sxy);
  P_(1, 1) = std::max(P_(1, 1), sxy * sxy);
  P_(3, 3) = std::max(P_(3, 3), syaw * syaw);
  rej_.clear();
}

CorrectionFilter::Mat4 CorrectionFilter::pose_cov(const Iso3& q) const {
  const double c = std::cos(est_.yaw), s = std::sin(est_.yaw);
  const Vec3 qp = q.translation();
  Mat4 J = Mat4::Identity();
  J(0, 3) = -s * qp.x() - c * qp.y();
  J(1, 3) = c * qp.x() - s * qp.y();
  return J * P_ * J.transpose();
}

}  // namespace team_loc
