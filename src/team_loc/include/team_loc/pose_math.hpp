// Pose helpers for the course <- odom correction. Pure Eigen, no ROS.
//
// Frames (docs/CONTRACTS.md, docs/proposals/localization.md section 2):
//   course     planner frame: metres, ENU-like, z up, origin = cage floor centre
//   odom       cuVSLAM's ENU-like frame, origin = base_link where the stack started
//   base_link  body FLU (origin at the Pixhawk IMU)
// PX4 reports attitude as FRD body -> NED world (Hamilton quaternion w, x, y, z).
#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace team_loc {

using Iso3 = Eigen::Isometry3d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

inline double wrap_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

inline Mat3 rot_z(double yaw) { return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix(); }

// Heading of the body x axis projected on the horizontal plane, CCW from world +x.
inline double yaw_of(const Mat3& R) { return std::atan2(R(1, 0), R(0, 0)); }

// R with its heading removed: Rz(-yaw) * R. Keeps roll and pitch only.
inline Mat3 tilt_only(const Mat3& R) { return rot_z(-yaw_of(R)) * R; }

inline Iso3 make_iso(const Mat3& R, const Vec3& t) {
  Iso3 T = Iso3::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

// PX4 attitude (FRD -> NED, q = w, x, y, z) as a rotation FLU -> ENU.
inline Mat3 px4_q_to_enu_flu(double w, double x, double y, double z) {
  Mat3 R_ned_frd = Quat(w, x, y, z).normalized().toRotationMatrix();
  Mat3 ned_to_enu;
  ned_to_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  Mat3 flu_to_frd = Vec3(1, -1, -1).asDiagonal();
  return ned_to_enu * R_ned_frd * flu_to_frd;
}

// Launch pose of base_link in the course frame:
//   position = start_pos + (0, 0, base_height), heading = start_heading, roll/pitch = tilt.
// `tilt` is any rotation whose heading is ignored (e.g. PX4 attitude, or the odom pose).
inline Iso3 launch_pose(const Vec3& start_pos, double start_heading_rad, double base_height,
                        const Mat3& tilt) {
  return make_iso(rot_z(start_heading_rad) * tilt_only(tilt), start_pos + Vec3(0, 0, base_height));
}

// The anchor: T_course_odom such that T_course_odom * T_odom_base = T_course_base.
inline Iso3 anchor_course_odom(const Iso3& T_course_base, const Iso3& T_odom_base) {
  return T_course_base * T_odom_base.inverse();
}

// 4-DOF correction applied on the left in the course frame: (t, yaw).
struct Correction4 {
  Vec3 t = Vec3::Zero();
  double yaw = 0.0;
  Iso3 iso() const { return make_iso(rot_z(yaw), t); }
};

}  // namespace team_loc
