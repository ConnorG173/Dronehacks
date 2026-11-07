#include <gtest/gtest.h>

#include "team_loc/pose_math.hpp"

using namespace team_loc;

TEST(PoseMath, WrapPi) {
  EXPECT_NEAR(wrap_pi(3 * M_PI / 2), -M_PI / 2, 1e-12);
  EXPECT_NEAR(wrap_pi(-3 * M_PI / 2), M_PI / 2, 1e-12);
  EXPECT_NEAR(wrap_pi(0.3), 0.3, 1e-12);
}

TEST(PoseMath, TiltOnlyRemovesHeading) {
  Mat3 R = rot_z(1.2) * Eigen::AngleAxisd(0.1, Vec3::UnitY()).toRotationMatrix();
  Mat3 T = tilt_only(R);
  EXPECT_NEAR(yaw_of(T), 0.0, 1e-9);
  // Same gravity direction in body frame.
  EXPECT_TRUE((R.transpose() * Vec3::UnitZ()).isApprox(T.transpose() * Vec3::UnitZ(), 1e-9));
}

TEST(PoseMath, Px4LevelHeadings) {
  // PX4 level, heading north (q = identity, FRD->NED): body x = north = ENU +y.
  Mat3 R = px4_q_to_enu_flu(1, 0, 0, 0);
  EXPECT_TRUE((R * Vec3::UnitX()).isApprox(Vec3::UnitY(), 1e-12));
  EXPECT_TRUE((R * Vec3::UnitZ()).isApprox(Vec3::UnitZ(), 1e-12));  // FLU up = ENU up
  EXPECT_NEAR(yaw_of(R), M_PI / 2, 1e-12);
  // Heading east: NED yaw +90 deg about down. ENU yaw 0.
  const double h = M_PI / 4;  // half angle of 90 deg
  Mat3 E = px4_q_to_enu_flu(std::cos(h), 0, 0, std::sin(h));
  EXPECT_NEAR(yaw_of(E), 0.0, 1e-12);
}

TEST(PoseMath, Px4PitchUpIsPositiveFluPitch) {
  // Nose up 10 deg in FRD/NED is a positive rotation about body y (right). In FLU, nose up is a
  // negative rotation about y (left): body x gets a +z component either way.
  const double a = 10.0 * M_PI / 180.0;
  Mat3 R = px4_q_to_enu_flu(std::cos(a / 2), 0, std::sin(a / 2), 0);
  EXPECT_GT((R * Vec3::UnitX()).z(), 0.15);
}

TEST(PoseMath, AnchorReproducesLaunchPose) {
  const Vec3 start(-2.5146, 0.9144, 0.0);
  const double heading = 30.0 * M_PI / 180.0;
  // odom: the stack started 1 m away at a different heading, tilted a little.
  Iso3 T_odom_base = make_iso(rot_z(-0.7) * Eigen::AngleAxisd(0.02, Vec3::UnitX()).toRotationMatrix(),
                              Vec3(1.0, -0.3, 0.05));
  Mat3 tilt = Eigen::AngleAxisd(0.02, Vec3::UnitX()).toRotationMatrix();
  Iso3 T_course_base = launch_pose(start, heading, 0.12, tilt);
  Iso3 A = anchor_course_odom(T_course_base, T_odom_base);
  Iso3 got = A * T_odom_base;
  EXPECT_TRUE(got.translation().isApprox(start + Vec3(0, 0, 0.12), 1e-9));
  EXPECT_NEAR(yaw_of(got.linear()), heading, 1e-9);
  // A later odom motion of 1 m forward (body x) lands 1 m along the start heading in course.
  Iso3 moved = T_odom_base * make_iso(Mat3::Identity(), Vec3(1, 0, 0));
  Vec3 d = (A * moved).translation() - got.translation();
  EXPECT_NEAR(d.x(), std::cos(heading), 1e-3);
  EXPECT_NEAR(d.y(), std::sin(heading), 1e-3);
}
