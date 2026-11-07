#include <gtest/gtest.h>

#include "team_loc/fuser_core.hpp"

using namespace team_loc;

namespace {
AnchorParams params() {
  AnchorParams p;
  p.start_pos = Vec3(-2.5, 0.9, 0.0);
  p.start_heading_rad = 0.0;
  p.base_height_m = 0.1;
  return p;
}
Iso3 at(double x, double y, double yaw = 0) { return make_iso(rot_z(yaw), Vec3(x, y, 0)); }
Vec3 course_pos(const AnchorTracker& a, const Iso3& T) { return (a.T_course_odom() * T).translation(); }
}  // namespace

TEST(Anchor, WithPx4RefreshesWhileLandedThenFreezesAtTakeoff) {
  AnchorTracker a(params());
  double t = 0;
  a.on_attitude(t, Mat3::Identity());
  a.on_landed(t, true);
  EXPECT_TRUE(a.on_odom(t, at(5, 5), 0.0, true));
  EXPECT_EQ(a.state(), AnchorTracker::State::Live);
  EXPECT_EQ(a.tilt_source(), "px4_attitude");
  // VIO creeps 3 cm while sitting on the pad: still anchored to the launch mark.
  t += 0.5;
  a.on_landed(t, true);
  a.on_attitude(t, Mat3::Identity());
  EXPECT_TRUE(a.on_odom(t, at(5.03, 5), 0.0, true));
  EXPECT_TRUE(course_pos(a, at(5.03, 5)).isApprox(Vec3(-2.5, 0.9, 0.1), 1e-9));
  // Takeoff: frozen, motion is followed.
  t += 0.5;
  a.on_landed(t, false);
  EXPECT_FALSE(a.on_odom(t, at(6.03, 5), 0.5, true));
  EXPECT_EQ(a.state(), AnchorTracker::State::Frozen);
  EXPECT_TRUE(course_pos(a, at(6.03, 5)).isApprox(Vec3(-1.5, 0.9, 0.1), 1e-9));
  // Landing somewhere else must not re-anchor.
  t += 5;
  a.on_landed(t, true);
  EXPECT_FALSE(a.on_odom(t, at(7, 4), 0.0, true));
  EXPECT_TRUE(course_pos(a, at(7, 4)).isApprox(Vec3(-0.53, -0.1, 0.1), 1e-9));
  // A new arming (new run, drone back on the launch mark) re-anchors.
  t += 1;
  a.on_armed(t, false);
  a.on_armed(t, true);
  EXPECT_TRUE(a.on_odom(t, at(2, 2), 0.0, true));
  EXPECT_TRUE(course_pos(a, at(2, 2)).isApprox(Vec3(-2.5, 0.9, 0.1), 1e-9));
  EXPECT_EQ(a.anchor_count(), 3);
}

TEST(Anchor, DoesNotAnchorWhenVioLostOrMovingOnPad) {
  AnchorTracker a(params());
  a.on_landed(0, true);
  EXPECT_FALSE(a.on_odom(0, at(0, 0), 0.0, false));
  EXPECT_FALSE(a.anchored());
  EXPECT_TRUE(a.on_odom(0.1, at(0, 0), 0.0, true));       // first anchor even if not still
  EXPECT_FALSE(a.on_odom(0.2, at(0.1, 0), 0.3, true));    // moving: no refresh
  EXPECT_TRUE(course_pos(a, at(0, 0)).isApprox(Vec3(-2.5, 0.9, 0.1), 1e-9));
}

TEST(Anchor, NoPx4FallbackFreezesAtFirstMotion) {
  AnchorTracker a(params());
  EXPECT_TRUE(a.on_odom(0, at(1, 1), 0.0, true));
  EXPECT_EQ(a.tilt_source(), "odom");
  EXPECT_TRUE(a.on_odom(0.1, at(1.02, 1), 0.0, true));
  EXPECT_FALSE(a.on_odom(0.2, at(1.2, 1), 0.4, true));
  EXPECT_EQ(a.state(), AnchorTracker::State::Frozen);
  EXPECT_TRUE(course_pos(a, at(1.2, 1)).isApprox(Vec3(-2.32, 0.9, 0.1), 1e-9));
}

TEST(Anchor, HeadingRotatesOdomMotion) {
  auto p = params();
  p.start_heading_rad = M_PI / 2;  // drone faces course +y
  AnchorTracker a(p);
  a.on_landed(0, true);
  ASSERT_TRUE(a.on_odom(0, at(0, 0, 0.4), 0.0, true));  // odom heading 0.4 rad, arbitrary
  a.on_landed(0.1, false);
  a.on_odom(0.1, at(0, 0, 0.4), 0.0, true);
  // 1 m along the body's forward direction in odom.
  Iso3 fwd = at(0, 0, 0.4) * make_iso(Mat3::Identity(), Vec3(1, 0, 0));
  EXPECT_TRUE(course_pos(a, fwd).isApprox(Vec3(-2.5, 1.9, 0.1), 1e-9));
}

TEST(Vio, RateJumpAndPath) {
  VioMonitor v(VioParams{});
  double t = 0;
  for (int i = 0; i < 31; ++i, t += 1.0 / 30) EXPECT_FALSE(v.on_odom(t, t, at(0.01 * i, 0)));
  EXPECT_NEAR(v.rate_hz(t - 1.0 / 30), 30.0, 1.5);
  EXPECT_NEAR(v.path_length(), 0.30, 1e-9);
  EXPECT_NEAR(v.speed(), 0.3, 0.05);
  EXPECT_FALSE(v.jump_latched());
  EXPECT_TRUE(v.on_odom(t, t, at(1.5, 0)));  // 1.2 m in one frame
  EXPECT_TRUE(v.jump_latched());
  EXPECT_NEAR(v.path_length(), 0.30, 1e-9);  // a jump is not travel
  v.clear_jump();
  EXPECT_FALSE(v.jump_latched());
  EXPECT_TRUE(v.on_odom(t + 0.033, t + 0.033, at(1.5, 0, 0.6)));  // 34 deg yaw step
  EXPECT_GT(v.age(t + 1.0), 0.9);
}

TEST(Vio, LongGapWithPlausibleMotionIsNotAJump) {
  VioMonitor v(VioParams{});
  EXPECT_FALSE(v.on_odom(0.0, 0.0, at(0, 0, 0)));
  // 1 s without VIO, then 0.7 m and 60 deg later: plausible for a drone, not a teleport.
  EXPECT_FALSE(v.on_odom(1.0, 1.0, at(0.7, 0, 60.0 * M_PI / 180.0)));
  EXPECT_FALSE(v.jump_latched());
  // The same 60 deg within one frame is a jump.
  EXPECT_TRUE(v.on_odom(1.033, 1.033, at(0.7, 0, 0)));
}

TEST(Status, Levels) {
  StatusParams p;
  StatusInputs in;
  EXPECT_EQ(evaluate_status(in, p).level, 2);  // no VIO yet
  in.vio_ever = true;
  in.vio_age_s = 0.03;
  in.vio_rate_hz = 30;
  in.anchored = true;
  in.vo_state = 1;
  EXPECT_EQ(evaluate_status(in, p).level, 0);
  in.vio_rate_hz = 15;
  EXPECT_EQ(evaluate_status(in, p).level, 1);
  in.vio_rate_hz = 30;
  in.vo_state = 2;
  EXPECT_EQ(evaluate_status(in, p).level, 2);
  in.vo_state = -1;  // no status topic: not an error by itself
  EXPECT_EQ(evaluate_status(in, p).level, 0);
  in.vio_age_s = 0.5;
  auto r = evaluate_status(in, p);
  EXPECT_EQ(r.level, 2);
  EXPECT_NE(r.message.find("stale"), std::string::npos);
  in.vio_age_s = 0.03;
  // Sigma thresholds only matter with correction enabled.
  in.sigma_xy_m = 0.3;
  EXPECT_EQ(evaluate_status(in, p).level, 0);
  in.correction_enabled = true;
  in.fix_age_s = 1.0;
  EXPECT_EQ(evaluate_status(in, p).level, 1);
  in.sigma_xy_m = 0.6;
  EXPECT_EQ(evaluate_status(in, p).level, 2);
  in.sigma_xy_m = 0.05;
  in.moving = true;
  in.fix_age_s = -1;
  EXPECT_EQ(evaluate_status(in, p).level, 1);
  in.jump_latched = true;
  EXPECT_EQ(evaluate_status(in, p).level, 2);
}

TEST(Cov, GrowsWithDistance) {
  CovParams p;
  double s0, s10, y0, y10;
  pose_covariance(p, 0, &s0, &y0);
  auto c = pose_covariance(p, 10, &s10, &y10);
  EXPECT_NEAR(s0, 0.05, 1e-12);
  EXPECT_NEAR(s10, std::hypot(0.05, 0.2), 1e-12);
  EXPECT_GT(y10, y0);
  EXPECT_DOUBLE_EQ(c[0], c[7]);
  EXPECT_GT(c[35], 0);
}
