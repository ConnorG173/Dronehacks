#include <gtest/gtest.h>

#include <random>

#include "team_loc/fuser_core.hpp"

using namespace team_loc;

namespace {
Iso3 q_at(double x, double y, double yaw) { return make_iso(rot_z(yaw), Vec3(x, y, 1.0)); }
Vec3 fix_of(const Correction4& truth, const Iso3& q) {
  Iso3 p = truth.iso() * q;
  return Vec3(p.translation().x(), p.translation().y(), yaw_of(p.linear()));
}
Mat3 R_fix(double sxy = 0.04, double syaw_deg = 1.0) {
  Mat3 R = Mat3::Zero();
  R(0, 0) = R(1, 1) = sxy * sxy;
  R(2, 2) = std::pow(syaw_deg * M_PI / 180.0, 2);
  return R;
}
}  // namespace

TEST(Filter, ConvergesToSmallDriftAndShrinksCovariance) {
  CorrectionFilter f{FilterParams{}};
  Correction4 truth;
  truth.t = Vec3(0.06, -0.05, 0);
  truth.yaw = 1.5 * M_PI / 180.0;
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0, 0.02), ny(0, 0.5 * M_PI / 180.0);
  for (int k = 0; k < 40; ++k) {
    f.predict(0.2, 0.2);
    Iso3 q = q_at(-2.0 + 0.1 * k, 0.5 + 0.03 * k, 0.1 * k);
    Vec3 z = fix_of(truth, q) + Vec3(n(rng), n(rng), ny(rng));
    EXPECT_NE(f.update(q, z, R_fix()), CorrectionFilter::Result::Invalid);
  }
  EXPECT_GE(f.accepted(), 38);
  // Compare the corrected pose of the last q (what flies), not the raw state.
  Iso3 q = q_at(1.9, 1.7, 3.9);
  Vec3 e = (f.est().iso() * q).translation() - (truth.iso() * q).translation();
  EXPECT_LT(std::hypot(e.x(), e.y()), 0.03);
  EXPECT_LT(std::abs(wrap_pi(f.est().yaw - truth.yaw)), 0.6 * M_PI / 180.0);
  EXPECT_LT(std::sqrt(f.pose_cov(q)(0, 0)), 0.05);
}

TEST(Filter, RejectsSingleOutlierKeepsState) {
  CorrectionFilter f{FilterParams{}};
  Iso3 q = q_at(-1, 1, 0);
  double d2 = 0;
  EXPECT_EQ(f.update(q, Vec3(-1 + 1.12, 1, 0), R_fix(), &d2), CorrectionFilter::Result::Rejected);  // one bay off
  EXPECT_GT(d2, 12.25);
  EXPECT_NEAR(f.est().t.norm(), 0.0, 1e-12);
  EXPECT_EQ(f.update(q, Vec3(-1.01, 1.0, 0), R_fix()), CorrectionFilter::Result::Accepted);
  EXPECT_EQ(f.consecutive_rejected(), 0);
}

TEST(Filter, ReacquiresAfterConsistentRejections) {
  CorrectionFilter f{FilterParams{}};
  Correction4 truth;
  truth.t = Vec3(0.5, 0.3, 0);
  truth.yaw = 0.0;
  auto r1 = f.update(q_at(-2, 1, 0), fix_of(truth, q_at(-2, 1, 0)), R_fix());
  auto r2 = f.update(q_at(-1.9, 1, 0), fix_of(truth, q_at(-1.9, 1, 0)), R_fix());
  EXPECT_EQ(r1, CorrectionFilter::Result::Rejected);
  EXPECT_EQ(r2, CorrectionFilter::Result::Rejected);
  EXPECT_EQ(f.update(q_at(-1.8, 1, 0), fix_of(truth, q_at(-1.8, 1, 0)), R_fix()),
            CorrectionFilter::Result::Reacquired);
  EXPECT_NEAR(f.est().t.x(), 0.5, 1e-6);
  EXPECT_NEAR(f.est().t.y(), 0.3, 1e-6);
  EXPECT_EQ(f.update(q_at(-1.7, 1, 0), fix_of(truth, q_at(-1.7, 1, 0)), R_fix()),
            CorrectionFilter::Result::Accepted);
}

TEST(Filter, NoReacquireOnInconsistentOutliers) {
  CorrectionFilter f{FilterParams{}};
  Iso3 q = q_at(-1, 1, 0);
  for (double dx : {1.12, -1.12, 1.12, -1.12, 0.8, -0.8})
    EXPECT_EQ(f.update(q, Vec3(-1 + dx, 1, 0), R_fix()), CorrectionFilter::Result::Rejected);
  EXPECT_EQ(f.reacquired(), 0);
  EXPECT_NEAR(f.est().t.norm(), 0.0, 1e-12);
}

TEST(Filter, SlewLimitsAppliedCorrection) {
  FilterParams p;
  CorrectionFilter f{p};
  Correction4 truth;
  truth.t = Vec3(0.5, 0, 0);
  for (double x : {-2.0, -1.9, -1.8}) f.update(q_at(x, 1, 0), fix_of(truth, q_at(x, 1, 0)), R_fix());
  ASSERT_GT(f.reacquired(), 0);
  Correction4 a;
  f.slew(a, 1.0);
  EXPECT_NEAR(a.t.x(), p.slew_mps, 1e-9);          // 0.2 m after 1 s, not 0.5
  for (int i = 0; i < 20; ++i) f.slew(a, 0.1);
  EXPECT_NEAR(a.t.x(), 0.5, 1e-9);                  // and it arrives without overshoot
}

TEST(Filter, PredictGrowsWithDistance) {
  CorrectionFilter f{FilterParams{}};
  const double p0 = f.P()(0, 0);
  f.predict(0.0, 0.0);
  EXPECT_DOUBLE_EQ(f.P()(0, 0), p0);
  f.predict(10.0, 0.0);
  EXPECT_NEAR(f.P()(0, 0) - p0, 0.04 * 0.04 * 10.0, 1e-12);
  // Yaw uncertainty far from the course origin shows up in position.
  Mat3 dummy;
  (void)dummy;
  auto C_near = f.pose_cov(q_at(0, 0, 0));
  auto C_far = f.pose_cov(q_at(3, 0, 0));
  EXPECT_GT(C_far(1, 1), C_near(1, 1));
}
