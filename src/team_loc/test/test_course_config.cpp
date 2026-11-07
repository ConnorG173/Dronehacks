#include <gtest/gtest.h>

#include <stdexcept>

#include "team_loc/course_config.hpp"

using namespace team_loc;

TEST(CourseConfig, ParsesFlatKeyValue) {
  auto c = parse_course_config_text(
      "# comment\nStartPos:  [-2.5, 0.9, 0.0]   # pad\nStartHeadingDeg: 90\n"
      "Checkpoint: [1, 2, 3, 0.1, 0.1, 0.1]\nCheckpoint: 4 5 6 0.1 0.1 0.1\nMapFile: ../maps/x.bin\n");
  auto sp = c.get("StartPos", 3);
  EXPECT_DOUBLE_EQ(sp[0], -2.5);
  EXPECT_DOUBLE_EQ(c.get("StartHeadingDeg", 1)[0], 90.0);
  ASSERT_EQ(c.numbers.at("Checkpoint").size(), 2u);
  EXPECT_DOUBLE_EQ(c.numbers.at("Checkpoint")[1][0], 4.0);
  EXPECT_EQ(c.strings.at("MapFile")[0], "../maps/x.bin");
  EXPECT_THROW(c.get("StartPos", 2), std::runtime_error);
  EXPECT_THROW(c.get("Goal", 3), std::runtime_error);
}

TEST(CourseConfig, RejectsMalformedLine) {
  EXPECT_THROW(parse_course_config_text("StartPos [1 2 3]\n"), std::runtime_error);
}

TEST(CourseConfig, RealCourseFile) {
  auto c = load_course_config(COURSE_CONFIG_PATH);
  auto sp = c.get("StartPos", 3);
  EXPECT_NEAR(sp[0], -2.5146, 1e-9);
  EXPECT_NEAR(sp[1], 0.9144, 1e-9);
  EXPECT_EQ(c.get("StartHeadingDeg", 1).size(), 1u);
}
