// Reader for the planner's flat course config (planner/README.md "Inputs"): `Key: value` lines,
// '#' comments, numbers separated by spaces or commas, optional [ ]. Repeated keys
// (Checkpoint) are kept in order. Only what localization needs is interpreted here.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace team_loc {

struct CourseConfig {
  std::map<std::string, std::vector<std::vector<double>>> numbers;  // key -> each occurrence
  std::map<std::string, std::vector<std::string>> strings;          // raw values, each occurrence

  // First occurrence of `key`, which must have exactly `n` numbers. Throws std::runtime_error.
  std::vector<double> get(const std::string& key, size_t n) const;
};

// Throws std::runtime_error with the file and line on a malformed line or unreadable file.
CourseConfig parse_course_config_text(const std::string& text, const std::string& origin = "<text>");
CourseConfig load_course_config(const std::string& path);

}  // namespace team_loc
