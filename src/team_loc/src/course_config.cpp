#include "team_loc/course_config.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace team_loc {
namespace {

std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

// "[1, 2 3]" -> {1, 2, 3}; returns false if any token is not a number.
bool parse_numbers(std::string v, std::vector<double>& out) {
  for (char& c : v)
    if (c == '[' || c == ']' || c == ',') c = ' ';
  std::istringstream is(v);
  std::string tok;
  out.clear();
  while (is >> tok) {
    try {
      size_t used = 0;
      double d = std::stod(tok, &used);
      if (used != tok.size()) return false;
      out.push_back(d);
    } catch (const std::exception&) {
      return false;
    }
  }
  return !out.empty();
}

}  // namespace

std::vector<double> CourseConfig::get(const std::string& key, size_t n) const {
  auto it = numbers.find(key);
  if (it == numbers.end() || it->second.empty())
    throw std::runtime_error("course config: missing numeric key '" + key + "'");
  const auto& v = it->second.front();
  if (v.size() != n)
    throw std::runtime_error("course config: '" + key + "' needs " + std::to_string(n) +
                             " numbers, has " + std::to_string(v.size()));
  return v;
}

CourseConfig parse_course_config_text(const std::string& text, const std::string& origin) {
  CourseConfig cfg;
  std::istringstream in(text);
  std::string line;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (line.empty()) continue;
    auto colon = line.find(':');
    if (colon == std::string::npos || colon == 0)
      throw std::runtime_error(origin + ":" + std::to_string(lineno) + ": expected 'Key: value'");
    std::string key = trim(line.substr(0, colon));
    std::string val = trim(line.substr(colon + 1));
    cfg.strings[key].push_back(val);
    std::vector<double> nums;
    if (parse_numbers(val, nums)) cfg.numbers[key].push_back(nums);
  }
  return cfg;
}

CourseConfig load_course_config(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("course config: cannot open '" + path + "'");
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_course_config_text(ss.str(), path);
}

}  // namespace team_loc
