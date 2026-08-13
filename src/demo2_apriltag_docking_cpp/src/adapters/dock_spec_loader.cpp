#include "demo2_apriltag_docking_cpp/adapters/dock_spec_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <charconv>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_set>

namespace demo2_apriltag_docking_cpp::adapters {
namespace {

int parse_tag_id(const YAML::Node & node)
{
  if (!node.IsScalar()) {
    throw std::invalid_argument("invalid tag id");
  }
  const std::string raw = node.Scalar();
  const char * begin = raw.data();
  const char * end = begin + raw.size();
  while (begin != end && std::isspace(static_cast<unsigned char>(*begin)) != 0) {
    ++begin;
  }
  while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1))) != 0) {
    --end;
  }
  int tag_id = 0;
  const auto result = std::from_chars(begin, end, tag_id);
  if (result.ec != std::errc() || result.ptr != end || tag_id < 0) {
    throw std::invalid_argument("invalid tag id: " + raw);
  }
  return tag_id;
}

std::string required_field(const YAML::Node & dock, const std::string & name, int tag_id)
{
  const YAML::Node value = dock[name];
  if (!value.IsScalar()) {
    throw std::invalid_argument("tag " + std::to_string(tag_id) + " has missing or empty fields");
  }
  const std::string text = value.Scalar();
  if (text.find_first_not_of(" \t\n\r\f\v") == std::string::npos) {
    throw std::invalid_argument("tag " + std::to_string(tag_id) + " has missing or empty fields");
  }
  return text;
}

}  // namespace

std::unordered_map<int, core::DockSpec> load_dock_specs(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node docks = root["docks"];
  if (!docks.IsMap() || docks.size() == 0U) {
    throw std::invalid_argument("docks must be a non-empty mapping");
  }

  std::unordered_map<int, core::DockSpec> specs;
  std::unordered_set<std::string> dock_ids;
  for (const auto & entry : docks) {
    const int tag_id = parse_tag_id(entry.first);
    if (specs.find(tag_id) != specs.end() || !entry.second.IsMap()) {
      throw std::invalid_argument("invalid or duplicate tag id: " + entry.first.Scalar());
    }

    const std::string dock_id = required_field(entry.second, "dock_id", tag_id);
    const std::string dock_type = required_field(entry.second, "dock_type", tag_id);
    const std::string tag_frame = required_field(entry.second, "tag_frame", tag_id);
    if (!dock_ids.insert(dock_id).second) {
      throw std::invalid_argument("duplicate dock id: " + dock_id);
    }
    specs.emplace(tag_id, core::DockSpec{tag_id, dock_id, dock_type, tag_frame});
  }
  return specs;
}

}  // namespace demo2_apriltag_docking_cpp::adapters
