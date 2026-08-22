#include "demo2_apriltag_docking/tag_policy.hpp"

#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace demo2_apriltag_docking
{

std::unordered_map<int, DockSpec> load_dock_specs(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node docks = root["docks"];
  if (!docks || !docks.IsMap() || docks.size() == 0U) {
    throw std::invalid_argument("docks must be a non-empty mapping");
  }

  std::unordered_map<int, DockSpec> specs;
  std::unordered_map<std::string, int> dock_ids;
  for (const auto & item : docks) {
    int tag_id;
    try {
      tag_id = item.first.as<int>();
    } catch (const YAML::Exception & ex) {
      throw std::invalid_argument(std::string("invalid tag id: ") + ex.what());
    }
    if (tag_id < 0 || specs.find(tag_id) != specs.end() || !item.second.IsMap()) {
      throw std::invalid_argument("invalid or duplicate tag id");
    }

    const auto read_field = [&item](const char * name) {
        const YAML::Node value = item.second[name];
        if (!value || !value.IsScalar()) {
          throw std::invalid_argument(std::string("tag has missing field: ") + name);
        }
        const std::string text = value.as<std::string>();
        if (text.empty()) {
          throw std::invalid_argument(std::string("tag has empty field: ") + name);
        }
        return text;
      };

    DockSpec spec{
      tag_id,
      read_field("dock_id"),
      read_field("dock_type"),
      read_field("tag_frame")};
    if (dock_ids.find(spec.dock_id) != dock_ids.end()) {
      throw std::invalid_argument("duplicate dock id: " + spec.dock_id);
    }
    dock_ids.emplace(spec.dock_id, tag_id);
    specs.emplace(tag_id, std::move(spec));
  }
  return specs;
}

}  // namespace demo2_apriltag_docking
