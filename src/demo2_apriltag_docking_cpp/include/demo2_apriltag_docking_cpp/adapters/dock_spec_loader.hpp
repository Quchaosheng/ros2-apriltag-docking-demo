#pragma once

#include "demo2_apriltag_docking_cpp/core/tag_policy.hpp"

#include <string>
#include <unordered_map>

namespace demo2_apriltag_docking_cpp::adapters {

std::unordered_map<int, core::DockSpec> load_dock_specs(const std::string & path);

}  // namespace demo2_apriltag_docking_cpp::adapters
