#include "demo2_apriltag_docking_cpp/adapters/dock_spec_loader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace adapters = demo2_apriltag_docking_cpp::adapters;

namespace {

class TemporaryYaml {
public:
  explicit TemporaryYaml(const std::string & content)
  : path_(std::filesystem::temp_directory_path() / "demo2_dock_specs_test.yaml")
  {
    std::ofstream stream(path_);
    stream << content;
  }

  ~TemporaryYaml()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::string path() const
  {
    return path_.string();
  }

private:
  std::filesystem::path path_;
};

}  // namespace

TEST(DockSpecLoader, LoadsValidMapping)
{
  TemporaryYaml yaml(
    "docks:\n"
    "  '0':\n"
    "    dock_id: demo_charge_dock\n"
    "    dock_type: charging_dock\n"
    "    tag_frame: tag36h11:0\n");
  const auto specs = adapters::load_dock_specs(yaml.path());
  ASSERT_EQ(specs.size(), 1U);
  EXPECT_EQ(specs.at(0).dock_id, "demo_charge_dock");
  EXPECT_EQ(specs.at(0).dock_type, "charging_dock");
  EXPECT_EQ(specs.at(0).tag_frame, "tag36h11:0");
}

TEST(DockSpecLoader, RejectsInvalidMappings)
{
  for (const std::string & content : {
      "docks: []\n",
      "docks:\n  -1: {dock_id: a, dock_type: b, tag_frame: c}\n",
      "docks:\n  0: {dock_id: '', dock_type: b, tag_frame: c}\n",
      "docks:\n  0: {dock_id: same, dock_type: a, tag_frame: tag:0}\n  1: {dock_id: same, dock_type: b, tag_frame: tag:1}\n",
      "docks:\n  0: {dock_id: a, dock_type: a, tag_frame: tag:0}\n  '0': {dock_id: b, dock_type: b, tag_frame: tag:1}\n",
    })
  {
    TemporaryYaml yaml(content);
    EXPECT_THROW(adapters::load_dock_specs(yaml.path()), std::exception) << content;
  }
}

