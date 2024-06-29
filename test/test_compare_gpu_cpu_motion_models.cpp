/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Wesley Ford <wford32@gatech.edu>
 * @date 23 April 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <cmath>
#include <ctime>
#include <gtest/gtest.h>

#include <scrimmage/log/Frame.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/simcontrol/SimUtils.h>
#include <scrimmage/proto/Frame.pb.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/common/CSV.h>

#include <CL/opencl.hpp>
#include <boost/optional.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>

namespace fs = std::filesystem;

struct CompareMissions {
  std::string cpu_filename;
  std::string gpu_filename;
};

class TestMotionModels : public ::testing::TestWithParam<CompareMissions> {
  public:
    void SetUp() override {
      auto find_test_mission = [&](const std::string& filename) -> std::optional<fs::path> {
        // Strip current srcfile stem to get src dir to search
        fs::path filepath{filename}; 
        if (!fs::exists(filepath) && filepath.extension() == ".xml") {
          // Look in the scrimmage mission directory for files 

          fs::path src_file{__FILE__};
          fs::path src_dir = src_file.parent_path();

          fs::recursive_directory_iterator dir_it{src_dir};
          for(auto& dir_ent : dir_it) {
            const fs::path& ent_path = dir_ent.path();
            bool is_xml_file = dir_ent.exists() 
              && dir_ent.is_regular_file() 
              && ent_path.extension() == ".xml";
            if (is_xml_file && ent_path.stem() == filepath.stem()) {
              return std::make_optional(ent_path);
            }
          }
        }
        return std::nullopt;
      };
        
      // First make sure the env is sourced
      ASSERT_NE(std::getenv("SCRIMMAGE_KERNEL_PATH"), nullptr);

      CompareMissions cm = GetParam();
      auto cpu_mission_path_opt = find_test_mission(cm.cpu_filename);
      auto gpu_mission_path_opt = find_test_mission(cm.gpu_filename);

      ASSERT_TRUE(cpu_mission_path_opt.has_value());
      ASSERT_TRUE(gpu_mission_path_opt.has_value());

      cpu_mission_path = cpu_mission_path_opt.value();
      gpu_mission_path = gpu_mission_path_opt.value();
    }

  protected:
    fs::path cpu_mission_path, gpu_mission_path;
};

// Relative to this source file
INSTANTIATE_TEST_SUITE_P(Missions, TestMotionModels, testing::Values(
      CompareMissions{"./test_missions/straight_cpu.xml", "./test_missions/straight_gpu.xml"},
      CompareMissions{"./test_missions/straight_cpu_threaded.xml", "./test_missions/straight_gpu_threaded.xml"},
      CompareMissions{"./test_missions/straight_cpu_mul.xml", "./test_missions/straight_gpu_mul.xml"}
      )
    );


TEST_P(TestMotionModels, CompareMotionModelsTrajectories) {
  using scrimmage::Log, scrimmage_proto::Frame, scrimmage_proto::Contact, scrimmage_proto::State;
  using FramePtr = std::shared_ptr<Frame>;
  using Frames = std::list<FramePtr>;
  Log cpu_log, gpu_log;
  constexpr double two_pi = 2*M_PI;

  auto clamp_angle = [](double rad) {
    // Makes sure angle is in [0, 2pi)
    while(rad < 0) {
      rad += two_pi;
    }
    while (rad >= two_pi) {
      rad -= two_pi;
    }
    return rad;
  }; 

  auto angle_diff = [&](double lhs, double rhs) {
    lhs = clamp_angle(lhs);
    rhs = clamp_angle(rhs);
    double cw_diff = lhs - rhs; 
    double ccw_diff = two_pi - cw_diff;
    return (abs(cw_diff) < abs(ccw_diff)) ? cw_diff : ccw_diff;
  };

  auto get_mission_seed = [&](const fs::path& mission_dir) -> unsigned long {
    const std::string seed_str{"Seed: "};
    fs::path log_txt = mission_dir / "log.txt";
    if(!fs::exists(log_txt)) { return -1;}
    std::ifstream log;
    log.open(log_txt);
    std::string line;
    for(; line.find(seed_str) == std::string::npos; std::getline(log, line));
    log.close();

    const std::size_t seed_start = line.find(seed_str) + seed_str.size();
    unsigned long seed = std::stoul(line.substr(seed_start));
    return seed;
  };

  // run_test is older, and produces a boost::optional
  boost::optional<std::string> cpu_log_dir = scrimmage::run_test(cpu_mission_path.string(), false, false);
  auto gpu_log_dir = scrimmage::run_test(gpu_mission_path.string(), false, false);

  ASSERT_TRUE(cpu_log_dir && gpu_log_dir);
  ASSERT_EQ(get_mission_seed(cpu_log_dir.value()), get_mission_seed(gpu_log_dir.value()));

  cpu_log.parse(*cpu_log_dir + "/frames.bin", Log::FileType::FRAMES);
  gpu_log.parse(*gpu_log_dir + "/frames.bin", Log::FileType::FRAMES);

  Frames &cpu_frames = cpu_log.frames();
  Frames &gpu_frames = gpu_log.frames();

  ASSERT_EQ(gpu_frames.size(), cpu_frames.size()); 

  auto cpu_frame_it = cpu_frames.begin();
  auto gpu_frame_it = gpu_frames.begin();

  for(; cpu_frame_it != cpu_frames.end() && gpu_frame_it != gpu_frames.end(); ++cpu_frame_it, ++gpu_frame_it) { 
    FramePtr cpu_frame = *cpu_frame_it;
    FramePtr gpu_frame = *gpu_frame_it;

    EXPECT_DOUBLE_EQ(gpu_frame->time(), cpu_frame->time());    
    EXPECT_EQ(gpu_frame->contact_size(), cpu_frame->contact_size());
    for(int i = 0; i < gpu_frame->contact_size(); ++i) {
      const Contact& cpu_contact = cpu_frame->contact(i);
      const Contact& gpu_contact = gpu_frame->contact(i);

      EXPECT_EQ(gpu_contact.type(), cpu_contact.type());
      EXPECT_EQ(gpu_contact.active(), cpu_contact.active());

      const State& cpu_state = cpu_contact.state();
      const State& gpu_state = gpu_contact.state();

      EXPECT_NEAR(gpu_state.position().x(), cpu_state.position().x(), 1e-8);
      EXPECT_NEAR(gpu_state.position().y(), cpu_state.position().y(), 1e-8);
      EXPECT_NEAR(gpu_state.position().z(), cpu_state.position().z(), 1e-8);

      EXPECT_NEAR(gpu_state.linear_velocity().x(), cpu_state.linear_velocity().x(), 1e-8);
      EXPECT_NEAR(gpu_state.linear_velocity().y(), cpu_state.linear_velocity().y(), 1e-8);
      EXPECT_NEAR(gpu_state.linear_velocity().z(), cpu_state.linear_velocity().z(), 1e-8);

      EXPECT_NEAR(gpu_state.angular_velocity().x(), cpu_state.angular_velocity().x(), 1e-8);
      EXPECT_NEAR(gpu_state.angular_velocity().y(), cpu_state.angular_velocity().y(), 1e-8);
      EXPECT_NEAR(gpu_state.angular_velocity().z(), cpu_state.angular_velocity().z(), 1e-8);

      scrimmage::Quaternion gpu_quat, cpu_quat;

      gpu_quat.set(
          gpu_state.orientation().w(),
          gpu_state.orientation().x(),
          gpu_state.orientation().y(), 
          gpu_state.orientation().z()); 

      cpu_quat.set(
          cpu_state.orientation().w(),
          cpu_state.orientation().x(),
          cpu_state.orientation().y(), 
          cpu_state.orientation().z()); 

      EXPECT_NEAR(angle_diff(gpu_quat.roll(), cpu_quat.roll()), 0, 1e-8);
      EXPECT_NEAR(angle_diff(gpu_quat.pitch(), cpu_quat.pitch()), 0, 1e-8);
      EXPECT_NEAR(angle_diff(gpu_quat.yaw(), cpu_quat.yaw()), 0, 1e-8);
    }
  }
} 
