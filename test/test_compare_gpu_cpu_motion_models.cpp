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

// Will want to run actual sims in test. This should just prep the mission parse 
// files
class TestMotionModels : public ::testing::TestWithParam<std::string> {
  public:
    void SetUp() override {
      std::string filename = scrimmage::expand_user(GetParam());
      if (!fs::exists(filename)) {
        scrimmage::FileSearch file_search;
        boost::optional<std::string> result = file_search.find_mission(filename, false);
        if (!result) {
          // The mission file wasn't found. Exit.
          throw std::invalid_argument{"Could not find mission file for " + filename};
        }
        // The mission file was found, save its path.
        filename = *result;
      }

      cpu_mission_path = filename;
      gpu_mission_path = cpu_mission_path;
      gpu_mission_path.replace_filename(
          cpu_mission_path.stem()
          .concat("_gpu_test"))
        .concat(cpu_mission_path.extension().string());

      std::ifstream cpu_mission_file;
      std::ofstream gpu_mission_file;

      cpu_mission_file.open(cpu_mission_path);
      gpu_mission_file.open(gpu_mission_path);

      std::stringstream ss;
      ss << cpu_mission_file.rdbuf();
      std::string cpu_mission_file_contents = ss.str();
      std::ostream_iterator<char>gpu_contents{gpu_mission_file};

      std::regex motion_model_tag_re{"[^<_\\/]*motion_model"};
      std::regex_replace(gpu_contents, 
          cpu_mission_file_contents.cbegin(),
          cpu_mission_file_contents.cend(),
          motion_model_tag_re, 
          "gpu_motion_model");

      cpu_mission_file.close();
      gpu_mission_file.close();
    }

    void TearDown() override {
      // Remove gpu file.
      std::remove(gpu_mission_path.c_str());
    }

  protected:
    fs::path cpu_mission_path, gpu_mission_path;
};

INSTANTIATE_TEST_SUITE_P(Missions, TestMotionModels, testing::Values("straight"));

TEST_P(TestMotionModels, CompareMotionModelsTrajectories) {
  using scrimmage::Log, scrimmage_proto::Frame, scrimmage_proto::Contact, scrimmage_proto::State;
  using FramePtr = std::shared_ptr<Frame>;
  using Frames = std::list<FramePtr>;
  Log cpu_log, gpu_log;

  // TODO: Make sure seeds are the same

  boost::optional<std::string> cpu_log_dir = scrimmage::run_test(cpu_mission_path.string());
  auto gpu_log_dir = scrimmage::run_test(gpu_mission_path.string());

  ASSERT_TRUE(cpu_log_dir && gpu_log_dir);

  cpu_log.parse(*cpu_log_dir + "/frames.bin", Log::FileType::FRAMES);
  gpu_log.parse(*gpu_log_dir + "/frames.bin", Log::FileType::FRAMES);

  Frames &cpu_frames = cpu_log.frames();
  Frames &gpu_frames = gpu_log.frames();

  ASSERT_EQ(gpu_frames.size(), cpu_frames.size()); 


  auto cpu_frame_it = cpu_frames.begin();
  auto gpu_frame_it = gpu_frames.begin();

  scrimmage::CSV csv;
  std::set<std::string> headers; 
  headers.insert("time");
  for(; cpu_frame_it != cpu_frames.end() && gpu_frame_it != gpu_frames.end(); ++cpu_frame_it, ++gpu_frame_it) { 
    FramePtr cpu_frame = *cpu_frame_it;
    FramePtr gpu_frame = *gpu_frame_it;

    EXPECT_DOUBLE_EQ(gpu_frame->time(), cpu_frame->time());    
    EXPECT_EQ(gpu_frame->contact_size(), cpu_frame->contact_size());
    std::list<std::pair<std::string, double>> pairs;
    pairs.emplace_back("time", gpu_frame->time());
    for(int i = 0; i < gpu_frame->contact_size(); ++i) {
      const Contact& cpu_contact = cpu_frame->contact(i);
      const Contact& gpu_contact = gpu_frame->contact(i);

      EXPECT_EQ(gpu_contact.type(), cpu_contact.type());
      EXPECT_EQ(gpu_contact.active(), cpu_contact.active());

      const State& cpu_state = cpu_contact.state();
      const State& gpu_state = gpu_contact.state();

      EXPECT_NEAR(gpu_state.position().x(), cpu_state.position().x(), 1e-5);
      EXPECT_NEAR(gpu_state.position().y(), cpu_state.position().y(), 1e-5);
      EXPECT_NEAR(gpu_state.position().z(), cpu_state.position().z(), 1e-5);
      

      std::string id_str = std::to_string(gpu_contact.id().id());
      std::list<std::string> dirs{"x", "y", "z"};
      auto pick_dir_value = [&](std::string& dir, scrimmage_proto::Vector3d pos) {
        if (dir == "x") {
          return pos.x(); 
        } else if (dir == "y") {
          return pos.y(); 
        } else if (dir == "z") {
          return pos.z(); 
        }
        return 0.;
      };


      for(std::string dir : dirs) {
        std::string gpu_header = id_str + "_gpu_" + dir; 
        std::string cpu_header = id_str + "_cpu_" + dir; 
        if (headers.count(gpu_header) == 0) {
          headers.insert(gpu_header);
        }
        if (headers.count(cpu_header) == 0) {
          headers.insert(cpu_header);
        }
        pairs.emplace_back(gpu_header, pick_dir_value(dir, gpu_state.position()));
        pairs.emplace_back(cpu_header, pick_dir_value(dir, cpu_state.position()));
      }


      EXPECT_NEAR(gpu_state.linear_velocity().x(), cpu_state.linear_velocity().x(), 1e-5);
      EXPECT_NEAR(gpu_state.linear_velocity().y(), cpu_state.linear_velocity().y(), 1e-5);
      EXPECT_NEAR(gpu_state.linear_velocity().z(), cpu_state.linear_velocity().z(), 1e-5);

      EXPECT_NEAR(gpu_state.angular_velocity().x(), cpu_state.angular_velocity().x(), 1e-5);
      EXPECT_NEAR(gpu_state.angular_velocity().y(), cpu_state.angular_velocity().y(), 1e-5);
      EXPECT_NEAR(gpu_state.angular_velocity().z(), cpu_state.angular_velocity().z(), 1e-5);

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

      EXPECT_NEAR(gpu_quat.roll(), cpu_quat.roll(), 1e-5);
      EXPECT_NEAR(gpu_quat.pitch(), cpu_quat.pitch(), 1e-5);
      EXPECT_NEAR(gpu_quat.yaw(), cpu_quat.yaw(), 1e-5);
    }
    std::list<std::string> list_headers;
    list_headers.insert(list_headers.end(), headers.begin(), headers.end());
    csv.set_column_headers(list_headers, false);
    csv.append(pairs, false, true);
  }
  csv.to_csv(GetParam() + "_gpu_vs_cpu.csv");
}
