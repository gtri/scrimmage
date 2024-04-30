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

  double position_rmse = 0;

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

      EXPECT_NEAR(gpu_state.position().x(), cpu_state.position().x(), 1e-1);
      EXPECT_NEAR(gpu_state.position().y(), cpu_state.position().y(), 1e-1);
      EXPECT_NEAR(gpu_state.position().z(), cpu_state.position().z(), 1e-1);

      double x_diff = gpu_state.position().x() - cpu_state.position().x();
      double y_diff = gpu_state.position().y() - cpu_state.position().y();
      double z_diff = gpu_state.position().z() - cpu_state.position().z();

      position_rmse += pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2);

      
      EXPECT_NEAR(gpu_state.linear_velocity().x(), cpu_state.linear_velocity().x(), 1e-1);
      EXPECT_NEAR(gpu_state.linear_velocity().y(), cpu_state.linear_velocity().y(), 1e-1);
      EXPECT_NEAR(gpu_state.linear_velocity().z(), cpu_state.linear_velocity().z(), 1e-1);

      EXPECT_NEAR(gpu_state.angular_velocity().x(), cpu_state.angular_velocity().x(), 1e-1);
      EXPECT_NEAR(gpu_state.angular_velocity().y(), cpu_state.angular_velocity().y(), 1e-1);
      EXPECT_NEAR(gpu_state.angular_velocity().z(), cpu_state.angular_velocity().z(), 1e-1);

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

      EXPECT_NEAR(angle_diff(gpu_quat.roll(), cpu_quat.roll()), 0, 1e-1);
      EXPECT_NEAR(angle_diff(gpu_quat.pitch(), cpu_quat.pitch()), 0, 1e-1);
      EXPECT_NEAR(angle_diff(gpu_quat.yaw(), cpu_quat.yaw()), 0, 1e-1);
    }
  }

  position_rmse = sqrt(position_rmse / cpu_frames.size()); 
  std::cout << "Position RMSE: " << position_rmse << std::endl; 
} 
