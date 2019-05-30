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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */


#include <gtest/gtest.h>
#include <scrimmage/common/Pose.h>
#include <scrimmage/common/PoseStamped.h>
#include <scrimmage/common/Twist.h>
#include <scrimmage/common/TwistStamped.h>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>
#include <memory>

TEST(pose, constructor) {
    scrimmage::Pose pose1;
    EXPECT_FLOAT_EQ(pose1.pos()[0], 0);
    EXPECT_FLOAT_EQ(pose1.pos()[1], 0);
    EXPECT_FLOAT_EQ(pose1.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose1.quat().w(), 1);
    EXPECT_FLOAT_EQ(pose1.quat().vec()[0], 0);
    EXPECT_FLOAT_EQ(pose1.quat().vec()[1], 0);
    EXPECT_FLOAT_EQ(pose1.quat().vec()[2], 0);

    Eigen::Vector3d pos2(-2.3, 1, 0.00);
    scrimmage::Pose pose2(pos2);
    EXPECT_FLOAT_EQ(pose2.pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose2.pos()[1], 1);
    EXPECT_FLOAT_EQ(pose2.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose2.quat().w(), 1);
    EXPECT_FLOAT_EQ(pose2.quat().vec()[0], 0);
    EXPECT_FLOAT_EQ(pose2.quat().vec()[1], 0);
    EXPECT_FLOAT_EQ(pose2.quat().vec()[2], 0);

    scrimmage::Quaternion quat3(-.5, .1, 3.14);
    scrimmage::Pose pose3(quat3);
    EXPECT_FLOAT_EQ(pose3.pos()[0], 0);
    EXPECT_FLOAT_EQ(pose3.pos()[1], 0);
    EXPECT_FLOAT_EQ(pose3.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose3.quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose3.quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose3.quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose3.quat().vec()[2],  0.96771109);

    scrimmage::Pose pose4(pos2, quat3);
    EXPECT_FLOAT_EQ(pose4.pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose4.pos()[1], 1);
    EXPECT_FLOAT_EQ(pose4.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose4.quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose4.quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose4.quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose4.quat().vec()[2],  0.96771109);
}

TEST(pose, ostream) {
    scrimmage::Pose pose;
    std::stringstream ss;
    std::string ans = "Pose {\n  pos: 0, 0, 0\n  quat: w: 1, x: 0, y: 0, z: 0\n}";
    ss << pose;
    EXPECT_STREQ(ss.str().c_str(), ans.c_str());
}

TEST(pose_stamped, constructor) {
    scrimmage::PoseStamped pose1;
    EXPECT_FLOAT_EQ(pose1.pos()[0], 0);
    EXPECT_FLOAT_EQ(pose1.pos()[1], 0);
    EXPECT_FLOAT_EQ(pose1.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose1.quat().w(), 1);
    EXPECT_FLOAT_EQ(pose1.quat().vec()[0], 0);
    EXPECT_FLOAT_EQ(pose1.quat().vec()[1], 0);
    EXPECT_FLOAT_EQ(pose1.quat().vec()[2], 0);
    EXPECT_FLOAT_EQ(pose1.time(), 0);

    Eigen::Vector3d pos2(-2.3, 1, 0.00);
    scrimmage::PoseStamped pose2(pos2);
    EXPECT_FLOAT_EQ(pose2.pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose2.pos()[1], 1);
    EXPECT_FLOAT_EQ(pose2.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose2.quat().w(), 1);
    EXPECT_FLOAT_EQ(pose2.quat().vec()[0], 0);
    EXPECT_FLOAT_EQ(pose2.quat().vec()[1], 0);
    EXPECT_FLOAT_EQ(pose2.quat().vec()[2], 0);
    EXPECT_FLOAT_EQ(pose2.time(), 0);

    scrimmage::Quaternion quat3(-.5, .1, 3.14);
    scrimmage::PoseStamped pose3(quat3);
    EXPECT_FLOAT_EQ(pose3.pos()[0], 0);
    EXPECT_FLOAT_EQ(pose3.pos()[1], 0);
    EXPECT_FLOAT_EQ(pose3.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose3.quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose3.quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose3.quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose3.quat().vec()[2],  0.96771109);
    EXPECT_FLOAT_EQ(pose3.time(), 0);

    scrimmage::PoseStamped pose4(pos2, quat3);
    EXPECT_FLOAT_EQ(pose4.pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose4.pos()[1], 1);
    EXPECT_FLOAT_EQ(pose4.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose4.quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose4.quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose4.quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose4.quat().vec()[2],  0.96771109);
    EXPECT_FLOAT_EQ(pose4.time(), 0);
}

TEST(pose_stamped, const_access) {
    const Eigen::Vector3d pos(-2.3, 1, 0.00);
    const scrimmage::Quaternion quat(-.5, .1, 3.14);
    const scrimmage::PoseStamped pose(pos, quat);

    EXPECT_FLOAT_EQ(pose.pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose.pos()[1], 1);
    EXPECT_FLOAT_EQ(pose.pos()[2], 0);
    EXPECT_FLOAT_EQ(pose.quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose.quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose.quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose.quat().vec()[2],  0.96771109);
    EXPECT_FLOAT_EQ(pose.time(), 0);
}

TEST(pose_stamped, ostream) {
    scrimmage::PoseStamped pose;
    std::stringstream ss;
    std::string ans = "PoseStamped {\n  time: 0\n  pos: 0, 0, 0\n  quat: w: 1, x: 0, y: 0, z: 0\n}";
    ss << pose;
    EXPECT_STREQ(ss.str().c_str(), ans.c_str());
}

TEST(pose, polymorphism) {
    const Eigen::Vector3d pos(-2.3, 1, 0.00);
    const scrimmage::Quaternion quat(-.5, .1, 3.14);

    std::vector<scrimmage::Pose *> pose_vec(2);
    pose_vec[0] = new scrimmage::Pose(pos, quat);
    pose_vec[1] = new scrimmage::PoseStamped(pos, quat);

    // Pose
    EXPECT_FLOAT_EQ(pose_vec[0]->pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose_vec[0]->pos()[1], 1);
    EXPECT_FLOAT_EQ(pose_vec[0]->pos()[2], 0);
    EXPECT_FLOAT_EQ(pose_vec[0]->quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose_vec[0]->quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose_vec[0]->quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose_vec[0]->quat().vec()[2],  0.96771109);

    // PoseStamped
    EXPECT_FLOAT_EQ(pose_vec[1]->pos()[0], -2.3);
    EXPECT_FLOAT_EQ(pose_vec[1]->pos()[1], 1);
    EXPECT_FLOAT_EQ(pose_vec[1]->pos()[2], 0);
    EXPECT_FLOAT_EQ(pose_vec[1]->quat().w(), -0.011594434);
    EXPECT_FLOAT_EQ(pose_vec[1]->quat().vec()[0], -0.048622191);
    EXPECT_FLOAT_EQ(pose_vec[1]->quat().vec()[1], -0.24705613);
    EXPECT_FLOAT_EQ(pose_vec[1]->quat().vec()[2],  0.96771109);

    // free memory
    for (auto it = pose_vec.begin(); it != pose_vec.end(); ++it) {
        delete *it;
    }
}

TEST(twist, constructor) {
    scrimmage::Twist twist1;
    EXPECT_FLOAT_EQ(twist1.vel()[0], 0);
    EXPECT_FLOAT_EQ(twist1.vel()[1], 0);
    EXPECT_FLOAT_EQ(twist1.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist1.ang_vel()[0], 0);
    EXPECT_FLOAT_EQ(twist1.ang_vel()[1], 0);
    EXPECT_FLOAT_EQ(twist1.ang_vel()[2], 0);

    Eigen::Vector3d vel2(-2.3, 1, 0.00);
    scrimmage::Twist twist2(vel2);
    EXPECT_FLOAT_EQ(twist2.vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist2.vel()[1], 1);
    EXPECT_FLOAT_EQ(twist2.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist2.ang_vel()[0], 0);
    EXPECT_FLOAT_EQ(twist2.ang_vel()[1], 0);
    EXPECT_FLOAT_EQ(twist2.ang_vel()[2], 0);

    Eigen::Vector3d vel3(0, 0, 0);
    Eigen::Vector3d ang_vel3(-.5, .1, 3.14);
    scrimmage::Twist twist3(vel3, ang_vel3);
    EXPECT_FLOAT_EQ(twist3.vel()[0], 0);
    EXPECT_FLOAT_EQ(twist3.vel()[1], 0);
    EXPECT_FLOAT_EQ(twist3.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist3.ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist3.ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist3.ang_vel()[2], 3.14);

    scrimmage::Twist twist4(vel2, ang_vel3);
    EXPECT_FLOAT_EQ(twist4.vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist4.vel()[1], 1);
    EXPECT_FLOAT_EQ(twist4.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist4.ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist4.ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist4.ang_vel()[2], 3.14);
}


TEST(twist_stamped, constructor) {
    scrimmage::TwistStamped twist1;
    EXPECT_FLOAT_EQ(twist1.vel()[0], 0);
    EXPECT_FLOAT_EQ(twist1.vel()[1], 0);
    EXPECT_FLOAT_EQ(twist1.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist1.ang_vel()[0], 0);
    EXPECT_FLOAT_EQ(twist1.ang_vel()[1], 0);
    EXPECT_FLOAT_EQ(twist1.ang_vel()[2], 0);
    EXPECT_FLOAT_EQ(twist1.time(), 0);

    Eigen::Vector3d vel2(-2.3, 1, 0.00);
    scrimmage::TwistStamped twist2(vel2);
    EXPECT_FLOAT_EQ(twist2.vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist2.vel()[1], 1);
    EXPECT_FLOAT_EQ(twist2.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist2.ang_vel()[0], 0);
    EXPECT_FLOAT_EQ(twist2.ang_vel()[1], 0);
    EXPECT_FLOAT_EQ(twist2.ang_vel()[2], 0);
    EXPECT_FLOAT_EQ(twist2.time(), 0);

    Eigen::Vector3d vel3(0, 0, 0);
    Eigen::Vector3d ang_vel3(-.5, .1, 3.14);
    scrimmage::TwistStamped twist3(vel3, ang_vel3);
    EXPECT_FLOAT_EQ(twist3.vel()[0], 0);
    EXPECT_FLOAT_EQ(twist3.vel()[1], 0);
    EXPECT_FLOAT_EQ(twist3.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist3.ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist3.ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist3.ang_vel()[2], 3.14);
    EXPECT_FLOAT_EQ(twist3.time(), 0);

    scrimmage::TwistStamped twist4(vel2, ang_vel3);
    EXPECT_FLOAT_EQ(twist4.vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist4.vel()[1], 1);
    EXPECT_FLOAT_EQ(twist4.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist4.ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist4.ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist4.ang_vel()[2], 3.14);
    EXPECT_FLOAT_EQ(twist4.time(), 0);
}

TEST(twist, ostream) {
    scrimmage::Twist twist;
    std::stringstream ss;
    std::string ans = "Twist {\n  vel: 0, 0, 0\n  ang_vel: 0, 0, 0\n}";
    ss << twist;
    EXPECT_STREQ(ss.str().c_str(), ans.c_str());
}

TEST(twist_stamped, const_access) {
    const Eigen::Vector3d vel(-2.3, 1, 0.00);
    const Eigen::Vector3d ang_vel(-.5, .1, 3.14);
    const scrimmage::TwistStamped twist(vel, ang_vel);

    EXPECT_FLOAT_EQ(twist.vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist.vel()[1], 1);
    EXPECT_FLOAT_EQ(twist.vel()[2], 0);
    EXPECT_FLOAT_EQ(twist.ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist.ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist.ang_vel()[2], 3.14);
    EXPECT_FLOAT_EQ(twist.time(), 0);
}

TEST(twist_stamped, ostream) {
    scrimmage::TwistStamped twist;
    std::stringstream ss;
    std::string ans = "TwistStamped {\n  time: 0\n  vel: 0, 0, 0\n  ang_vel: 0, 0, 0\n}";
    ss << twist;
    EXPECT_STREQ(ss.str().c_str(), ans.c_str());
}

TEST(twist, polymorphism) {
    const Eigen::Vector3d vel(-2.3, 1, 0.00);
    const Eigen::Vector3d ang_vel(-.5, .1, 3.14);

    std::vector<scrimmage::Twist *> twist_vec(2);
    twist_vec[0] = new scrimmage::Twist(vel, ang_vel);
    twist_vec[1] = new scrimmage::TwistStamped(vel, ang_vel);

    // Twist
    EXPECT_FLOAT_EQ(twist_vec[0]->vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist_vec[0]->vel()[1], 1);
    EXPECT_FLOAT_EQ(twist_vec[0]->vel()[2], 0);
    EXPECT_FLOAT_EQ(twist_vec[0]->ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist_vec[0]->ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist_vec[0]->ang_vel()[2], 3.14);

    // TwistStamped
    EXPECT_FLOAT_EQ(twist_vec[1]->vel()[0], -2.3);
    EXPECT_FLOAT_EQ(twist_vec[1]->vel()[1], 1);
    EXPECT_FLOAT_EQ(twist_vec[1]->vel()[2], 0);
    EXPECT_FLOAT_EQ(twist_vec[1]->ang_vel()[0], -.5);
    EXPECT_FLOAT_EQ(twist_vec[1]->ang_vel()[1], .1);
    EXPECT_FLOAT_EQ(twist_vec[1]->ang_vel()[2], 3.14);

    // free memory
    for (auto it = twist_vec.begin(); it != twist_vec.end(); ++it) {
        delete *it;
    }
}
