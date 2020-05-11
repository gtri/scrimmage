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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */


#include <gtest/gtest.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/Angles.h>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::Vector3d;
namespace sc = scrimmage;

TEST(test_quaternion, rotation) {
    Vector3d vector = Vector3d(1, 0, 0);
    sc::Quaternion quaternion(Vector3d(1, 1, 1), 2 * M_PI / 3);
    Vector3d rotated_vector = quaternion.rotate(vector);
    EXPECT_NEAR(rotated_vector(0), 0, 1e-10);
    EXPECT_NEAR(rotated_vector(1), 1, 1e-10);
    EXPECT_NEAR(rotated_vector(2), 0, 1e-10);
}

TEST(test_quaternion, reverse_rotation) {
    Vector3d vector = Vector3d(1, 0, 0);
    sc::Quaternion quaternion(Vector3d(1, 1, 1), -2 * M_PI / 3);
    Vector3d rotated_vector = quaternion.rotate_reverse(vector);
    EXPECT_NEAR(rotated_vector(0), 0, 1e-10);
    EXPECT_NEAR(rotated_vector(1), 1, 1e-10);
    EXPECT_NEAR(rotated_vector(2), 0, 1e-10);
}

TEST(test_quaternion, euler_convert) {
    double roll = 0.3;
    double pitch = 0.2;
    double yaw = 0.1;
    sc::Quaternion quaternion(roll, pitch, yaw);
    EXPECT_NEAR(roll, quaternion.roll(), 1e-10);
    EXPECT_NEAR(pitch, quaternion.pitch(), 1e-10);
    EXPECT_NEAR(yaw, quaternion.yaw(), 1e-10);
}

TEST(test_quaternion, frames) {
    Vector3d vec1(0, 0, 0);
    Vector3d vec2(1, 0, 1);
    Vector3d vec_diff = vec2 - vec1;

    // pointing upward with negative pitch
    sc::Quaternion q1(0, -sc::Angles::deg2rad(45), 0);

    Vector3d vec_diff_local1 = q1.rotate_reverse(vec_diff);
    EXPECT_NEAR(sqrt(2), vec_diff_local1(0), 1e-10);
    EXPECT_NEAR(0, vec_diff_local1(1), 1e-10);
    EXPECT_NEAR(0, vec_diff_local1(2), 1e-10);

    Vector3d vec_diff_global = q1.rotate(vec_diff_local1);
    EXPECT_NEAR(vec_diff(0), vec_diff_global(0), 1e-10);
    EXPECT_NEAR(vec_diff(1), vec_diff_global(1), 1e-10);
    EXPECT_NEAR(vec_diff(2), vec_diff_global(2), 1e-10);
}

TEST(test_quaternion, constructors) {
    // copy constructor
    scrimmage::Quaternion quat1(0, 0, M_PI / 2);
    scrimmage::Quaternion quat2(M_PI, 0, 0);
    scrimmage::Quaternion quat3 = quat1 * quat2;

    // three different ways of copying
    scrimmage::Quaternion quat4(quat1 * quat2);
    scrimmage::Quaternion quat5;
    quat5 = quat1 * quat2;

    // test values
    EXPECT_TRUE(std::abs(quat3.w()) < 1e-7);
    EXPECT_DOUBLE_EQ(quat3.vec()[0], std::sqrt(1./2.));
    EXPECT_DOUBLE_EQ(quat3.vec()[1], std::sqrt(1./2.));
    EXPECT_TRUE(std::abs(quat3.vec()[2]) < 1e-7);

    EXPECT_TRUE(std::abs(quat4.w()) < 1e-7);
    EXPECT_DOUBLE_EQ(quat4.vec()[0], std::sqrt(1./2.));
    EXPECT_DOUBLE_EQ(quat4.vec()[1], std::sqrt(1./2.));
    EXPECT_TRUE(std::abs(quat4.vec()[2]) < 1e-7);

    EXPECT_TRUE(std::abs(quat5.w()) < 1e-7);
    EXPECT_DOUBLE_EQ(quat5.vec()[0], std::sqrt(1./2.));
    EXPECT_DOUBLE_EQ(quat5.vec()[1], std::sqrt(1./2.));
    EXPECT_TRUE(std::abs(quat5.vec()[2]) < 1e-7);
}
