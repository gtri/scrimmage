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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 29 Feb 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */


#include <gtest/gtest.h>

#include <scrimmage/parse/MissionParse.h>

class MissionParseTest : public testing::Test {
    protected:
        void SetUp() override {
            mp.parse(this->mission_file);
        }
        const std::string mission_file = "test_mission_parse_mission.xml";
        scrimmage::MissionParse mp;
};

// Values should match the ones in test_mission_parse_mission.xml
TEST_F(MissionParseTest, mission_parse_test_runtag_info) {
    // Test that the runtag info matches
    EXPECT_DOUBLE_EQ(mp.t0(), 0);
    EXPECT_DOUBLE_EQ(mp.tend(), 100);
    EXPECT_DOUBLE_EQ(mp.dt(), 0.1);
    EXPECT_DOUBLE_EQ(mp.time_warp(), 10);
    EXPECT_DOUBLE_EQ(mp.motion_multiplier(), 1);
    EXPECT_EQ(mp.enable_gui(), true);
    EXPECT_EQ(mp.network_gui(), false);
    EXPECT_EQ(mp.start_paused(), true);
    EXPECT_EQ(mp.full_screen(), true);
    EXPECT_EQ(mp.window_width(), 800);
    EXPECT_EQ(mp.window_height(), 600);
}
