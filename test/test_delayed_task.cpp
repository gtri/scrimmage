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
#include <scrimmage/common/DelayedTask.h>

namespace sc = scrimmage;

TEST(test_delayed_task, delayed_task) {
    const int num_repeats = 3;
    const double delay = 1;
    int value = 0;
    sc::DelayedTask delayed_task(delay, num_repeats);

    delayed_task.task = [&](double t) {
        value++;
        return value < 3;
    };

    EXPECT_FALSE(delayed_task.done());

    bool updated, task_success;
    std::tie(updated, task_success) = delayed_task.update(0);
    EXPECT_TRUE(updated);
    EXPECT_TRUE(task_success);
    EXPECT_FALSE(delayed_task.done());
    EXPECT_EQ(value, 1);

    std::tie(updated, task_success) = delayed_task.update(0.5);
    EXPECT_FALSE(updated);
    EXPECT_TRUE(task_success);
    EXPECT_FALSE(delayed_task.done());
    EXPECT_EQ(value, 1);

    std::tie(updated, task_success) = delayed_task.update(1.1);
    EXPECT_TRUE(updated);
    EXPECT_TRUE(task_success);
    EXPECT_EQ(value, 2);

    std::tie(updated, task_success) = delayed_task.update(2.2);
    EXPECT_TRUE(updated);
    EXPECT_FALSE(task_success);
    EXPECT_FALSE(delayed_task.done());
    EXPECT_EQ(value, 3);

    std::tie(updated, task_success) = delayed_task.update(3.3);
    EXPECT_TRUE(updated);
    EXPECT_FALSE(task_success);
    EXPECT_EQ(value, 4);
    EXPECT_TRUE(delayed_task.done());
}
