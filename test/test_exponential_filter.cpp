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
#include <scrimmage/common/ExponentialFilter.h>

#include <cmath>

namespace sc = scrimmage;

TEST(test_quaternion, rotation) {
    const std::vector<double> tau_vec {0.1, 1, 10};
    const double dt = 0.001;

    for (double tau : tau_vec) {
        const int num_steps = 5 * tau / dt;

        sc::ExponentialFilter filt(tau);
        filt.add_estimate(0, 0);

        for (int i = 1; i < num_steps; i++) {
            double t = i * dt;

            // this should be ignored since it is in the past
            filt.add_estimate(50, -1);
            double est = filt.add_estimate(1, t);
            EXPECT_NEAR(est, 1 - exp(-t / tau), 0.001);
        }
    }
}
