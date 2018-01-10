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
#ifndef INCLUDE_SCRIMMAGE_COMMON_DELAYEDTASK_H_
#define INCLUDE_SCRIMMAGE_COMMON_DELAYEDTASK_H_

#include <utility>
#include <functional>

namespace scrimmage {

/*! \brief repeats a task after a delay and some condition (if set) are met.
 * The task can be set to repeat a finite number of times.
 */
class DelayedTask {
 public:
    DelayedTask();
    DelayedTask(double _delay, int repeats);

    void set_delay_from_freq(double freq);
    bool done() const;

    void set_repeats(int repeats_left);
    void set_repeat_infinitely(bool repeat_infinitely);
    std::pair<bool, bool> update(double t);

    bool disable;
    double delay;
    double last_updated_time;
    double end_time;
    double eps;
    std::function<bool(double)> condition;
    std::function<bool(double)> task;

 protected:
    bool repeat_infinitely_;
    int repeats_left_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_DELAYEDTASK_H_
