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

#include <scrimmage/common/DelayedTask.h>

#include <limits>

namespace scrimmage {

DelayedTask::DelayedTask() : DelayedTask(0, -1) {}

DelayedTask::DelayedTask(double _delay, int repeats) :
        disable(false),
        delay(_delay),
        last_updated_time(-std::numeric_limits<double>::infinity()),
        end_time(std::numeric_limits<double>::infinity()),
        eps(0.0),
        condition(nullptr),
        task(nullptr),
        repeat_infinitely_(true),
        repeats_left_(-1) {
    set_repeats(repeats);
}

void DelayedTask::set_delay_from_freq(double freq) {
    delay = freq > 0 ? 1 / freq : 0;
}

std::pair<bool, bool> DelayedTask::update(double t) {
    bool updated = false, task_success = true;
    const bool condition_satisfied = condition ? condition(t) : true;
    const bool time_good = t >= last_updated_time + delay - eps && t <= end_time + eps;

    if (!disable && !done() && condition_satisfied && time_good) {
        last_updated_time = t;
        if (task) {
            task_success = task(t);
        }
        if (!repeat_infinitely_) {
            repeats_left_--;
        }

        updated = true;
    }
    return std::make_pair(updated, task_success);
}

bool DelayedTask::done() const {
    return !repeat_infinitely_ && repeats_left_ < 0;
}

void DelayedTask::set_repeats(int repeats_left) {
    repeats_left_ = repeats_left;
    repeat_infinitely_ = repeats_left_ < 0;
}

void DelayedTask::set_repeat_infinitely(bool repeat_infinitely) {
    repeat_infinitely_ = repeat_infinitely;
    repeats_left_ = -1;
}

} // namespace scrimmage
