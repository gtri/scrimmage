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

#ifndef INCLUDE_SCRIMMAGE_COMMON_TIMER_H_
#define INCLUDE_SCRIMMAGE_COMMON_TIMER_H_

#include <boost/date_time/posix_time/posix_time.hpp>

namespace scrimmage {

class Timer {
 public:
    void start_overall_timer();

    boost::posix_time::time_duration elapsed_time();

    void start_loop_timer();

    boost::posix_time::time_duration loop_wait();

    void set_iterate_rate(double iterate_rate);

    void set_time_warp(double time_warp);

    void update_time_config();

    static uint64_t getnanotime();

    void inc_warp();

    void dec_warp();

    double time_warp();

 protected:
    double time_warp_ = NAN;
    boost::posix_time::ptime start_time_;

    boost::posix_time::ptime actual_time_;
    boost::posix_time::time_duration actual_elapsed_time_;

    boost::posix_time::ptime sim_time_;
    boost::posix_time::time_duration sim_elapsed_time_;

    boost::posix_time::ptime loop_timer_;
    boost::posix_time::time_duration iterate_period_;
    double iterate_rate_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_TIMER_H_
