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

#include <scrimmage/common/Timer.h>

#include <ctime>

#include <boost/thread.hpp>

namespace scrimmage {

void Timer::start_overall_timer() {
    start_time_ = boost::posix_time::microsec_clock::local_time();
    actual_time_ = start_time_;
    sim_time_ = start_time_;
    // Nat - same
    loop_end_time_ = loop_timer_ + iterate_period_;
    loop_timer_running_ = false;
    // end of same
}

boost::posix_time::time_duration Timer::elapsed_time() {
    return boost::posix_time::microsec_clock::local_time() - start_time_;
}

// Nat - one of the four functions used. Called at the beginning and while the simulation is playing
void Timer::start_loop_timer() {

    std::cout << "FUNCTION 1 - In start loop timer" << std::endl;

    loop_timer_ = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration time_diff = loop_timer_ - actual_time_;
    
    // Nat - same
    if (!loop_timer_running_) {
        // set loop to end on current time
        loop_end_time_ = loop_timer_;
        loop_timer_running_ = true;
    }
    loop_end_time_ += iterate_period_;
    // end of same

    actual_time_ = loop_timer_;

    sim_time_ += sim_time_period_; // Nat - same
}

// Nat - same
// Nat - one of the four functions used. This is not called while paused or playing. I believe
// this is only called during the scrimmage-playback simulations
void Timer::pause_loop_timer() {
    std::cout << "FUNCTION 2 - In pause loop timer" << std::endl;

    loop_timer_running_ = false;
}
// end of same

// Nat - one of the four functions used. This is called while the simulation is playing
boost::posix_time::time_duration Timer::loop_wait() {
    std::cout << "FUNCTION 3 - In start loop timer" << std::endl;

    boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();

    // Notes:
    // - loop_timer_ is: boost::posix_time::microsec_clock::local_time();
    // - time duration: boost::posix_time::time_duration time_diff = time - loop_timer_;
    // - remainder: boost::posix_time::time_duration remainder = iterate_period_ - time_diff;
    // - iterate period: iterate_period_ = boost::posix_time::time_duration(0, 0, 0, milli);

    // Nat - same
    boost::posix_time::time_duration time_diff = time - loop_timer_;
    if (time_diff < iterate_period_) {
    //if (time > loop_end_time_) {
        // already took too long, go to next period now.
        //return boost::posix_time::time_duration(0, 0, 0, 0);
        
        // The following lines did not fix it
        // std::cout << "Fix for threading" << std::endl;
        // boost::posix_time::time_duration remainder = loop_end_time_ - time;
        //boost::posix_time::time_duration time_diff = time - loop_timer_;
        boost::posix_time::time_duration remainder = iterate_period_ - time_diff;
        boost::this_thread::sleep(remainder);
        return remainder;
    }

    boost::posix_time::time_duration remainder = loop_end_time_ - time;
    boost::this_thread::sleep(remainder);
    // end of same

    return remainder;
}

void Timer::set_iterate_rate(double iterate_rate) {
    iterate_rate_ = iterate_rate;
}

void Timer::set_time_warp(double time_warp) {
    time_warp_ = time_warp;
}

// Nat - one of the four functions used. This is called once at the beginning of the simulation
void Timer::update_time_config() {
    std::cout << "FUNCTION 4 - In start loop timer" << std::endl;

    if (iterate_rate_ > 0 && time_warp_ > 0) {
        uint64_t milli = (1.0 / iterate_rate_ * 1000000.0) / time_warp_;
        iterate_period_ = boost::posix_time::time_duration(0, 0, 0, milli);
    } else {
        iterate_period_ = boost::posix_time::time_duration(0, 0, 0, 0);
    }

    // Nat - same
    sim_time_period_ = iterate_period_ * time_warp_;
    loop_timer_running_ = false;
    // end of same
}

uint64_t Timer::getnanotime() {
    uint64_t nano = 0;
    timespec ts;
    // clock_gettime(CLOCK_MONOTONIC, &ts); // Works on FreeBSD
    clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux

    nano = ts.tv_sec * 1e9 + ts.tv_nsec;
    return nano;
}

void Timer::inc_warp() {
    if (time_warp_ == 0) {
        time_warp_ = 1;
    } else if (time_warp_ < 2) {
        time_warp_ *= 2;
    } else {
        time_warp_ += 1.0;
    }
    update_time_config();
}

void Timer::dec_warp() {
    if (time_warp_ == 0) {
        time_warp_ = 1;
    } else if (time_warp_ < 2) {
        time_warp_ /= 2;
    } else {
        time_warp_ -= 1.0;
    }
    update_time_config();
}

double Timer::time_warp() {
    return time_warp_;
}

} // namespace scrimmage
