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

#include <scrimmage/common/Time.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/interaction/ROSClockServer/ROSClockServer.h>

#include <rosgraph_msgs/Clock.h>

#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::ROSClockServer,
                ROSClockServer_plugin)

namespace scrimmage {
namespace interaction {

ROSClockServer::ROSClockServer() {}

bool ROSClockServer::init(std::map<std::string, std::string> &mission_params,
                          std::map<std::string, std::string> &plugin_params) {
    // Store the current UNIX time at startup. We will add the simulation time
    // (which starts at 0 seconds), to this start time when publishing to the
    // ROS clock server.
    sim_start_time_ = std::chrono::system_clock::now();

    // initialize ros
    if (!ros::isInitialized()) {
        int argc = 0;
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }

    // run once, to preserve original synchronous behavior
    check_rosmaster();

    // rosmaster check thread
    running_ = true;
    check_rosmaster_thread_ = std::thread(&ROSClockServer::check_rosmaster_loop, this);

    return true;
}

void ROSClockServer::close(double t) {
    (void)t;
    running_ = false;
    check_rosmaster_thread_.join();
}

void ROSClockServer::check_rosmaster() {
    bool rosmaster_state = ros::master::check();
    if (prev_rosmaster_state_ == false && rosmaster_state == true) {
        ros::param::set("/use_sim_time", true);
        // time to reinitialize
        nh_ = std::make_shared<ros::NodeHandle>();  // this actually starts the node
        // Create Publisher
        pub_mutex_.lock();
        clock_pub_.shutdown();
        clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 1);
        pub_mutex_.unlock();
    }
    prev_rosmaster_state_ = rosmaster_state;
}

void ROSClockServer::check_rosmaster_loop() {
    while (running_) {
        check_rosmaster();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void ROSClockServer::publish_clock_msg(const double &t) {
    // Add the current simulation time to the system time at the start of the
    // simulation.
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>>
        sim_time_point = sim_start_time_ + std::chrono::duration<double>(t);

    // Convert the time_point into a duration that represents double time since
    // the epoch.
    std::chrono::duration<double> time_since_epoch = sim_time_point.time_since_epoch();
    double sim_time = time_since_epoch.count();

    // Extract the seconds and nanoseconds to construct the ros::Time message
    uint32_t sec = std::floor(sim_time);
    uint32_t nsec = (sim_time - static_cast<double>(sec)) * 1e9;

    // Create the ros Clock message and publish it
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time(sec, nsec);

    pub_mutex_.lock();
    clock_pub_.publish(clock_msg);
    pub_mutex_.unlock();
}

bool ROSClockServer::step_entity_interaction(std::list<sc::EntityPtr> &ents, double t, double dt) {
    publish_clock_msg(time_->t());
    return true;
}
}  // namespace interaction
}  // namespace scrimmage
