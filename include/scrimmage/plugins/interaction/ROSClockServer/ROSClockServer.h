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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_ROSCLOCKSERVER_ROSCLOCKSERVER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_ROSCLOCKSERVER_ROSCLOCKSERVER_H_

#include <scrimmage/entity/Entity.h>
#include <scrimmage/simcontrol/EntityInteraction.h>

#include <ros/ros.h>

#include <chrono>  // NOLINT
#include <list>
#include <map>
#include <memory>
#include <mutex>  // NOLINT
#include <string>
#include <thread>  // NOLINT

namespace scrimmage {
namespace interaction {

class ROSClockServer : public scrimmage::EntityInteraction {
 public:
    ROSClockServer();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t,
                                 double dt) override;
    void close(double t) override;

 protected:
    void publish_clock_msg(const double &t);

 private:
    void check_rosmaster();
    void check_rosmaster_loop();

    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher clock_pub_;

    bool node_initialized_ = false;
    bool prev_rosmaster_state_ = false;
    std::thread check_rosmaster_thread_;
    bool running_ = false;
    std::mutex pub_mutex_;

    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>>
        sim_start_time_;
};
}  // namespace interaction
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_ROSCLOCKSERVER_ROSCLOCKSERVER_H_
