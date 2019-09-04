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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_NETWORK_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_NETWORK_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/common/CSV.h>

#include <map>
#include <list>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

namespace scrimmage {

class Network : public EntityPlugin {
 public:
    Network();

    virtual bool init(std::map<std::string, std::string> &/*mission_params*/,
                      std::map<std::string, std::string> &/*plugin_params*/);
    virtual bool step(std::map<std::string, std::list<NetworkDevicePtr>> &pubs,
                      std::map<std::string, std::list<NetworkDevicePtr>> &subs);
    std::string type() override;

    void set_rtree(const RTreePtr &rtree);

    void set_random(RandomPtr random);

    void close(double t) override;

    inline virtual void set_mission_parse(MissionParsePtr mp)
    { mp_ = mp; }

    // Get/set delay values
    double comm_delay() {return comm_delay_;}
    void set_comm_delay(double del) {comm_delay_ = del;}

 protected:
    RTreePtr rtree_;
    RandomPtr random_;
    MissionParsePtr mp_;

    // Delay handling
    double comm_delay_ = -1;
    bool is_stochastic_delay_ = false;

    // Key 1: Publisher Entity ID
    // Key 2: Subscriber Entity ID
    // Value : Whether the publisher can reach the subscriber with a message
    std::unordered_map<int, std::unordered_map<int, bool>> reachable_map_;

    virtual bool is_reachable(const scrimmage::EntityPluginPtr &pub_plugin,
                              const scrimmage::EntityPluginPtr &sub_plugin);

    virtual bool is_successful_transmission(const scrimmage::EntityPluginPtr &pub_plugin,
                                            const scrimmage::EntityPluginPtr &sub_plugin);

    virtual double get_transmission_delay();

    bool network_init(std::map<std::string, std::string> &/*mission_params*/,
                      std::map<std::string, std::string> &/*plugin_params*/);

 private:
    // Key: Topic String
    std::map<std::string, unsigned int> pub_counts_;
    std::map<std::string, unsigned int> sub_counts_;
    bool monitor_all_pubs_ = false;
    bool monitor_all_subs_ = false;

    // Logging utility
    bool write_csv_ = false;
    CSV csv_;
};

typedef std::shared_ptr<Network> NetworkPtr;
using NetworkMap = std::map<std::string, NetworkPtr>;
using NetworkMapPtr = std::shared_ptr<NetworkMap>;

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_PUBSUB_NETWORK_H_
