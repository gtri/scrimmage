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

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/External.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/pubsub/Network.h>

#include <iostream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>

using std::endl;
using std::cout;

namespace ba = boost::adaptors;

namespace scrimmage {

External::External() :
    entity_(std::make_shared<Entity>()),
    plugin_manager_(std::make_shared<PluginManager>()),
    log_(std::make_shared<Log>()),
    last_t_(NAN),
    pubsub_(std::make_shared<PubSub>()),
    time_(std::make_shared<Time>()),
    mp_(std::make_shared<MissionParse>()) {
}

bool External::create_entity(int max_entities, int entity_id,
                             const std::string &entity_name) {

    if (mp_->get_mission_filename() == "") {
        cout << "External::mp()->parse() has not been run yet, exiting External::create_entity()" << endl;
        return false;
    }

    // Parse the entity name and find the entity block ID for the associated
    // entity name
    auto it_name_id = mp_->entity_name_to_id().find(entity_name);
    if (it_name_id == mp_->entity_name_to_id().end()) {
        cout << "Entity name (" << entity_name << ") not found in mission file"
             << endl;
        return false;
    }

    // If there aren't any networks defined, define the GlobalNetwork by
    // default.
    if (mp_->network_names().size() == 0) {
        mp_->network_names().push_back("GlobalNetwork");
    }

    // Load the network names (don't need to load the network plugins)
    for (std::string network_name : mp_->network_names()) {
        // Seed the pubsub with network names
        pubsub_->add_network_name(network_name);
    }

    std::map<std::string, std::string> info =
        mp_->entity_descriptions()[it_name_id->second];

    ContactMapPtr contacts = std::make_shared<ContactMap>();
    std::shared_ptr<RTree> rtree = std::make_shared<RTree>();
    rtree->init(max_entities);

    FileSearchPtr file_search = std::make_shared<FileSearch>();
    entity_ = std::make_shared<Entity>();

    std::string output_type = get("output_type", mp_->params(), std::string("all"));

    mp_->create_log_dir();
    bool enable_log =
        output_type.find("all") != std::string::npos ||
        output_type.find("frames") != std::string::npos;

    log_->set_enable_log(enable_log);
    log_->init(mp_->log_dir(), Log::WRITE);

    RandomPtr random = std::make_shared<Random>();
    random->seed();
    entity_->set_random(random);

    AttributeMap &attr_map = mp_->entity_attributes()[it_name_id->second];
    bool success =
        entity_->init(
            attr_map, info, contacts, mp_, mp_->projection(), entity_id,
            it_name_id->second, plugin_manager_, file_search, rtree, pubsub_,
            time_);

    return success;
}

bool External::step(double t) {
    mutex.lock();
    double dt = std::isnan(last_t_) ? 0 : t - last_t_;
    time_->set_t(t);
    time_->set_dt(dt);
    last_t_ = t;
    mutex.unlock();

    this->call_update_contacts(t);

    mutex.lock();
    // do all the scrimmage updates (e.g., step_autonomy, step controller, etc)
    // incorporating motion_dt_
    for (AutonomyPtr autonomy : entity_->autonomies()) {
        autonomy->run_callbacks();
        autonomy->step_autonomy(t, dt);
    }

    entity_->setup_desired_state();

    int num_steps = dt <= min_motion_dt ? 1 : ceil(dt / min_motion_dt);
    double motion_dt = dt / num_steps;
    double temp_t = t - dt;
    for (int i = 0; i < num_steps; i++) {
        entity_->controller()->run_callbacks();
        entity_->controller()->step(temp_t, motion_dt);
        temp_t += motion_dt;
    }

    // do logging
    log_->save_frame(create_frame(t, entity_->contacts()));

    send_messages();

    // shapes
    scrimmage_proto::Shapes shapes;
    shapes.set_time(t);
    for (AutonomyPtr autonomy : entity_->autonomies()) {
        for (auto autonomy_shape : autonomy->shapes()) {
            // increase length of shapes by 1 (including mallocing a new object)
            // return a pointer to the malloced object
            scrimmage_proto::Shape *shape_at_end_of_shapes = shapes.add_shape();

            // copy autonomy shape to list
            *shape_at_end_of_shapes = *autonomy_shape;
        }
    }
    log_->save_shapes(shapes);

    mutex.unlock();
    return true;
}

EntityPtr &External::entity() {
    return entity_;
}

void External::send_messages() {
    // Send messages that are published by a SCRIMMAGE plugin to the external
    // network (e.g., ROS, MOOS)
    auto to_publisher = [&](auto &network_device) {return std::dynamic_pointer_cast<Publisher>(network_device);};
    auto has_callback = [&](auto &pub) {return pub && pub->callback;};

    for (auto &topic_device_kv : pubsub_->pubs() | ba::map_values) {
        for (auto &dev_list : topic_device_kv | ba::map_values) {
            for (auto pub : dev_list |
                            ba::transformed(to_publisher) |
                            ba::filtered(has_callback)) {
                for (auto msg : pub->pop_msgs()) {
                    pub->callback(msg);
                }
            }
        }
    }
}

void External::call_update_contacts(double t) {
    mutex.lock();
    if (update_contacts_task.update(t).first) {
        auto rtree = entity_->rtree();
        rtree->init(100);
        rtree->clear();
        for (auto &kv : *entity_->contacts()) {
            rtree->add(kv.second.state()->pos(), kv.second.id());
        }
    }
    mutex.unlock();
}

MissionParsePtr External::mp() {
    return mp_;
}
} // namespace scrimmage
