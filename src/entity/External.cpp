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
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/simcontrol/SimUtils.h>
#include <scrimmage/simcontrol/EntityInteraction.h>

#include <iostream>
#include <iomanip>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/copy.hpp>

using std::endl;
using std::cout;

namespace ba = boost::adaptors;
namespace br = boost::range;

namespace scrimmage {

External::External() :
    entity_(std::make_shared<Entity>()),
    plugin_manager_(std::make_shared<PluginManager>()),
    log_(std::make_shared<Log>()),
    last_t_(NAN),
    pubsub_(std::make_shared<PubSub>()),
    time_(std::make_shared<Time>()),
    mp_(std::make_shared<MissionParse>()),
    id_to_team_map_(std::make_shared<std::unordered_map<int, int>>()),
    id_to_ent_map_(std::make_shared<std::unordered_map<int, EntityPtr>>()) {}

bool External::create_entity(int max_entities, int entity_id,
                             const std::string &entity_name) {

    if (mp_->get_mission_filename() == "") {
        cout << "External::mp()->parse() has not been run yet, "
            << "exiting External::create_entity()" << endl;
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

    std::map<std::string, std::string> info =
        mp_->entity_descriptions()[it_name_id->second];
    info.erase("motion_model"); // we don't need to initialize this

    ContactMapPtr contacts = std::make_shared<ContactMap>();
    std::shared_ptr<RTree> rtree = std::make_shared<RTree>();
    rtree->init(max_entities);

    FileSearchPtr file_search = std::make_shared<FileSearch>();
    RandomPtr random = std::make_shared<Random>();
    random->seed();

    std::list<scrimmage_proto::ShapePtr> shapes;

    SimUtilsInfo sim_info;
    sim_info.mp = mp_;
    sim_info.plugin_manager = plugin_manager_;
    sim_info.file_search = file_search;
    sim_info.rtree = rtree;
    sim_info.pubsub = pubsub_;
    sim_info.time = time_;
    sim_info.random = random;
    sim_info.id_to_team_map = id_to_team_map_;
    sim_info.id_to_ent_map = id_to_ent_map_;

    networks_ = std::make_shared<NetworkMap>();
    if (!create_networks(sim_info, *networks_)) {
        std::cout << "External::create_entity() failed on create_networks()" << std::endl;
        return false;
    }

    metrics_.clear();
    if (!create_metrics(sim_info, contacts, metrics_)) {
        std::cout << "External::create_entity() failed on create_metrics()" << std::endl;
        return false;
    }

    ent_inters_.clear();
    if (!create_ent_inters(sim_info, contacts, shapes, ent_inters_)) {
        std::cout << "External::create_entity() failed on create_ent_inters()" << std::endl;
        return false;
    }

    entity_ = std::make_shared<Entity>();
    entity_->set_random(random);

    AttributeMap &attr_map = mp_->entity_attributes()[it_name_id->second];
    bool ent_success =
        entity_->init(
            attr_map, info, contacts, mp_, mp_->projection(), entity_id,
            it_name_id->second, plugin_manager_, file_search, rtree, pubsub_,
            time_);
    if (!ent_success) {
        std::cout << "External::create_entity() failed on entity_->init()" << std::endl;
        return false;
    }

    connect(entity_->controller()->vars(), vars);
    if (!verify_io_connection(entity_->controller()->vars(), vars)) {
        auto ctrl = entity_->controller();
        std::cout << "VariableIO Error: "
            << ctrl->name()
            << " does not provide inputs required by the External class."
            << std::endl;
        print_io_error("External", vars);
        std::cout << ctrl->name() << " currently provides the following: ";
        br::copy(ctrl->vars().output_variable_index() | ba::map_keys,
            std::ostream_iterator<std::string>(std::cout, ", "));
        return false;
    }

    return true;
}

void External::setup_logging() {
    std::string output_type = get("output_type", mp_->params(), std::string("all"));

    mp_->create_log_dir();
    bool enable_log =
        output_type.find("all") != std::string::npos ||
        output_type.find("frames") != std::string::npos;

    log_->set_enable_log(enable_log);
    log_->init(mp_->log_dir(), Log::WRITE);
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

    ////////////////////////////////////////////////////////////
    // run main loops of plugins
    ////////////////////////////////////////////////////////////
    for (AutonomyPtr autonomy : entity_->autonomies()) {
        autonomy->step_autonomy(t, dt);
    }

    entity_->setup_desired_state();

    int num_steps = dt <= min_motion_dt ? 1 : ceil(dt / min_motion_dt);
    double motion_dt = dt / num_steps;
    double temp_t = t - dt;
    for (int i = 0; i < num_steps; i++) {
        entity_->controller()->step(temp_t, motion_dt);
        temp_t += motion_dt;
    }

    if (!ent_inters_.empty()) {
        update_ents();
        for (EntityInteractionPtr ent_inter : ent_inters_) {
            ent_inter->step_entity_interaction(ents_, t, dt);
        }
    }

    for (MetricsPtr metrics : metrics_) {
        metrics->step_metrics(t, dt);
    }

    if (!send_messages()) {return false;}

    // shapes
    scrimmage_proto::Shapes shapes;
    shapes.set_time(t);
    auto add_shapes = [&](auto p) {
        auto add_shape = [&](auto &shape) {*shapes.add_shape() = *shape;};
        br::for_each(p->shapes(), add_shape);
    };

    br::for_each(entity_->autonomies(), add_shapes);
    add_shapes(entity_->controller());
    br::for_each(ent_inters_, add_shapes);
    br::for_each(metrics_, add_shapes);

    log_->save_frame(create_frame(t, entity_->contacts()));
    log_->save_shapes(shapes);

    mutex.unlock();
    return true;
}

EntityPtr &External::entity() {
    return entity_;
}

bool External::send_messages() {
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

    // any messages that are not linked to the external network can now
    // be sent via the internal scrimmage messaging system
    auto &pubs = pubsub_->pubs();
    auto &subs = pubsub_->subs();
    for (auto &network : *networks_ | ba::map_values) {
        auto name = network->name();
        if (!network->step(pubs[name], subs[name])) {
            std::cout << "Network failed: " << name << std::endl;
            return false;
        }
    }

    br::for_each(entity_->autonomies(), run_callbacks);
    run_callbacks(entity_->controller());
    br::for_each(ent_inters_, run_callbacks);
    br::for_each(metrics_, run_callbacks);
    br::for_each(entity_->sensors() | ba::map_values, run_callbacks);
    return true;
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
        update_ents();
    }
    mutex.unlock();
}

MissionParsePtr External::mp() {
    return mp_;
}

void External::update_ents() {
    if (ent_inters_.empty() && !metrics_.empty()) return;

    auto &contacts = *entity_->contacts();

    id_to_team_map_->clear();
    id_to_ent_map_->clear();
    ents_.clear();
    for (auto &kv : contacts) {
        ID &id = kv.second.id();
        (*id_to_team_map_)[id.id()] = id.team_id();

        auto ent = std::make_shared<Entity>();
        ent->id() = id;
        ent->state() = kv.second.state();
        ents_.push_back(ent);

        (*id_to_ent_map_)[id.id()] = ent;
    }
}

void External::close() {
    mutex.lock();
    entity_->close(time_->t());
    entity_ = std::make_shared<Entity>();

    auto close = [&](auto p) {p->close(time_->t());};
    br::for_each(ent_inters_, close);
    br::for_each(metrics_, close);
    mutex.unlock();
}

} // namespace scrimmage
