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

#include <scrimmage/common/Algorithm.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/sensor/Sensable.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/autonomy/Autonomy.h>

#include <scrimmage/math/State.h>

#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Visual.pb.h>
#include <scrimmage/proto/Frame.pb.h>

#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Message.h>

#include <scrimmage/msgs/Event.pb.h>

#include <iostream>
#include <string>
#include <memory>
#include <future> // NOLINT

#include <GeographicLib/LocalCartesian.hpp>
#include <boost/thread.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sm = scrimmage_msgs;

using std::cout;
using std::endl;

namespace scrimmage {

SimControl::SimControl() :
        id_to_team_map_(new std::unordered_map<int, int>()),
        id_to_ent_map_(new std::unordered_map<int, EntityPtr>()),
        time_(std::make_shared<Time>()),
        timer_(Timer()),
        random_(new Random()),
        plugin_manager_(new PluginManager()),
        file_search_(std::make_shared<FileSearch>()) {
        networks_(new std::map<std::string, NetworkPtr>()),
        pubsub_(std::make_shared<PubSub>()),
        sim_plugin_(std::make_shared<Plugin>()) {

    pause(false);
    prev_paused_ = false;
    single_step(false);

    contacts_mutex_.lock();
    contacts_ = std::make_shared<ContactMap>();
    contacts_mutex_.unlock();
}

bool SimControl::init() {
    if (mp_ == NULL) {
        cout << "Mission Parse hasn't been set yet." << endl;
        return false;
    }

    proj_ = mp_->projection(); // get projection (origin) from mission

    if (get("show_plugins", mp_->params(), false)) {
        plugin_manager_->print_plugins("scrimmage::Autonomy", "Autonomy Plugins", *file_search_);
        plugin_manager_->print_plugins("scrimmage::MotionModel", "Motion Plugins", *file_search_);
        plugin_manager_->print_plugins("scrimmage::Controller", "Controller Plugins", *file_search_);
        plugin_manager_->print_plugins("scrimmage::EntityInteraction", "Entity Interaction Plugins", *file_search_);
        plugin_manager_->print_plugins("scrimmage::Sensor", "Sensor Plugins", *file_search_);
        plugin_manager_->print_plugins("scrimmage::Sensable", "Sensable Plugins", *file_search_);
        plugin_manager_->print_plugins("scrimmage::Metrics", "Metrics Plugins", *file_search_);
    }

#if ENABLE_JSBSIM == 1
    jsbsim_root_ = "./";
    if (const char* env_p = std::getenv("JSBSIM_ROOT")) {
        jsbsim_root_ = std::string(env_p);
    } else {
        cout << "Missing JSBSIM_ROOT env variable, using ./" << endl;
    }
#endif

    t0_ = mp_->t0();
    tend_ = mp_->tend();
    dt_ = mp_->dt();

    time_->set_t(t0_);
    time_->set_dt(dt_);
    time_->set_time_warp(mp_->time_warp());

    // Setup random seed
    if (mp_->params().count("seed") > 0) {
        random_->seed(std::stoul(mp_->params()["seed"]));
    } else {
        random_->seed();
    }
    log_->write_ascii("Seed: " + std::to_string(random_->get_seed()));

    int max_num_entities = 0;
    for (auto &kv : mp_->gen_info()) {
        max_num_entities += kv.second.total_count;
    }

    rtree_ = std::make_shared<scrimmage::RTree>();
    rtree_->init(max_num_entities);

    // What is the end condition?
    if (mp_->params().count("end_condition") > 0) {
        std::string cond = mp_->params()["end_condition"];

        auto add_cond = [&](auto nm, auto flag) {
            if (cond.find(nm) != std::string::npos) {
                end_conditions_.insert(flag);
            }
        };

        add_cond("time", EndConditionFlags::TIME);
        add_cond("one_team", EndConditionFlags::ONE_TEAM);
        add_cond("none", EndConditionFlags::NONE);
        add_cond("all_dead", EndConditionFlags::ALL_DEAD);

    } else {
        end_conditions_.insert(EndConditionFlags::TIME);
    }

    // Start with the simulation paused?
    if (mp_->start_paused()) {
        pause(true);
    }

    setup_timer(1.0 / dt_, mp_->time_warp());

    if (mp_->params().count("stream_port") > 0 &&
        mp_->params().count("stream_ip") > 0) {

        if (mp_->network_gui()) {
            outgoing_interface_->init_network(Interface::client,
                                              mp_->params()["stream_ip"],
                                              std::stoi(mp_->params()["stream_port"]));

            network_thread_ = std::thread(&Interface::init_network, &(*incoming_interface_),
                                          Interface::server,
                                          "localhost",
                                          std::stoi(mp_->params()["stream_port"])+1);
            network_thread_.detach();
        } else {
            outgoing_interface_->set_mode(Interface::shared);
            incoming_interface_->set_mode(Interface::shared);
        }

    } else {
        outgoing_interface_->set_mode(Interface::shared);
        incoming_interface_->set_mode(Interface::shared);
    }

    // Send initial gui information through GUI interface
    mp_->utm_terrain()->set_time(this->t());
    outgoing_interface_->send_utm_terrain(mp_->utm_terrain());

    // Load the appropriate network plugins
    for (std::string network_name : mp_->network_names()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            mp_->attributes()[network_name];
        NetworkPtr network =
            std::dynamic_pointer_cast<Network>(
                plugin_manager_->make_plugin("scrimmage::Network",
                                             network_name, file_search_,
                                             config_parse, overrides));

        // If the name was overridden, use the override.
        std::string name = get<std::string>("name", config_parse.params(),
                                            network_name);
        network->set_name(name);
        network->set_time(time_);
        network->set_pubsub(pubsub_);
        network->set_random(random_);
        network->set_rtree(rtree_);

        if (network == nullptr) {
            cout << "Failed to load network plugin: " << network_name << endl;
            continue;
        }

        network->init(mp_->params(), config_parse.params());
        (*networks_)[network_name] = network;
    }

    // Seed the PubSub object with the possible network names
    for (auto &kv : *networks_) {
        pubsub_->add_network_name(kv.second->name());
    }

    // Create base shape objects
    for (auto &kv : mp_->team_info()) {
        int i = 0;
        for (Eigen::Vector3d &base_pos : kv.second.bases) {
            ShapePtr base = std::make_shared<scrimmage_proto::Shape>();
            base->set_type(scrimmage_proto::Shape::Sphere);
            base->set_opacity(kv.second.opacities[i]);
            sc::set(base->mutable_center(), base_pos);
            sc::set(base->mutable_color(), kv.second.color);
            base->set_radius(kv.second.radii[i]);
            base->set_persistent(true);
            shapes_[0].push_back(base);
            i++;
        }
    }

    // Setup simcontrol's pubsub plugin
    sim_plugin_->set_pubsub(pubsub_);
    pub_end_time_ = sim_plugin_->advertise("GlobalNetwork", "EndTime", 1);
    pub_ent_gen_ = sim_plugin_->advertise("GlobalNetwork", "EntityGenerated", 1);
    pub_ent_rm_ = sim_plugin_->advertise("GlobalNetwork", "EntityRemoved", 1);
    pub_ent_pres_end_ = sim_plugin_->advertise("GlobalNetwork", "EntityPresentAtEnd", 1);
    pub_ent_int_exit_ = sim_plugin_->advertise("GlobalNetwork", "EntityInteractionExit", 1);
    pub_no_teams_ = sim_plugin_->advertise("GlobalNetwork", "NoTeamsPresent", 1);
    pub_one_team_ = sim_plugin_->advertise("GlobalNetwork", "OneTeamPresent", 1);

    // Get the list of "metrics" plugins
    for (std::string metrics_name : mp_->metrics()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            mp_->attributes()[metrics_name];
        MetricsPtr metrics =
            std::dynamic_pointer_cast<Metrics>(
                plugin_manager_->make_plugin(
                    "scrimmage::Metrics", metrics_name,
                    *file_search_, config_parse, overrides));

        if (metrics != nullptr) {
            metrics->set_id_to_team_map(id_to_team_map_);
            metrics->set_id_to_ent_map(id_to_ent_map_);
            metrics->set_pubsub(pubsub_);
            metrics->set_time(time_);
            metrics->init(config_parse.params());
            metrics_.push_back(metrics);
        } else {
            cout << "Failed to load metrics: " << metrics_name << endl;
            continue;
        }
    }

    // Get the list of "entity_interaction" plugins
    for (std::string ent_inter_name : mp_->entity_interactions()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            mp_->attributes()[ent_inter_name];
        EntityInteractionPtr ent_inter =
            std::dynamic_pointer_cast<EntityInteraction>(
                plugin_manager_->make_plugin("scrimmage::EntityInteraction",
                                             ent_inter_name, *file_search_,
                                             config_parse, overrides));

        if (ent_inter == nullptr) {
            cout << "Failed to load entity interaction plugin: "
                 << ent_inter_name << endl;
            continue;
        }

        ent_inter->set_random(random_);
        ent_inter->set_mission_parse(mp_);
        ent_inter->set_projection(proj_);
        ent_inter->set_pubsub(pubsub_);
        ent_inter->set_time(time_);
        ent_inter->set_id_to_team_map(id_to_team_map_);
        ent_inter->set_id_to_ent_map(id_to_ent_map_);
        ent_inter->init(mp_->params(), config_parse.params());

        // Get shapes from plugin
        shapes_[0].insert(shapes_[0].end(), ent_inter->shapes().begin(), ent_inter->shapes().end());
        ent_inter->shapes().clear();

        ent_inters_.push_back(ent_inter);
    }

    contacts_mutex_.lock();
    contacts_->reserve(max_num_entities+1);
    contacts_mutex_.unlock();

    if (get("show_plugins", mp_->params(), false)) {
        plugin_manager_->print_returned_plugins();
    }

    use_entity_threads_ = get("multi_threaded", mp_->params(), false);
    if (use_entity_threads_) {
        entity_pool_stop_ = false;
        num_entity_threads_ = get("num_threads", mp_->attributes()["multi_threaded"], 1);
        entity_worker_threads_.clear();
        entity_worker_threads_.reserve(num_entity_threads_);
        for (int i = 0; i < num_entity_threads_; i++) {
            entity_worker_threads_.push_back(std::thread(&SimControl::worker, this));
        }
    }

    run_send_shapes(); // draw any intial shapes

    // screenshots
    if (enable_gui() && get<bool>("enable_screenshots", mp_->params(), false) && mp_) {
        auto it = mp_->attributes().find("enable_screenshots");
        std::map<std::string, std::string> attr;
        if (it != mp_->attributes().end()) {
            attr = it->second;
        }

        screenshot_task_.disable = false;

        screenshot_task_.delay = get<double>("min_period", attr, 0.0);
        screenshot_task_.set_repeat_infinitely(true);
        screenshot_task_.last_updated_time =
            get<double>("start", attr, 0.0) - screenshot_task_.delay;
        screenshot_task_.end_time =
            get<double>("end", attr, std::numeric_limits<double>::infinity());
        screenshot_task_.eps = dt_ / 4;

        if (screenshot_task_.update(t_).first) {
            request_screenshot();
        }
    } else {
        screenshot_task_.disable = true;
    }
    prev_paused_ = paused();

    // reseeding
    auto it = mp_->attributes().find("seed");
    if (it != mp_->attributes().end()) {
        auto it2 = it->second.find("reseed_time");
        if (it2 != it->second.end()) {
            auto it3 = it->second.find("reseed");
            uint32_t reseed = 0;
            bool has_reseed = it3 != it->second.end();
            if (has_reseed) {
                reseed = std::stoul(it3->second);
            }
            reseed_task_ = DelayedTask(std::stod(it2->second), 0);
            reseed_task_.last_updated_time = 0;
            reseed_task_.task = [&, has_reseed, reseed](double t) {
                if (has_reseed) {
                    random_->seed(reseed);
                } else {
                    random_->seed();
                }
                log_->write_ascii("ReSeed: " + std::to_string(random_->get_seed()));
                return true;
            };
        }
    }
    return true;
}

void SimControl::request_screenshot() {
    prev_paused_ = paused();
    pause(true);
    scrimmage_proto::GUIMsg gui_msg;
    gui_msg.set_time(t_);
    gui_msg.set_single_step(true);
    outgoing_interface_->push_gui_msg(gui_msg);
}

bool SimControl::generate_entities(double t) {
    // Initialize each entity
    using NormDist = std::normal_distribution<double>;
    auto gener = random_->gener();

    for (auto &kv : mp_->entity_descriptions()) {
        int ent_desc_id = kv.first;
        std::map<std::string, std::string> params = kv.second;

        // Determine if we have to generate entities for this entity
        // description
        if (mp_->gen_info().count(ent_desc_id) == 0) {
            continue;
        }

        // Generate entities if time has been reached
        int gen_count = 0;
        for (double &gen_time : mp_->next_gen_times()[ent_desc_id]) {

            GenerateInfo &gen_info = mp_->gen_info()[ent_desc_id];
            if (t < gen_time || gen_info.total_count <= 0) {
                continue;
            }

#if ENABLE_JSBSIM == 1
            params["JSBSIM_ROOT"] = jsbsim_root_;
#endif
            params["dt"] = std::to_string(dt_);
            params["motion_multiplier"] = std::to_string(mp_->motion_multiplier());

            double x0 = scrimmage::get("x0", params, 0.0);
            double y0 = scrimmage::get("y0", params, 0.0);
            double z0 = scrimmage::get("z0", params, 0.0);
            double heading = scrimmage::get("heading", params, 0.0);

            Eigen::Vector3d pos(x0, y0, z0);

            NormDist x_normal_dist(x0, pow(get("variance_x", params, 100.0), 0.5));
            NormDist y_normal_dist(y0, pow(get("variance_y", params, 100.0), 0.5));
            NormDist z_normal_dist(z0, pow(get("variance_z", params, 0.0), 0.5));
            NormDist heading_normal_dist(heading, pow(get("variance_heading", params, 0.0), 0.5));
            params["heading"] = std::to_string(heading_normal_dist(*gener));

            bool use_variance_all_ents = scrimmage::get<bool>("use_variance_all_ents", params, false);

            // Use variance if not the first entity in this group, or if a
            // collision exists (This happens when you place <entity> tags"
            // at the same location). Or, if use_variance_all_ents is
            // specified as true in the entity block.
            if (!gen_info.first_in_group || collision_exists(pos) || use_variance_all_ents) {

                // Use the uniform distribution to place aircraft
                // within the x/y variance
                int ct = 0;
                const int max_ct = 1e6;
                bool reselect_pos = collision_exists(pos) || use_variance_all_ents;
                while (ct++ < max_ct && !exit_ && reselect_pos) {
                    pos(0) = x_normal_dist(*gener);
                    pos(1) = y_normal_dist(*gener);
                    pos(2) = z_normal_dist(*gener);
                    reselect_pos = collision_exists(pos);
                }

                if (ct >= max_ct) {
                    cout << "----------------------------------" << endl;
                    cout << "ERROR: Having difficulty finding collision-free location for entity at: "
                         << "(" << x0 << "," << y0 << "," << z0 << ")" << endl
                         << "With variance: (" << pos(0) << "," << pos(1) << "," << pos(2) << ")" << endl;
                    return false;
                } else if (exit_) {
                    return false;
                }

                params["x"] = std::to_string(pos(0));
                params["y"] = std::to_string(pos(1));
                params["z"] = std::to_string(pos(2));
            } else {
                mp_->gen_info()[ent_desc_id].first_in_group = false;
            }

            // Fill in the ent's lat/lon/alt value
            double lat, lon, alt;
            proj_->Reverse(pos(0), pos(1), pos(2), lat, lon, alt);
            params["latitude"] = std::to_string(lat);
            params["longitude"] = std::to_string(lon);
            params["altitude"] = std::to_string(alt);

            std::shared_ptr<Entity> ent = std::make_shared<Entity>();
            ent->set_random(random_);

            contacts_mutex_.lock();
            AttributeMap &attr_map = mp_->entity_attributes()[ent_desc_id];
            bool ent_status = ent->init(attr_map, params,
                contacts_, mp_, proj_, next_id_, ent_desc_id,
                plugin_manager_, file_search_, rtree_, pubsub_, time_);
            contacts_mutex_.unlock();

            if (!ent_status) {
                cout << "Failed to parse entity at start position: "
                     << "x=" << x0 << ", y=" << y0 << endl;
                return false;
            }

            (*id_to_team_map_)[ent->id().id()] = ent->id().team_id();
            (*id_to_ent_map_)[ent->id().id()] = ent;

            contact_visuals_[ent->id().id()] = ent->contact_visual();

            // Send the visual information to the viewer
            outgoing_interface_->send_contact_visual(ent->contact_visual());

            // Store pointer to entities that aren't ready yet
            if (!ent->ready()) {
                not_ready_.push_back(ent);
            }

            ents_.push_back(ent);
            rtree_->add(ent->state()->pos(), ent->id());
            contacts_mutex_.lock();
            (*contacts_)[ent->id().id()] =
                Contact(ent->id(), ent->radius(), ent->state(),
                    ent->type(), ent->contact_visual(), ent->properties());
            contacts_mutex_.unlock();

            auto msg = std::make_shared<Message<sm::EntityGenerated>>();
            msg->data.set_entity_id(ent->id().id());
            pub_ent_gen_->publish(msg);

            next_id_++;
            gen_info.total_count--;

            // Is rate generation enabled?
            if (gen_info.rate > 0) {
                NormDist norm_dist(t + 1.0 / gen_info.rate, gen_info.time_variance);

                // save next gen time to pointer to next gen time
                gen_time = norm_dist(*random_->gener());
                if (gen_time <= t) {
                    cout << "Next generation time less than current time. "
                         << "generate_time_variance is too large." << endl;
                    gen_time = t + (1.0 / gen_info.rate);
                }
            }

            if (++gen_count >= gen_info.gen_count) {
                break;
            }
        }
    }

    // Delete gen_info's that don't have ents remaining (count==0:
    remove_if(mp_->gen_info(), [&](auto &kv) {return kv.second.total_count <= 0;});
    return true;
}

void SimControl::set_mission_parse(MissionParsePtr mp) { mp_ = mp; }

MissionParsePtr SimControl::mp() { return mp_; }

void SimControl::set_log(std::shared_ptr<Log> &log) { log_ = log; }

bool SimControl::enable_gui() {
    return mp_->enable_gui();
}

void SimControl::start() {
    thread_ = std::thread(&SimControl::run, this);
}

void SimControl::display_progress(bool enable) { display_progress_ = enable; }

void SimControl::join() {
    thread_.join();
}

void SimControl::create_rtree() {
    rtree_->clear();
    for (EntityPtr &ent: ents_) {
        rtree_->add(ent->state()->pos(), ent->id());
    }
}

void SimControl::set_autonomy_contacts() {
    std::map<std::string, AutonomyPtr> autonomy_map;
    for (EntityPtr &ent : ents_) {
        for (AutonomyPtr &autonomy : ent->autonomies()) {
            if (autonomy->need_reset()) {
                std::string type = autonomy->type();
                auto it = autonomy_map.find(type);
                if (it == autonomy_map.end()) {
                    contacts_mutex_.lock();
                    autonomy->set_contacts(contacts_);
                    contacts_mutex_.unlock();
                    autonomy_map[type] = autonomy;
                } else {
                    contacts_mutex_.lock();
                    autonomy->set_contacts_from_plugin(it->second);
                    contacts_mutex_.unlock();
                }
            }
        }
    }
}

bool SimControl::run_networks() {
    bool all_true = true;
    for (auto &kv : *networks_) {
        bool result = kv.second->step(pubsub_->pubs()[kv.second->name()],
                                      pubsub_->subs()[kv.second->name()]);
        if (!result) {
            cout << "Network requested simulation termination: "
                 << kv.second->name() << endl;
        }
        all_true &= result;

        shapes_[0].insert(shapes_[0].end(), kv.second->shapes().begin(),
                          kv.second->shapes().end());
        kv.second->shapes().clear();
    }
    return all_true;
}

bool SimControl::run_interaction_detection() {
    bool any_false = false;
    for (EntityInteractionPtr ent_inter : ent_inters_) {
        // Execute callbacks for received messages before calling
        // entity interaction plugins
        for (SubscriberBasePtr &sub : ent_inter->subs()) {
            for (auto msg : sub->msgs<sc::MessageBase>(true)) {
                sub->accept(msg);
            }
        }
        bool result = ent_inter->step_entity_interaction(ents_, t_, dt_);
        if (!result) {
            cout << "Entity interaction requested simulation termination: "
                 << ent_inter->name() << endl;
        }
        any_false = any_false || !result;
        shapes_[0].insert(shapes_[0].end(), ent_inter->shapes().begin(),
                          ent_inter->shapes().end());
        ent_inter->shapes().clear();
    }

    // Determine if entities need to be removed
    for (auto it : ents_) {
        if (!it->is_alive() && it->posthumous(this->t())) {
            int id = it->id().id();

            auto msg = std::make_shared<Message<sm::EntityRemoved>>();
            msg->data.set_entity_id(id);
            pub_ent_rm_->publish(msg);

            // Set the entity and contact to inactive to remove from
            // simulation
            it->set_active(false);
            auto it_cnt = contacts_->find(id);
            if (it_cnt != contacts_->end()) {
                it_cnt->second.set_active(false);
            } else {
                cout << "Failed to find contact to set inactive." << endl;
            }
        }
    }
    return any_false;
}

bool SimControl::run_metrics() {
    bool all_true = true;
    for (MetricsPtr &metric : metrics_) {
        // Execute callbacks for received messages before calling
        // metrics
        for (SubscriberBasePtr &sub : metric->subs()) {
            for (auto msg : sub->msgs<sc::MessageBase>(true)) {
                sub->accept(msg);
            }
        }
        all_true &= metric->step_metrics(t_, dt_);
    }
    return all_true;
}

bool SimControl::run_logging() {
    contacts_mutex_.lock();
    outgoing_interface_->send_frame(t_, contacts_);
    contacts_mutex_.unlock();
    return true;
}

void SimControl::run_remove_inactive() {
    auto it = ents_.begin();
    while (it != ents_.end()) {
        if (!(*it)->active()) {
            int id = (*it)->id().id();
            (*it)->close(t());
            it = ents_.erase(it);
            contacts_mutex_.lock();
            contacts_->erase(id);
            contacts_mutex_.unlock();
        } else {
            ++it;
        }
    }
}

void SimControl::run() {
    start_overall_timer();

    // Simulate over the time range
    int loop_number = 0;
    bool exit_loop = false;
    set_time(t0_);
    bool end_condition_interaction;

    do {
        double t = this->t();
        reseed_task_.update(t);
        start_loop_timer();

        if (!generate_entities(t)) {
            cout << "Failed to generate entity" << endl;
        }

        if (!wait_for_ready()) {
            cleanup();
            return;
        }

        create_rtree();
        set_autonomy_contacts();
        if (!run_entities()) {
            std::cout << "Exiting due to plugin request." << std::endl;
            break;
        }

        end_condition_interaction = run_interaction_detection();
        if (end_condition_interaction) {
            auto msg = std::make_shared<Message<sm::EntityInteractionExit>>();
            pub_ent_int_exit_->publish(msg);
        }

        // The networks are run before the metrics, so that messages that are
        // published on the final time stamp can be processed by the metrics.
        if (!run_networks()) {
            std::cout << "Exiting due to network plugin request."
                      << std::endl;
            break;
        }

        if (!run_metrics()) {
            std::cout << "Exiting due to metrics plugin exception" << std::endl;
            break;
        }

        if (!run_logging()) {
            std::cout << "Exiting due to logging exception" << std::endl;
            break;
        }

        run_remove_inactive();
        run_send_shapes();
        run_send_contact_visuals(); // send updated visuals

        if (display_progress_) {
            if (loop_number % 100 == 0) {
                sc::display_progress((tend_ == 0) ? 1.0 : t / tend_);
            }
        }

        if (screenshot_task_.update(t_).first) {
            request_screenshot();
        }

        // Wait loop timer.
        // Stay in loop if currently paused.
        do {
            loop_wait();

            // Were we told to exit, externally?
            exit_mutex_.lock();
            if (exit_) {
                exit_loop = true;
            }
            exit_mutex_.unlock();

            if (single_step()) {
                single_step(false);
                take_step_mutex_.lock();
                take_step_ = true;
                take_step_mutex_.unlock();
                pause(prev_paused_);
                break;
            }

            run_check_network_msgs();

            scrimmage_proto::SimInfo info;
            info.set_time(this->t());
            info.set_desired_warp(this->time_warp());
            info.set_actual_warp(this->actual_time_warp());
            info.set_shutting_down(false);
            outgoing_interface_->send_sim_info(info);
        } while (paused() && !exit_loop);

        // Increment time and loop counter
        set_time(t + dt_);
        loop_number++;
        prev_paused_ = paused_;
    } while (!end_condition_interaction && !end_condition_reached(t(), dt_) && !exit_loop);

    cleanup();
    return;
}

void SimControl::cleanup() {
    if (use_entity_threads_) {
        entity_pool_stop_ = true;
        entity_pool_condition_var_.notify_all();
        for (std::thread &t : entity_worker_threads_) {
            t.join();
        }
    }

    // account for last step
    set_time(t() - dt_);

    // Save EntityPresentAtEnd messages
    for (EntityPtr &ent : ents_) {
        auto msg = std::make_shared<Message<sm::EntityPresentAtEnd>>();
        msg->data.set_entity_id(ent->id().id());
        pub_ent_pres_end_->publish(msg);
        ent->close(t());
    }

    run_logging();

    if (display_progress_) cout << endl;

    set_finished(true);
}

bool SimControl::wait_for_ready() {
    // Wait for all entities to be ready
    int not_ready_loop = 0;
    while (!not_ready_.empty()) {
        for (std::list<EntityPtr>::iterator it = not_ready_.begin();
             it != not_ready_.end(); /* No increment */) {
            if ((*it)->ready()) {
                // Remove references to entities that are ready
                it = not_ready_.erase(it);
            } else {
                ++it;
            }
        }

        // Check to see if we were told to exit
        exit_mutex_.lock();
        bool exit = exit_;
        exit_mutex_.unlock();
        if (exit) {
            cout << "Simulation ended waiting for entity to be ready" << endl;
            return false;
        }

        // Give other possible threads (MOOS, ROS, etc) a chance to
        // run.
        if (!not_ready_.empty()) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        }
        if (not_ready_loop > 1e4) {
            cout << "Warning: Entities taking a long time to be ready" << endl;
            not_ready_loop = 0;
        }
        not_ready_loop++;
    }
    return true;
}

bool SimControl::end_condition_reached(double t, double dt) {

    if (end_conditions_.count(EndConditionFlags::TIME) &&t > mp_->tend() + dt / 2.0) {
        auto msg = std::make_shared<Message<sm::EndTime>>();
        pub_end_time_->publish(msg);
        return true;
    }

    if (ents_.empty()) {
        auto msg = std::make_shared<Message<sm::NoTeamsPresent>>();
        pub_no_teams_->publish(msg);
        if (end_conditions_.count(EndConditionFlags::ALL_DEAD) ||
            end_conditions_.count(EndConditionFlags::ONE_TEAM)) {

            std::cout << std::endl << "End of Simulation: No Entities Remaining" << std::endl;
            return true;
        }
    } else if (end_conditions_.count(EndConditionFlags::ONE_TEAM)) {
        const int team1_id = ents_.front()->id().team_id();
        const bool all_same_team = std::all_of(ents_.rbegin(), ents_.rend(),
            [&](EntityPtr &ent) {return team1_id == ent->id().team_id();});
        if (all_same_team) {
            auto msg = std::make_shared<Message<sm::OneTeamPresent>>();
            pub_one_team_->publish(msg);
            std::cout << std::endl << "End of Simulation: One Team (" << team1_id << ")" << std::endl;
            return true;
        }
    }
    return false;
}

Timer &SimControl::timer() {return timer_;}

std::list<MetricsPtr> &SimControl::metrics() { return metrics_; }

PluginManagerPtr &SimControl::plugin_manager() {return plugin_manager_;}

FileSearchPtr &SimControl::file_search() {return file_search_;}

bool SimControl::take_step() {
    take_step_mutex_.lock();
    bool value = take_step_;
    take_step_mutex_.unlock();
    return value;
}

void SimControl::step_taken() {
    take_step_mutex_.lock();
    take_step_ = false;
    take_step_mutex_.unlock();
}

void SimControl::set_incoming_interface(InterfacePtr &incoming_interface)
{ incoming_interface_ = incoming_interface; }

void SimControl::set_outgoing_interface(InterfacePtr &outgoing_interface)
{ outgoing_interface_ = outgoing_interface; }

void SimControl::run_check_network_msgs() {
    // Do we have any simcontrol message updates from GUI?
    if (incoming_interface_->gui_msg_update()) {
        incoming_interface_->gui_msg_mutex.lock();
        auto &control = incoming_interface_->gui_msg();
        auto it = control.begin();
        while (it != control.end()) {
            if (it->inc_warp()) {
                this->inc_warp();
            } else if (it->dec_warp()) {
                this->dec_warp();
            } else if (it->toggle_pause()) {
                this->pause(!this->paused());
                prev_paused_ = paused();
            } else if (it->single_step()) {
                this->single_step(true);
            } else if (it->shutting_down()) {
                send_shutdown_msg_ = false;
                this->force_exit();
            } else if (it->request_cached()) {
                outgoing_interface_->send_cached();
            }
            control.erase(it++);
        }
        incoming_interface_->gui_msg_mutex.unlock();
    }
}

bool SimControl::collision_exists(Eigen::Vector3d &p) {
    return std::any_of(ent_inters_.begin(), ent_inters_.end(),
        [&](auto ent_inter) {return ent_inter->collision_exists(ents_, p);});
}

void SimControl::force_exit() {
    exit_mutex_.lock();
    exit_ = true;
    exit_mutex_.unlock();
}

bool SimControl::external_exit() {
    bool exit;
    exit_mutex_.lock();
    exit = exit_;
    exit_mutex_.unlock();
    return exit;
}

void SimControl::set_finished(bool finished) {
    scrimmage_proto::SimInfo info;
    info.set_time(this->t());
    info.set_desired_warp(this->time_warp());
    info.set_actual_warp(this->actual_time_warp());
    info.set_shutting_down(true);
    if (send_shutdown_msg_) {
        outgoing_interface_->send_sim_info(info);
    }

    finished_mutex_.lock();
    finished_ = finished;
    finished_mutex_.unlock();
}

bool SimControl::finished() {
    bool status;
    finished_mutex_.lock();
    status = finished_;
    finished_mutex_.unlock();

    return status;
}

void SimControl::get_contacts(std::unordered_map<int, Contact> &contacts) {
    // The Viewer GUI also looks at the contacts
    contacts_mutex_.lock();
    contacts = *contacts_;
    for (auto &kv : contacts) {
        sc::State state = *kv.second.state();
        *kv.second.state() = state;
    }
    contacts_mutex_.unlock();
}

void SimControl::set_contacts(ContactMapPtr &contacts) {
    contacts_mutex_.lock();
    contacts_ = contacts;
    contacts_mutex_.unlock();
}

void SimControl::get_contact_visuals(std::map<int, ContactVisualPtr> &contact_visuals) {
    contact_visuals = contact_visuals_;
}

void SimControl::set_contact_visuals(std::map<int, ContactVisualPtr> &contact_visuals) {
    contact_visuals_ = contact_visuals;
}

void SimControl::inc_warp() {
    timer_mutex_.lock();
    timer_.inc_warp();
    timer_mutex_.unlock();
}

void SimControl::dec_warp() {
    timer_mutex_.lock();
    timer_.dec_warp();
    timer_mutex_.unlock();
}

void SimControl::pause(bool pause) {
    paused_mutex_.lock();
    paused_ = pause;
    paused_mutex_.unlock();
}

bool SimControl::paused() {
    bool result;
    paused_mutex_.lock();
    result = paused_;
    paused_mutex_.unlock();
    return result;
}

double SimControl::time_warp() {
    double warp;
    timer_mutex_.lock();
    warp = timer_.time_warp();
    timer_mutex_.unlock();
    return warp;
}

double SimControl::actual_time_warp() { return -1; }

void SimControl::set_time(double t) {
    time_mutex_.lock();
    t_ = t;
    time_->set_t(t_);
    time_mutex_.unlock();
}

double SimControl::t() {
    double t;
    time_mutex_.lock();
    t = t_;
    time_mutex_.unlock();
    return t;
}

void SimControl::setup_timer(double rate, double time_warp) {
    // dt_ is a period, 1 / dt_ is a rate (Hz)
    timer_mutex_.lock();
    timer_.set_iterate_rate(rate);
    timer_.set_time_warp(time_warp);
    timer_.update_time_config();
    timer_mutex_.unlock();
}

void SimControl::start_overall_timer() {
    timer_mutex_.lock();
    timer_.start_overall_timer();
    timer_mutex_.unlock();
}

void SimControl::start_loop_timer() {
    timer_mutex_.lock();
    timer_.start_loop_timer();
    timer_mutex_.unlock();
}

void SimControl::loop_wait() {
    timer_mutex_.lock();
    timer_.loop_wait();
    timer_mutex_.unlock();
}

void SimControl::single_step(bool value) {
    single_step_mutex_.lock();
    single_step_ = value;
    single_step_mutex_.unlock();
}

bool SimControl::single_step() {
    single_step_mutex_.lock();
    bool value = single_step_;
    single_step_mutex_.unlock();
    return value;
}

void SimControl::worker() {
    while (true) {
        entity_pool_mutex_.lock();
        entity_pool_condition_var_.wait(entity_pool_mutex_);
        entity_pool_mutex_.unlock();

        if (entity_pool_stop_) {
            break;
        }

        while (true) {
            entity_pool_mutex_.lock();
            if (entity_pool_queue_.empty()) {
                entity_pool_mutex_.unlock();
                break;
            }
            std::shared_ptr<Task> task = entity_pool_queue_.front();
            EntityPtr ent = task->ent;
            entity_pool_queue_.pop_front();
            entity_pool_mutex_.unlock();

            bool success = true;
            for (AutonomyPtr &autonomy : ent->autonomies()) {
                // Execute callbacks for received messages before calling
                // step_autonomy
                for (SubscriberBasePtr &sub : autonomy->subs()) {
                    for (auto msg : sub->msgs<sc::MessageBase>(true)) {
                        sub->accept(msg);
                    }
                }
                success &= autonomy->step_autonomy(t_, dt_);
            }

            entity_pool_mutex_.lock();
            task->prom.set_value(success);
            entity_pool_mutex_.unlock();
        }
    }
}

void print_err(PluginPtr p) {
    std::cout << "failed to update entity " << p->parent()->id().id()
        << ", plugin type \"" << p->type() << "\""
        << ", plugin name \"" << p->name() << "\"" << std::endl;
}

bool SimControl::run_entities() {
    contacts_mutex_.lock();
    bool success = true;

    // run autonomies threaded or in a single thread
    if (use_entity_threads_) {

        // put tasks on queue
        std::vector<std::future<bool>> futures;
        futures.reserve(ents_.size());

        entity_pool_mutex_.lock();
        for (EntityPtr &ent : ents_) {
            std::shared_ptr<Task> task = std::make_shared<Task>();
            task->ent = ent;
            entity_pool_queue_.push_back(task);
            futures.push_back(task->prom.get_future());
        }
        entity_pool_mutex_.unlock();

        // tell the threads to run
        entity_pool_condition_var_.notify_all();

        // wait for results
        for (std::future<bool> &future : futures) {
            success &= future.get();
        }
    } else {

        for (EntityPtr &ent : ents_) {
            for (AutonomyPtr &a : ent->autonomies()) {
                // Execute callbacks for received messages before calling
                // step_autonomy
                for (SubscriberBasePtr &sub : a->subs()) {
                    for (auto msg : sub->msgs<sc::MessageBase>(true)) {
                        sub->accept(msg);
                    }
                }
                if (!a->step_autonomy(t_, dt_)) {
                    print_err(a);
                    success = false;
                }
            }
        }
    }

    double motion_dt = dt_ / mp_->motion_multiplier();
    double temp_t = t_;
    for (int i = 0; i < mp_->motion_multiplier(); i++) {
        // Run each entity's controllers
        for (EntityPtr &ent : ents_) {
            for (ControllerPtr &ctrl : ent->controllers()) {
                // Execute callbacks for received messages before calling
                // controllers
                for (SubscriberBasePtr &sub : ctrl->subs()) {
                    for (auto msg : sub->msgs<sc::MessageBase>(true)) {
                        sub->accept(msg);
                    }
                }
                if (!ctrl->step(temp_t, motion_dt)) {
                    print_err(ctrl);
                    success = false;
                }
            }
        }

        // Run each entity's motion model
        for (EntityPtr &ent : ents_) {
            // Execute callbacks for received messages before calling
            // motion models
            for (SubscriberBasePtr &sub : ent->motion()->subs()) {
                for (auto msg : sub->msgs<sc::MessageBase>(true)) {
                    sub->accept(msg);
                }
            }
            if (!ent->motion()->step(temp_t, motion_dt)) {
                print_err(ent->motion());
                success = false;
            }
        }
        temp_t += motion_dt;
    }

    for (EntityPtr &ent : ents_) {
        ent->setup_desired_state();
    }

    for (EntityPtr &ent : ents_) {
        for (AutonomyPtr &autonomy : ent->autonomies()) {
            if (autonomy->need_reset()) {
                autonomy->set_state(ent->motion()->state());
            }
        }
    }

    contacts_mutex_.unlock();

    for (EntityPtr &ent : ents_) {
        std::list<ShapePtr> &shapes = shapes_[ent->id().id()];
        for (AutonomyPtr &autonomy : ent->autonomies()) {
            shapes.insert(shapes.end(), autonomy->shapes().begin(), autonomy->shapes().end());
            autonomy->shapes().clear();
        }
    }
    return success;
}

void SimControl::run_send_shapes() {
    // Convert map of shapes to sp::Shapes type
    scrimmage_proto::Shapes shapes;
    shapes.set_time(this->t());
    for (auto &kv : shapes_) {
        for (auto &shape : kv.second) {
            scrimmage_proto::Shape *s = shapes.add_shape();
            *s = *shape;
        }
    }
    outgoing_interface_->send_shapes(shapes);
    shapes_.clear();
}

void SimControl::run_send_contact_visuals() {
    for (EntityPtr &ent : ents_) {
        if (ent->visual_changed()) {
            outgoing_interface_->send_contact_visual(ent->contact_visual());
            ent->set_visual_changed(false);
        }
    }
}
} // namespace scrimmage
