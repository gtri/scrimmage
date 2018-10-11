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
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/ParameterServer.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>
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

#if ENABLE_PYTHON_BINDINGS == 1
#include <pybind11/pybind11.h>
#endif

#include <GeographicLib/LocalCartesian.hpp>

#include <boost/thread.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/numeric.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sm = scrimmage_msgs;
namespace br = boost::range;
namespace ba = boost::adaptors;

using std::cout;
using std::endl;

namespace scrimmage {

SimControl::SimControl() :
    id_to_team_map_(std::make_shared<std::unordered_map<int, int>>()),
    id_to_ent_map_(std::make_shared<std::unordered_map<int, EntityPtr>>()),
    time_(std::make_shared<Time>()),
    param_server_(std::make_shared<ParameterServer>()),
    timer_(Timer()),
    random_(std::make_shared<Random>()),
    plugin_manager_(std::make_shared<PluginManager>()),
    networks_(std::make_shared<std::map<std::string, NetworkPtr>>()),
    pubsub_(std::make_shared<PubSub>()),
    file_search_(std::make_shared<FileSearch>()),
    sim_plugin_(std::make_shared<Plugin>()),
    limited_verbosity_(false) {

    pause(false);
    prev_paused_ = false;
    single_step(false);

    contacts_mutex_.lock();
    contacts_ = std::make_shared<ContactMap>();
    contacts_mutex_.unlock();
}

bool SimControl::init() {
    ents_.clear();
    ent_inters_.clear();
    metrics_.clear();
    contacts_->clear();
    shapes_.clear();
    contact_visuals_.clear();
    networks_->clear();
    pubsub_->pubs().clear();
    pubsub_->subs().clear();

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
        auto seed = std::stoul(mp_->params()["seed"]);
        random_->seed(seed);
#if ENABLE_PYTHON_BINDINGS == 1
        pybind11::module::import("random").attr("seed")(seed);
        try {
            pybind11::module::import("numpy.random").attr("seed")(seed);
        } catch (pybind11::error_already_set) {
            // ignore. numpy not installed
        }
#endif
    } else {
        random_->seed();
    }
    log_->write_ascii("Seed: " + std::to_string(random_->get_seed()));

    auto get_count = [&](auto &kv) {return kv.second.total_count;};
    int max_num_entities =
        boost::accumulate(mp_->gen_info() | ba::transformed(get_count), 0);

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

    // If the GlobalNetwork doesn't exist, add it.
    auto it_global_network = std::find(mp_->network_names().begin(),
                                       mp_->network_names().end(),
                                       "GlobalNetwork");
    if (it_global_network == mp_->network_names().end()) {
        mp_->network_names().push_back("GlobalNetwork");
    }

    // setup sim_plugin
    sim_plugin_->parent()->set_random(random_);
    sim_plugin_->set_time(time_);
    sim_plugin_->set_pubsub(pubsub_);

    // Create base shape objects
    for (auto &kv : mp_->team_info()) {
        int i = 0;
        for (Eigen::Vector3d &base_pos : kv.second.bases) {
            auto base = std::make_shared<scrimmage_proto::Shape>();
            base->mutable_sphere()->set_radius(kv.second.radii[i]);
            sc::set(base->mutable_sphere()->mutable_center(), base_pos);

            base->set_opacity(kv.second.opacities[i]);
            sc::set(base->mutable_color(), kv.second.color);
            base->set_persistent(true);
            sim_plugin_->draw_shape(base);
            i++;
        }
    }

    // Get the list of "metrics" plugins
    SimUtilsInfo info;
    info.mp = mp_;
    info.plugin_manager = plugin_manager_;
    info.file_search = file_search_;
    info.rtree = rtree_;
    info.pubsub = pubsub_;
    info.time = time_;
    info.param_server = param_server_;
    info.random = random_;
    info.id_to_team_map = id_to_team_map_;
    info.id_to_ent_map = id_to_ent_map_;

    networks_ = std::make_shared<NetworkMap>();
    if (!create_networks(info, *networks_)) return false;
    if (!create_metrics(info, contacts_, metrics_)) return false;
    if (!create_ent_inters(info, contacts_, shapes_[0], ent_inters_)) return false;

    // Setup simcontrol's pubsub plugin
    pub_end_time_ = sim_plugin_->advertise("GlobalNetwork", "EndTime");
    pub_ent_gen_ = sim_plugin_->advertise("GlobalNetwork", "EntityGenerated");
    pub_ent_rm_ = sim_plugin_->advertise("GlobalNetwork", "EntityRemoved");
    pub_ent_pres_end_ = sim_plugin_->advertise("GlobalNetwork", "EntityPresentAtEnd");
    pub_ent_int_exit_ = sim_plugin_->advertise("GlobalNetwork", "EntityInteractionExit");
    pub_no_teams_ = sim_plugin_->advertise("GlobalNetwork", "NoTeamsPresent");
    pub_one_team_ = sim_plugin_->advertise("GlobalNetwork", "OneTeamPresent");
    pub_world_point_clicked_ = sim_plugin_->advertise("GlobalNetwork", "WorldPointClicked");
    pub_custom_key_ = sim_plugin_->advertise("GlobalNetwork", "CustomKeyPress");

    contacts_mutex_.lock();
    contacts_->reserve(max_num_entities+1);
    contacts_mutex_.unlock();

    if (get("show_plugins", mp_->params(), false)) {
        plugin_manager_->print_returned_plugins();
    }

    if (get("multi_threaded", mp_->params(), false)) {
        auto it = mp_->attributes().find("multi_threaded");
        if (it != mp_->attributes().end()) {
            auto &attr_map = it->second;
            auto add = [&](auto attr_str, auto attr_type) {
                if (str2bool(get(attr_str, attr_map, "true"))) {
                    entity_thread_types_.insert(attr_type);
                }
            };
            add("autonomy", Task::Type::AUTONOMY);
            add("controller", Task::Type::CONTROLLER);
            add("motion", Task::Type::MOTION);
            add("sensor", Task::Type::SENSOR);
        }

        if (!entity_thread_types_.empty()) {
            entity_pool_stop_ = false;
            num_entity_threads_ = get("num_threads", mp_->attributes()["multi_threaded"], 1);
            entity_worker_threads_.clear();
            entity_worker_threads_.reserve(num_entity_threads_);
            for (int i = 0; i < num_entity_threads_; i++) {
                entity_worker_threads_.push_back(std::thread(&SimControl::worker, this));
            }
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
    gui_msg.set_time(t_ + dt_);
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
                plugin_manager_, file_search_, rtree_, pubsub_, time_,
                param_server_,
                std::set<std::string>{},
                [](std::map<std::string, std::string>&){});
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
        if (!result && kv.second->print_err_on_exit) {
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

    auto run_interaction = [&](auto ent_inter) {
        bool result = ent_inter->step_entity_interaction(ents_, t_, dt_);
        if (!result && ent_inter->print_err_on_exit) {
            cout << "Entity interaction requested simulation termination: "
                 << ent_inter->name() << endl;
        }
        return result;
    };

    auto handle_shapes = [&](auto ent_inter) {
        shapes_[0].insert(shapes_[0].end(), ent_inter->shapes().begin(),
                          ent_inter->shapes().end());
        ent_inter->shapes().clear();
    };

    br::for_each(ent_inters_, run_callbacks);
    bool success = std::all_of(ent_inters_.begin(), ent_inters_.end(), run_interaction);
    br::for_each(ent_inters_, handle_shapes);

    // Determine if entities need to be removed
    for (auto &ent : ents_) {
        if (!ent->is_alive() && ent->posthumous(this->t())) {
            int id = ent->id().id();

            auto msg = std::make_shared<Message<sm::EntityRemoved>>();
            msg->data.set_entity_id(id);
            pub_ent_rm_->publish(msg);

            // Set the entity and contact to inactive to remove from
            // simulation
            ent->set_active(false);
            auto it_cnt = contacts_->find(id);
            if (it_cnt != contacts_->end()) {
                it_cnt->second.set_active(false);
            } else {
                cout << "Failed to find contact to set inactive." << endl;
            }
        }
    }
    return success;
}

bool SimControl::run_metrics() {
    br::for_each(metrics_, run_callbacks);
    auto run_metric = [&](auto &metric) {return metric->step_metrics(t_, dt_);};
    return std::all_of(metrics_.begin(), metrics_.end(), run_metric);
}

bool SimControl::run_logging() {
    contacts_mutex_.lock();
    outgoing_interface_->send_frame(t_ + dt_, contacts_);
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

bool SimControl::run_single_step(int loop_number) {
    double t = this->t();
    reseed_task_.update(t);
    start_loop_timer();

    if (!generate_entities(t)) {
        cout << "Failed to generate entity" << endl;
        return false;
    }

    if (screenshot_task_.update(t_).first) {
        request_screenshot();
    }

    if (!run_logging()) {
        if (!limited_verbosity_) {
            std::cout << "Exiting due to logging exception" << std::endl;
        }
        return false;
    }

    // Wait loop timer.
    // Stay in loop if currently paused.
    bool exit_loop = false;
    do {

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

    if (!wait_for_ready()) {
        cleanup();
        return false;
    }

    create_rtree();
    set_autonomy_contacts();
    if (!run_entities()) {
        if (!limited_verbosity_) {
            std::cout << "Exiting due to plugin request." << std::endl;
        }
        return false;
    }

    if (!run_sensors()) {
        if (!limited_verbosity_) {
            std::cout << "Exiting due to plugin request." << std::endl;
        }
        return false;
    }

    if (!run_interaction_detection()) {
        auto msg = std::make_shared<Message<sm::EntityInteractionExit>>();
        pub_ent_int_exit_->publish(msg);
        return false;
    }

    // The networks are run before the metrics, so that messages that are
    // published on the final time stamp can be processed by the metrics.
    if (!run_networks()) {
        if (!limited_verbosity_) {
            std::cout << "Exiting due to network plugin request." << std::endl;
        }
        return false;
    }

    if (!run_metrics()) {
        if (!limited_verbosity_) {
            std::cout << "Exiting due to metrics plugin exception" << std::endl;
        }
        return false;
    }

    run_remove_inactive();
    run_send_shapes();
    run_send_contact_visuals(); // send updated visuals

    if (display_progress_) {
        if (loop_number % 100 == 0) {
            sc::display_progress((tend_ == 0) ? 1.0 : t / tend_);
        }
    }

    // Increment time and loop counter
    loop_wait();
    set_time(t + dt_);
    prev_paused_ = paused_;

    return !exit_loop;
}

void SimControl::run() {
    start_overall_timer();

    // Simulate over the time range
    int loop_number = 0;
    set_time(t0_ - dt_);

    bool success = true;
    if (!generate_entities(t0_ - dt_)) {
        cout << "Failed to generate entity" << endl;
        success = false;
    }

    if (!run_interaction_detection()) {
        auto msg = std::make_shared<Message<sm::EntityInteractionExit>>();
        pub_ent_int_exit_->publish(msg);
        success = false;
    }

    set_time(t0_);
    while (success &&
           run_single_step(loop_number++) &&
           !end_condition_reached()) {}
    cleanup();
}

void SimControl::cleanup() {
    if (!entity_thread_types_.empty()) {
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

    for (EntityInteractionPtr ent_inter : ent_inters_) {
        ent_inter->close_plugin(t());
    }

    for (auto &kv : *networks_) {
        kv.second->close_plugin(t());
    }

    run_logging();

    if (display_progress_) cout << endl;

    set_finished(true);
}

void SimControl::close() {
    id_to_ent_map_ = nullptr;
    incoming_interface_ = nullptr;
    outgoing_interface_ = nullptr;
    mp_ = nullptr;
    ents_.clear();
    contacts_ = nullptr;
    shapes_.clear();
    contact_visuals_.clear();
    time_ = nullptr;
    entity_pool_queue_.clear();
    log_ = nullptr;
    random_ = nullptr;
    plugin_manager_ = nullptr;
    proj_ = nullptr;

    ent_inters_.clear();
    metrics_.clear();
    networks_->clear();
    pubsub_->pubs().clear();
    pubsub_->subs().clear();
    pubsub_ = nullptr;
    file_search_ = nullptr;
    rtree_ = nullptr;
    sim_plugin_->close_plugin(t());
    pub_end_time_ = nullptr;
    pub_ent_gen_ = nullptr;
    pub_ent_rm_ = nullptr;
    pub_ent_pres_end_ = nullptr;
    pub_ent_int_exit_ = nullptr;
    pub_no_teams_ = nullptr;
    pub_one_team_ = nullptr;
    pub_world_point_clicked_ = nullptr;
    pub_custom_key_ = nullptr;
    not_ready_.clear();
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

bool SimControl::end_condition_reached() {

    if (end_conditions_.count(EndConditionFlags::TIME) && t() > mp_->tend() - dt_ / 2.0) {
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
            } else if (it->custom_key() != "") {
                auto msg = std::make_shared<Message<std::string>>();
                std::string key(it->custom_key());
                msg->data = key;
                pub_custom_key_->publish(msg);
            }
            control.erase(it++);
        }
        incoming_interface_->gui_msg_mutex.unlock();
    }

    if (incoming_interface_->world_point_clicked_msg_update()) {
        incoming_interface_->world_point_clicked_msg_mutex.lock();
        auto &msg_list = incoming_interface_->world_point_clicked_msg();
        auto it = msg_list.begin();
        while (it != msg_list.end()) {
            auto msg = std::make_shared<Message<sp::WorldPointClicked>>();
            msg->data = *it;
            pub_world_point_clicked_->publish(msg);
            msg_list.erase(it++);
        }
        incoming_interface_->world_point_clicked_msg_mutex.unlock();
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
            Task::Type task_type = task->type;
            double temp_t = task->t;
            double temp_dt = task->dt;
            entity_pool_queue_.pop_front();
            entity_pool_mutex_.unlock();

            bool success = false;
            if (task_type == Task::Type::AUTONOMY) {
                auto &autonomies = ent->autonomies();
                br::for_each(autonomies, run_callbacks);
                auto run = [&](auto &a) {return a->step_autonomy(temp_t, temp_dt);};
                success = std::all_of(autonomies.begin(), autonomies.end(), run);
            } else if (task_type == Task::Type::CONTROLLER) {
                auto &controllers = ent->controllers();
                br::for_each(controllers, run_callbacks);
                auto run = [&](auto &c) {return c->step(temp_t, temp_dt);};
                success = std::all_of(controllers.begin(), controllers.end(), run);
            } else if (task_type == Task::Type::MOTION) {
                success = ent->motion()->step(temp_t, temp_dt);
            } else if (task_type == Task::Type::SENSOR) {
                auto sensors = ent->sensors() | ba::map_values;
                br::for_each(sensors, run_callbacks);
                auto run = [&](auto &s) {return s->step();};
                success = std::all_of(sensors.begin(), sensors.end(), run);
            }

            entity_pool_mutex_.lock();
            task->prom.set_value(success);
            entity_pool_mutex_.unlock();
        }
    }
}

void print_err(PluginPtr p) {
    if (p->print_err_on_exit) {
        std::cout << "failed to update entity " << p->parent()->id().id()
            << ", plugin type \"" << p->type() << "\""
            << ", plugin name \"" << p->name() << "\"" << std::endl;
    }
}

bool SimControl::run_sensors() {
    bool success = true;
    if (entity_thread_types_.count(Task::Type::SENSOR)) {
        success &= add_tasks(Task::Type::SENSOR, t_, dt_);
    } else {
        for (EntityPtr &ent : ents_) {
            br::for_each(ent->sensors() | ba::map_values, run_callbacks);
            for (auto &sensor : ent->sensors() | ba::map_values) {
                if (!sensor->step()) {
                    print_err(sensor);
                    success = false;
                }
            }
        }
    }

    for (EntityPtr &ent : ents_) {
        auto &shapes = shapes_[ent->id().id()];
        for (auto &sensor : ent->sensors() | ba::map_values) {
            shapes.insert(shapes.end(), sensor->shapes().begin(), sensor->shapes().end());
            sensor->shapes().clear();
        }
    }
    return success;
}

bool SimControl::add_tasks(Task::Type type, double t, double dt) {
    // FIXME: this will be much simpler once there is a
    // step function in Plugin.h
    // In particular, we can get rid of Task::Type and
    // more easily do entity_interaction/network plugins in multiple threads.

    // put tasks on queue
    std::vector<std::future<bool>> futures;
    futures.reserve(ents_.size());

    entity_pool_mutex_.lock();
    for (EntityPtr &ent : ents_) {
        std::shared_ptr<Task> task = std::make_shared<Task>();
        task->ent = ent;
        task->type = type;
        task->t = t;
        task->dt = dt;
        entity_pool_queue_.push_back(task);
        futures.push_back(task->prom.get_future());
    }
    entity_pool_mutex_.unlock();

    // tell the threads to run
    entity_pool_condition_var_.notify_all();

    // wait for results
    auto get = [&](auto &future) {return future.get();};
    return std::all_of(futures.begin(), futures.end(), get);
}

bool SimControl::run_entities() {
    contacts_mutex_.lock();
    bool success = true;

    auto exec_step = [&](auto p, auto step_func) {
        run_callbacks(p);
        if (!step_func(p)) {
            print_err(p);
            return false;
        } else {
            return true;
        }
    };

    // run autonomies threaded or in a single thread
    if (entity_thread_types_.count(Task::Type::AUTONOMY)) {
        success &= add_tasks(Task::Type::AUTONOMY, t_, dt_);
    } else {
        for (EntityPtr &ent : ents_) {
            for (auto a : ent->autonomies()) {
                success &= exec_step(a, [&](auto a){return a->step_autonomy(t_, dt_);});
            }
        }
    }

    double motion_dt = dt_ / mp_->motion_multiplier();
    double temp_t = t_;
    for (int i = 0; i < mp_->motion_multiplier(); i++) {
        // run controllers in a single thread since they are serially connected
        for (EntityPtr &ent : ents_) {
            for (auto c : ent->controllers()) {
                success &= exec_step(c, [&](auto c){return c->step(temp_t, motion_dt);});
            }
        }

        // run motion model
        auto step_all = [&](Task::Type type, auto getter) {
            if (entity_thread_types_.count(type)) {
                success &= add_tasks(type, temp_t, motion_dt);
            } else {
                for (EntityPtr &ent : ents_) {
                    auto step = [&](auto p){return p->step(temp_t, motion_dt);};
                    success &= exec_step(getter(ent), step);
                }
            }
        };
        step_all(Task::Type::MOTION, [&](auto ent){return ent->motion();});

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
        auto &shapes = shapes_[ent->id().id()];
        auto add_shapes = [&](const auto &p) {
            shapes.insert(shapes.end(), p->shapes().begin(), p->shapes().end());
            p->shapes().clear();
        };
        br::for_each(ent->autonomies(), add_shapes);
        br::for_each(ent->controllers(), add_shapes);
        add_shapes(ent->motion());
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

bool SimControl::output_runtime() {
    std::ofstream runtime_file(mp_->log_dir() + "/runtime_seconds.txt");
    if (!runtime_file.is_open()) return false;

    double t = timer_.elapsed_time().total_milliseconds() / 1000.0;
    double sim_t = time_->t();
    runtime_file << "wall: " << t << std::endl;
    runtime_file << "sim: " << sim_t << std::endl;
    runtime_file.close();
    return true;
}

bool SimControl::output_summary() {
    std::map<int, double> team_scores;
    std::map<int, std::map<std::string, double>> team_metrics;
    std::list<std::string> headers;

    bool metrics_empty = true;

    // Loop through each of the metrics plugins.
    for (auto metrics : metrics_) {
        if (metrics->get_print_team_summary()) {
            cout << sc::generate_chars("=", 80) << endl;
            cout << metrics->name() << endl;
            cout << sc::generate_chars("=", 80) << endl;
            metrics->calc_team_scores();
            metrics->print_team_summaries();
        }

        // Add all elements from individual metrics plugin to overall
        // metrics data structure
        for (auto const &team_str_double : metrics->team_metrics()) {
            team_metrics[team_str_double.first].insert(team_str_double.second.begin(),
                                                       team_str_double.second.end());
            metrics_empty = false;
        }

        // Calculate aggregated team scores:
        for (auto const &team_score : metrics->team_scores()) {
            if (team_scores.count(team_score.first) == 0) {
                team_scores[team_score.first] = 0;
            }
            team_scores[team_score.first] += team_score.second;
        }

        // Create list of all csv headers
        headers.insert(headers.end(), metrics->headers().begin(),
                       metrics->headers().end());
    }

    // Create headers string
    std::string csv_str = "team_id,score";
    for (std::string header : headers) {
        csv_str += "," + header;
    }
    csv_str += "\n";

    // Loop over each team and generate csv output
    for (auto const &team_str_double : team_metrics) {

        // Each line starts with team_id,score
        csv_str += std::to_string(team_str_double.first);
        csv_str += "," + std::to_string(team_scores[team_str_double.first]);

        // Loop over all possible headers, if the header doesn't exist for
        // a specific team, default the value for that header to zero.
        for (std::string header : headers) {
            csv_str += ",";

            auto it = team_str_double.second.find(header);
            if (it != team_str_double.second.end()) {
                csv_str += std::to_string(it->second);
            } else {
                csv_str += std::to_string(0.0);
            }
        }
        csv_str += "\n";
    }

    // Write CSV string to file
    std::string out_file = mp_->log_dir() + "/summary.csv";
    std::ofstream summary_file(out_file);
    if (!summary_file.is_open()) {
        std::cout << "could not open " << out_file
                  << " for writing metrics" << std::endl;
        return false;
    }
    summary_file << csv_str << std::flush;
    summary_file.close();

    // Print Overall Scores
    if (!metrics_empty) {
        cout << sc::generate_chars("=", 80) << endl;
        cout << "Overall Scores" << endl;
        cout << sc::generate_chars("=", 80) << endl;
        for (auto const &team_score : team_scores) {
            cout << "Team ID: " << team_score.first << endl;
            cout << "Score: " << team_score.second << endl;
            cout << sc::generate_chars("-", 80) << endl;
        }
    }

    return true;
}

void SimControl::set_limited_verbosity(bool limited_verbosity) {
    limited_verbosity_ = limited_verbosity;
}

InterfacePtr SimControl::incoming_interface() {return incoming_interface_;}
InterfacePtr SimControl::outgoing_interface() {return outgoing_interface_;}
std::list<EntityPtr> &SimControl::ents() {return ents_;}
} // namespace scrimmage
