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

#include "scrimmage/entity/Entity.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>

#include <boost/algorithm/cxx11/none_of.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/copy.hpp>

#include "scrimmage/autonomy/Autonomy.h"
#include "scrimmage/common/GlobalService.h"
#include "scrimmage/common/Utilities.h"
#include "scrimmage/entity/EntityPluginHelper.h"
#include "scrimmage/gpu/GPUMotionModel.h"
#include "scrimmage/math/Angles.h"
#include "scrimmage/math/State.h"
#include "scrimmage/motion/Controller.h"
#include "scrimmage/motion/MotionModel.h"
#include "scrimmage/parse/ConfigParse.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/PluginManager.h"
#include "scrimmage/log/Logger.h"
#include "scrimmage/proto/ProtoConversions.h"
#include "scrimmage/sensor/Sensor.h"
#include "scrimmage/simcontrol/SimUtils.h"

namespace sp = scrimmage_proto;
namespace br = boost::range;
namespace ba = boost::adaptors;

namespace scrimmage {

bool Entity::init(const SimUtilsInfo& sim_info, EntityInitParams init_params) {
    pubsub_ = sim_info.pubsub;
    global_services_ = sim_info.global_services;
    time_ = sim_info.time;
    file_search_ = sim_info.file_search;
    plugin_manager_ = sim_info.plugin_manager;
    contacts_ = sim_info.contacts;
    rtree_ = sim_info.rtree;
    proj_ = sim_info.proj;
    param_server_ = sim_info.param_server;
    gpu_controller_ = init_params.gpu_controller;
    gpu_motion_model_ = init_params.gpu_motion_model;

    auto mp = sim_info.mp;
    auto id_to_ent_map = sim_info.id_to_ent_map;
    auto id_to_team_map = sim_info.id_to_team_map;
    GPUMotionModelPtr gpu_motion_model = init_params.gpu_motion_model;

    int id = init_params.id;
    int ent_desc_id = init_params.ent_desc_id;
    std::map<std::string, std::string>& info = init_params.info;
    std::set<std::string>& plugin_tags = init_params.plugin_tags;
    const RuntimePluginOverrides& runtime_plugin_overrides = init_params.runtime_plugin_overrides;
    std::function<void(std::map<std::string, std::string>&)> param_override_func =
        init_params.param_override_func;
    int debug_level = init_params.debug_level;

    id_.set_id(id);
    id_.set_sub_swarm_id(ent_desc_id);
    try {
        id_.set_team_id(std::stoi(info["team_id"]));
    } catch (const std::exception& e) {
        LOG_ERROR("Entity " << id << ": failed to parse team_id '" << info["team_id"] << "': " << e.what());
        return false;
    }

    if (mp == nullptr) {
        mp_ = std::make_shared<MissionParse>();
    } else {
        mp_ = mp;
        parse_visual(info, mp_);
    }

    if (info.count("health") > 0) {
        try {
            health_points_ = std::stoi(info["health"]);
        } catch (const std::exception& e) {
            LOG_ERROR("Entity " << id << ": failed to parse health '" << info["health"] << "': " << e.what());
        }
    }

    radius_ = get<double>("radius", info, 1.0);

    ////////////////////////////////////////////////////////////
    // set state
    ////////////////////////////////////////////////////////////
    if (!state_belief_) {
        state_belief_ = std::make_shared<State>();
    }
    state_truth_ = state_belief_;

    double x = get("x", info, 0.0);
    double y = get("y", info, 0.0);
    double z = get("z", info, 0.0);
    state_truth_->pos() << x, y, z;

    double vx = get("vx", info, 0.0);
    double vy = get("vy", info, 0.0);
    double vz = get("vz", info, 0.0);
    state_truth_->vel() << vx, vy, vz;

    double sp = get("speed", info, 0.0);
    if (sp > 0 && vx == 0 && vy == 0 && vz == 0) {
        Eigen::Vector3d relative_vel_vector = Eigen::Vector3d::UnitX() * sp;
        Eigen::Vector3d vel_vector = state_truth_->quat().rotate(relative_vel_vector);
        state_truth_->vel() << vel_vector[0], vel_vector[1], vel_vector[2];
    }

    double roll = Angles::deg2rad(get("roll", info, 0.0));
    double pitch = Angles::deg2rad(get("pitch", info, 0.0));
    double yaw = Angles::deg2rad(get("heading", info, 0.0));
    state_truth_->quat().set(roll, pitch, yaw);

    EntityPtr parent = shared_from_this();

    // Save entity specific params in mp reference for later use
    mp_->entity_params()[id] = info;
    mp_->ent_id_to_block_id()[id] = ent_desc_id;

    ////////////////////////////////////////////////////////////
    // sensor
    ////////////////////////////////////////////////////////////
    auto sensor_plugins = mp_->get_plugins_by_type(ent_desc_id, "sensor");
    for (const auto& plugin_info : sensor_plugins) {
        ConfigParse config_parse;
        std::string sensor_name = plugin_info.name;

        std::map<std::string, std::string> sensor_overrides =
            resolve_plugin_params(plugin_info, runtime_plugin_overrides);

        PluginStatus<Sensor> status = plugin_manager_->make_plugin<Sensor>(
            "scrimmage::Sensor",
            sensor_name,
            *file_search_,
            config_parse,
            sensor_overrides,
            plugin_tags);
        if (status.status == PluginStatus<Sensor>::cast_failed) {
            LOG_ERROR("Failed to open sensor plugin: " << sensor_name);
            return false;
        } else if (status.status == PluginStatus<Sensor>::parse_failed) {
            LOG_ERROR("Failed to parse sensor plugin config: " << sensor_name);
            return false;
        } else if (status.status == PluginStatus<Sensor>::loaded) {
            SensorPtr sensor = status.plugin;

            // Get sensor's offset from entity origin
            std::vector<double> tf_xyz = {0.0, 0.0, 0.0};
            auto it_xyz = sensor_overrides.find("xyz");
            if (it_xyz != sensor_overrides.end()) {
                str2container(it_xyz->second, " ", tf_xyz, 3);
            }
            sensor->transform()->pos() << tf_xyz[0], tf_xyz[1], tf_xyz[2];

            // Get sensor's orientation relative to entity's coordinate frame
            std::vector<double> tf_rpy = {0.0, 0.0, 0.0};
            auto it_rpy = sensor_overrides.find("rpy");
            if (it_rpy != sensor_overrides.end()) {
                str2container(it_rpy->second, " ", tf_rpy, 3);
            }
            sensor->transform()->quat().set(
                Angles::deg2rad(tf_rpy[0]),
                Angles::deg2rad(tf_rpy[1]),
                Angles::deg2rad(tf_rpy[2]));

            sensor->set_parent(parent);
            sensor->set_pubsub(pubsub_);
            sensor->set_time(time_);
            sensor->set_id_to_team_map(id_to_team_map);
            sensor->set_id_to_ent_map(id_to_ent_map);
            sensor->set_param_server(param_server_);
            param_override_func(config_parse.params());

            // get loop rate from plugin's params
            auto it_loop_rate = config_parse.params().find("loop_rate");
            if (it_loop_rate != config_parse.params().end()) {
                const double loop_rate = std::stod(it_loop_rate->second);
                sensor->set_loop_rate(loop_rate);
            }

            std::string given_name = sensor_name + std::to_string(plugin_info.order);
            sensor->set_name(given_name);

            if (debug_level > 1) {
                LOG_INFO("--------------------------------");
                LOG_INFO("Sensor plugin params: " << given_name);
                LOG_INFO(config_parse);
            }
            try {
                sensor->init(config_parse.params());
            } catch (const std::exception& e) {
                LOG_ERROR("Sensor plugin '" << given_name << "' threw exception during init: " << e.what());
                return false;
            } catch (...) {
                LOG_ERROR("Sensor plugin '" << given_name << "' threw unknown exception during init");
                return false;
            }
            sensors_[given_name] = sensor;
        }
    }

    ////////////////////////////////////////////////////////////
    // motion model
    ////////////////////////////////////////////////////////////
    bool use_gpu_motion_model = gpu_motion_model != nullptr;
    bool init_empty_motion_model = true;  // Still init a dummy motion model when using a gpu one.
                                          // Otherwise program may segfault during execution.
    auto motion_plugins = mp_->get_plugins_by_type(ent_desc_id, "motion_model");
    if (!motion_plugins.empty() && !use_gpu_motion_model) {
        // MissionParse enforces singleton motion_model precedence, so front() is the
        // resolved winner after entity-local overrides have replaced any inherited one.
        const auto& motion_info = motion_plugins.front();
        ConfigParse config_parse;
        std::map<std::string, std::string> motion_overrides =
            resolve_plugin_params(motion_info, runtime_plugin_overrides);
        PluginStatus<MotionModel> status = plugin_manager_->make_plugin<MotionModel>(
            "scrimmage::MotionModel",
            motion_info.name,
            *file_search_,
            config_parse,
            motion_overrides,
            plugin_tags);
        if (status.status == PluginStatus<MotionModel>::cast_failed) {
            LOG_ERROR("Failed to open motion model plugin: " << motion_info.name);
            return false;
        } else if (status.status == PluginStatus<MotionModel>::parse_failed) {
            LOG_ERROR("Failed to parse motion model plugin config: " << motion_info.name);
            return false;
        } else if (status.status == PluginStatus<MotionModel>::loaded) {
            // We have created a valid motion model
            init_empty_motion_model = false;

            motion_model_ = status.plugin;
            motion_model_->set_state(state_truth_);
            motion_model_->set_parent(parent);
            motion_model_->set_pubsub(pubsub_);
            motion_model_->set_time(time_);
            motion_model_->set_id_to_team_map(id_to_team_map);
            motion_model_->set_id_to_ent_map(id_to_ent_map);
            motion_model_->set_param_server(param_server_);
            motion_model_->set_name(motion_info.name);
            param_override_func(config_parse.params());

            if (debug_level > 1) {
                LOG_INFO("--------------------------------");
                LOG_INFO("Motion plugin params: " << motion_info.name);
                LOG_INFO(config_parse);
            }
            try {
                motion_model_->init(info, config_parse.params());
            } catch (const std::exception& e) {
                LOG_ERROR("MotionModel plugin '" << motion_info.name << "' threw exception during init: " << e.what());
                return false;
            } catch (...) {
                LOG_ERROR("MotionModel plugin '" << motion_info.name << "' threw unknown exception during init");
                return false;
            }
        }
    } else if (use_gpu_motion_model) {
        gpu_motion_model->add_entity(shared_from_this());
    }
    if (init_empty_motion_model) {
        motion_model_ = std::make_shared<MotionModel>();
        motion_model_->set_state(state_truth_);
        motion_model_->set_parent(parent);
        motion_model_->set_pubsub(pubsub_);
        motion_model_->set_param_server(param_server_);
        motion_model_->set_time(time_);
        motion_model_->set_id_to_team_map(id_to_team_map);
        motion_model_->set_id_to_ent_map(id_to_ent_map);
        motion_model_->set_name("BLANK");
    }

    ////////////////////////////////////////////////////////////
    // controller
    ////////////////////////////////////////////////////////////
    auto controller_plugins = mp_->get_plugins_by_type(ent_desc_id, "controller");

    // Build the controller chain from back to front. The last controller in
    // XML order feeds the motion model, so it must be created first. Each
    // earlier controller then connects to the controller that was just added.
    for (auto rit = controller_plugins.rbegin(); rit != controller_plugins.rend(); ++rit) {
        const auto& controller_info = *rit;

        ConfigParse config_parse;
        std::map<std::string, std::string> controller_overrides =
            resolve_plugin_params(controller_info, runtime_plugin_overrides);
        PluginStatus<Controller> status = plugin_manager_->make_plugin<Controller>(
            "scrimmage::Controller",
            controller_info.name,
            *file_search_,
            config_parse,
            controller_overrides,
            plugin_tags);
        if (status.status == PluginStatus<Controller>::cast_failed) {
            LOG_ERROR("Failed to open controller plugin: " << controller_info.name);
            return false;
        } else if (status.status == PluginStatus<Controller>::parse_failed) {
            LOG_ERROR("Failed to parse controller plugin config: " << controller_info.name);
            return false;
        } else if (status.status == PluginStatus<Controller>::loaded) {
            ControllerPtr controller = status.plugin;

            controller->set_parent(shared_from_this());
            controller->set_time(time_);
            controller->set_id_to_team_map(id_to_team_map);
            controller->set_id_to_ent_map(id_to_ent_map);
            controller->set_param_server(param_server_);
            controller->set_pubsub(pubsub_);
            controller->set_name(controller_info.name);
            param_override_func(config_parse.params());

            // get loop rate from plugin's params
            auto it_loop_rate = config_parse.params().find("loop_rate");
            if (it_loop_rate != config_parse.params().end()) {
                const double loop_rate = std::stod(it_loop_rate->second);
                controller->set_loop_rate(loop_rate);
            }

            // Connect this controller to the motion model if it is the last
            // controller in XML top-down order (i.e., first in the reverse
            // list). If it is not the last controller, connect it to the next
            // controller.
            // If we are using a gpu motoin model, connect the final output to
            // the variableIO provided by the gpu motion model
            bool connect_to_motion_model = (controllers_.size() == 0);
            if (connect_to_motion_model) {
                if (!gpu_motion_model) {
                    connect(controller->vars(), motion_model_->vars());
                } else {
                    VariableIO& vario = gpu_motion_model->get_entity_input(shared_from_this());
                    connect(controller->vars(), vario);
                }
            } else {
                connect(controller->vars(), controllers_.back()->vars());
            }

            // Initialize this controller.
            if (debug_level > 1) {
                LOG_INFO("--------------------------------");
                LOG_INFO("Controller plugin params: " << controller_info.name);
                LOG_INFO(config_parse);
            }
            try {
                controller->init(config_parse.params());
            } catch (const std::exception& e) {
                LOG_ERROR("Controller plugin '" << controller_info.name << "' threw exception during init: " << e.what());
                return false;
            } catch (...) {
                LOG_ERROR("Controller plugin '" << controller_info.name << "' threw unknown exception during init");
                return false;
            }

            if (connect_to_motion_model && gpu_motion_model) {
                controller->vars().create_unconnected_output();
            }

            // Verify the VariableIO connection
            if (connect_to_motion_model) {
                if (!gpu_motion_model
                    && !verify_io_connection(controller->vars(), motion_model_->vars())) {
                    LOG_ERROR("VariableIO Error: " << std::quoted(controller->name())
                              << " does not provide inputs required by motion model "
                              << std::quoted(motion_model_->name()));
                    print_io_error(motion_model_->name(), motion_model_->vars());
                    return false;
                }
            } else if (not connect_to_motion_model) {
                if (!verify_io_connection(controller->vars(), controllers_.back()->vars())) {
                    LOG_ERROR("VariableIO Error: " << std::quoted(controller->name())
                              << " does not provide inputs required by next controller "
                              << std::quoted(controllers_.back()->name()));
                    print_io_error(controllers_.back()->name(), controllers_.back()->vars());
                    return false;
                }
            }

            // Save this controller instance
            controllers_.push_back(controller);
        }
    }

    // Since the controllers_ list was constructed in reverse XML order, we
    // need to reverse it to ensure that the controllers are executed in the
    // correct order
    std::reverse(controllers_.begin(), controllers_.end());

    // If the motion model requires any inputs and there are no controllers,
    // this is a VariableIO error.
    if (motion_model_ != nullptr && motion_model_->vars().input_variable_index().size() > 0
        && controllers_.size() == 0) {
        LOG_ERROR("VariableIO Error: There are not any controllers that "
                  << "provide the inputs required by " << std::quoted(motion_model_->name()));
        print_io_error(motion_model_->name(), motion_model_->vars());
        LOG_INFO("If you want to directly pass the outputs from the "
                  << "autonomy to the motion_model, see the DirectController "
                  << "controller plugin.");
        return false;
    }

    ////////////////////////////////////////////////////////////
    // autonomy
    ////////////////////////////////////////////////////////////
    auto autonomy_plugins = mp_->get_plugins_by_type(ent_desc_id, "autonomy");

    // Create the autonomy plugins
    for (const auto& autonomy_info : autonomy_plugins) {
        auto autonomy = make_autonomy<Autonomy>(
            autonomy_info.name,
            plugin_manager_,
            resolve_plugin_params(autonomy_info, runtime_plugin_overrides),
            parent,
            state_belief_,
            id_to_team_map,
            id_to_ent_map,
            proj_,
            contacts_,
            file_search_,
            rtree_,
            pubsub_,
            time_,
            param_server_,
            plugin_tags,
            param_override_func,
            controllers_,
            debug_level);

        if (autonomy) {
            autonomies_.push_back(*autonomy);
        }
    }

    bool connect_entity = true;
    if (info.count("connect_entity") > 0) {
        try {
            connect_entity = boost::lexical_cast<bool>(info["connect_entity"]);
        } catch (const boost::bad_lexical_cast& e) {
            LOG_ERROR("Entity " << id << ": failed to parse connect_entity '" << info["connect_entity"] << "': " << e.what());
        }
    }

    // Verify that at least one autonomy provides the inputs to the first
    // controller if the first controller requires some VariableIO input.
    if (connect_entity && not controllers_.empty()
        && controllers_.front()->vars().input_variable_index().size() > 0) {
        auto verify_io = [&](auto& autonomy) {
            return verify_io_connection(autonomy->vars(), controllers_.front()->vars());
        };
        if (boost::algorithm::none_of(autonomies_, verify_io)) {
            std::ostringstream autonomy_names_ss;
            for (const auto& a : autonomies_) {
                autonomy_names_ss << a->name() << ", ";
            }
            LOG_ERROR("VariableIO Error: "
                      << "no autonomies provide inputs required by Controller "
                      << std::quoted(controllers_.front()->name())
                      << ". Add VariableIO output declarations in "
                      << autonomy_names_ss.str());
            print_io_error(controllers_.front()->name(), controllers_.front()->vars());
            return false;
        }
    }

    if (not controllers_.empty()) {
        if (autonomies_.empty()) {
            controllers_.front()->set_desired_state(state_belief_);
        } else {
            controllers_.front()->set_desired_state(autonomies_.front()->desired_state());
        }
    }
    return true;
}

bool Entity::parse_visual(
    std::map<std::string, std::string>& info,
    MissionParsePtr mp) {
    visual_->set_id(id_.id());
    visual_->set_opacity(1.0);

    ConfigParse cv_parse;
    std::map<std::string, std::string> model_overrides;
    bool mesh_found, texture_found;
    auto it = info.find("visual_model");
    if (it == info.end()) {
        return true;
    }

    find_model_properties(
        it->second,
        cv_parse,
        *file_search_,
        model_overrides,
        visual_,
        mesh_found,
        texture_found);

    // Set the entity color. Use the team color by default
    std::vector<int> color;
    auto it_color = info.find("color");
    if (it_color != info.end() and str2container(it_color->second, ", ", color, 3)) {
    } else {
        set(color, mp->team_info()[id_.team_id()].color);
    }
    set(visual_->mutable_color(), color[0], color[1], color[2]);

    std::string visual_model = boost::to_upper_copy(info["visual_model"]);
    if (mesh_found) {
        type_ = Contact::Type::MESH;
        visual_->set_visual_mode(
            texture_found ? scrimmage_proto::ContactVisual::TEXTURE
                          : scrimmage_proto::ContactVisual::COLOR);
    } else if (visual_model == std::string("QUADROTOR")) {
        type_ = Contact::Type::QUADROTOR;
        visual_->set_visual_mode(scrimmage_proto::ContactVisual::COLOR);
    } else if (visual_model == std::string("AIRCRAFT")) {
        type_ = Contact::Type::AIRCRAFT;
        visual_->set_visual_mode(scrimmage_proto::ContactVisual::COLOR);
    } else if (visual_model == std::string("SPHERE")) {
        type_ = Contact::Type::SPHERE;
        visual_->set_visual_mode(scrimmage_proto::ContactVisual::COLOR);
    } else {
        type_ = Contact::Type::SPHERE;
        visual_->set_visual_mode(scrimmage_proto::ContactVisual::COLOR);
    }

    return true;
}

bool Entity::ready() {
    auto all_ready = [&](auto& rng, auto& func) {
        return std::all_of(rng.begin(), rng.end(), func);
    };

    auto single_ready = [&](auto& plugin) { return plugin->ready(); };
    auto values_single_ready = [&](auto& kv) { return kv.second->ready(); };

    bool autonomies_ready = all_ready(autonomies_, single_ready);
    bool controllers_ready = all_ready(controllers_, single_ready);
    bool sensors_ready = all_ready(sensors_, values_single_ready);
    bool motion_model_ready = motion_model_ == nullptr || motion_model_->ready();

    return autonomies_ready && controllers_ready && sensors_ready && motion_model_ready;
}

StatePtr& Entity::state() {
    return state_belief_;
}

void Entity::set_state_belief(const StatePtr& other) {
    if (state_belief_ == state_truth_) {
        LOG_WARN("Decoupling State Belief and State Truth. Ensure that you "
                     "have an explicit "
                  << "method of updating the state belief.");
        state_belief_ = std::make_shared<State>();
    }
    *state_belief_ = *other;
}

void Entity::set_state_belief(const State& other) {
    if (state_belief_ == state_truth_) {
        LOG_WARN("Decoupling State Belief and State Truth. Ensure that you "
                     "have an explicit "
                  << "method of updating the state belief.");
        state_belief_ = std::make_shared<State>();
    }
    *state_belief_ = other;
}

const std::shared_ptr<const State> Entity::state_belief() const {
    return state_belief_;
}
StatePtr& Entity::state_truth() {
    return state_truth_;
}

std::vector<AutonomyPtr>& Entity::autonomies() {
    return autonomies_;
}

MotionModelPtr& Entity::motion() {
    return motion_model_;
}

std::vector<ControllerPtr>& Entity::controllers() {
    return controllers_;
}

void Entity::set_id(ID& id) {
    id_ = id;
}

ID& Entity::id() {
    return id_;
}

void Entity::collision() {
    health_points_ -= 1e9;
}

void Entity::hit() {
    health_points_--;
}

void Entity::set_health_points(int health_points) {
    health_points_ = health_points;
}

int Entity::health_points() const {
    return health_points_;
}

bool Entity::is_alive() {
    return (health_points_ > 0);
}

bool Entity::posthumous(double t) {
    bool any_autonomies = std::any_of(autonomies_.begin(), autonomies_.end(), [t](AutonomyPtr& a) {
        return a->posthumous(t);
    });
    return any_autonomies && motion_model_->posthumous(t);
}

std::shared_ptr<GeographicLib::LocalCartesian> Entity::projection() {
    return proj_;
}

MissionParsePtr Entity::mp() {
    return mp_;
}

void Entity::set_mp(MissionParsePtr mp) {
    mp_ = mp;
}
void Entity::set_random(RandomPtr random) {
    random_ = random;
}

RandomPtr Entity::random() {
    return random_;
}

Contact::Type Entity::type() {
    return type_;
}

void Entity::set_visual_changed(bool visual_changed) {
    visual_changed_ = visual_changed;
}

bool Entity::visual_changed() {
    return visual_changed_;
}

scrimmage_proto::ContactVisualPtr& Entity::contact_visual() {
    return visual_;
}

std::unordered_map<std::string, SensorPtr>& Entity::sensors() {
    return sensors_;
}

std::unordered_map<std::string, SensorPtr> Entity::sensors(const std::string& sensor_name) {
    std::unordered_map<std::string, SensorPtr> out;
    for (auto& kv : sensors_) {
        if (kv.first.find(sensor_name) != std::string::npos) {
            out[kv.first] = kv.second;
        }
    }
    return out;
}

bool Entity::using_gpu_motion_model() const {
    return gpu_motion_model_ != nullptr;
}

SensorPtr Entity::sensor(const std::string& sensor_name) {
    std::unordered_map<std::string, SensorPtr> out = sensors(sensor_name);
    return out.empty() ? nullptr : out.begin()->second;
}

void Entity::set_active(bool active) {
    active_ = active;
}

bool Entity::active() {
    return active_;
}

void Entity::setup_desired_state() {
    if (controllers_.empty())
        return;

    auto it = std::find_if(autonomies_.rbegin(), autonomies_.rend(), [&](auto autonomy) {
        return autonomy->get_is_controlling();
    });

    if (it != autonomies_.rend()) {
        controllers_.front()->set_desired_state((*it)->desired_state());
    }
}

std::unordered_map<std::string, Service>& Entity::services() {
    return services_;
}
std::unordered_map<std::string, Service>& Entity::global_services() {
    return global_services_->services();
}

void Entity::set_global_services(const GlobalServicePtr& global_services) {
    global_services_ = global_services;
}

bool Entity::call_service(
    scrimmage::MessageBasePtr req,
    scrimmage::MessageBasePtr& res,
    const std::string& service_name) {
    auto it = services_.find(service_name);
    if (it == services_.end()) {
        // First check for a global service of this name
        bool found = global_services_->call_service(req, res, service_name);
        if (!found) {
            std::ostringstream services_ss;
            for (auto& kv : services_) {
                services_ss << kv.first << ", ";
            }
            LOG_WARN("request for service (" << service_name << ") that does not exist. "
                      << "services are: " << services_ss.str());
            return false;
        } else {
            return true;
        }
    }

    Service& service = it->second;
    bool success = service(req, res);

    if (!success) {
        LOG_WARN("call to " << service_name << " failed");
        return false;
    } else {
        return true;
    }
}

void Entity::print(const std::string& msg) {
    LOG_INFO(msg);
}

void Entity::close(double t) {
    for (AutonomyPtr autonomy : autonomies_) {
        autonomy->close_plugin(t);
    }

    for (auto& kv : sensors_) {
        kv.second->close_plugin(t);
    }

    for (ControllerPtr controller : controllers_) {
        controller->close_plugin(t);
    }

    if (motion_model_) {
        motion_model_->close_plugin(t);
    }

    visual_ = nullptr;
    controllers_.clear();
    autonomies_.clear();
    mp_ = nullptr;
    proj_ = nullptr;
    random_ = nullptr;
    state_belief_ = nullptr;
    state_truth_ = nullptr;
    properties_.clear();
    sensors_.clear();
    services_.clear();
    contacts_ = nullptr;
    rtree_ = nullptr;
    plugin_manager_ = nullptr;
    file_search_ = nullptr;
    pubsub_ = nullptr;
    global_services_ = nullptr;
    time_ = nullptr;
}

std::unordered_map<std::string, MessageBasePtr>& Entity::properties() {
    return properties_;
}

void Entity::set_time_ptr(TimePtr t) {
    time_ = t;
}

void Entity::set_gpu_controller(GPUControllerPtr gpu_controller) {
    gpu_controller_ = gpu_controller;
}

GPUControllerPtr Entity::gpu_controller() {
    return gpu_controller_;
}

// cppcheck-suppress passedByValue
void Entity::set_projection(const std::shared_ptr<GeographicLib::LocalCartesian>& proj) {
    proj_ = proj;
}

void Entity::print_plugins(std::ostream& out) const {
    out << "----------- Sensor -------------" << std::endl;
    for (auto& kv : sensors_) {
        out << kv.second->name() << std::endl;
    }
    out << "---------- Autonomy ------------" << std::endl;
    for (AutonomyPtr a : autonomies_) {
        out << a->name() << std::endl;
    }
    out << "---------- Controller ----------" << std::endl;
    for (ControllerPtr c : controllers_) {
        out << c->name() << std::endl;
    }
    out << "----------- Motion -------------" << std::endl;
    if (motion_model_->name() != "BLANK") {
        out << motion_model_->name() << std::endl;
    }
}

}  // namespace scrimmage
