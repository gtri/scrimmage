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
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/sensor/Sensable.h>

#include <iostream>
#include <memory>
#include <algorithm>

#include <boost/algorithm/string/case_conv.hpp>

using std::cout;
using std::endl;

namespace sp = scrimmage_proto;

namespace scrimmage {

bool Entity::init(AttributeMap &overrides,
                  std::map<std::string, std::string> &info,
                  ContactMapPtr &contacts,
                  MissionParsePtr mp,
                  const std::shared_ptr<GeographicLib::LocalCartesian> &proj,
                  int id, int ent_desc_id,
                  PluginManagerPtr plugin_manager,
                  NetworkPtr network,
                  const FileSearchPtr &file_search,
                  RTreePtr &rtree) {

    file_search_ = file_search;
    plugin_manager_ = plugin_manager;
    contacts_ = contacts;
    rtree_ = rtree;
    proj_ = proj;

    id_.set_id(id);
    id_.set_sub_swarm_id(ent_desc_id);
    id_.set_team_id(std::stoi(info["team_id"]));

    if (mp == nullptr) {
        mp_ = std::make_shared<MissionParse>();
    } else {
        mp_ = mp;
        parse_visual(info, mp_, overrides["visual_model"]);
    }

    network_ = network;

    if (info.count("health") > 0) {
        health_points_ = std::stoi(info["health"]);
    }

    radius_ = get<double>("radius", info, 1.0);

    ////////////////////////////////////////////////////////////
    // set state
    ////////////////////////////////////////////////////////////
    state_ = std::make_shared<State>();

    double x = get("x", info, 0.0);
    double y = get("y", info, 0.0);
    double z = get("z", info, 0.0);
    state_->pos() << x, y, z;

    double vx = get("vx", info, 0.0);
    double vy = get("vy", info, 0.0);
    double vz = get("vz", info, 0.0);
    state_->vel() << vx, vy, vz;

    double roll = Angles::deg2rad(get("roll", info, 0.0));
    double pitch = Angles::deg2rad(get("pitch", info, 0.0));
    double yaw = Angles::deg2rad(get("heading", info, 0.0));
    state_->quat().set(roll, pitch, yaw);

    EntityPtr parent = shared_from_this();

    ConfigParse config_parse;

    // Save entity specific params in mp reference for later use
    mp_->entity_params()[id] = info;
    mp_->ent_id_to_block_id()[id] = ent_desc_id;

    ////////////////////////////////////////////////////////////
    // motion model
    ////////////////////////////////////////////////////////////
    if (info.count("motion_model") == 0) {
        motion_model_ = std::make_shared<MotionModel>();
        motion_model_->set_state(state_);
        motion_model_->set_parent(parent);
        motion_model_->set_network(network);
        // cout << "Warning: Missing motion model tag, initializing with base class" << endl;
    } else {
        motion_model_ =
            std::dynamic_pointer_cast<MotionModel>(
                plugin_manager->make_plugin("scrimmage::MotionModel",
                    info["motion_model"], *file_search, config_parse,
                    overrides["motion_model"]));

        if (motion_model_ == nullptr) {
            cout << "Failed to open motion model plugin: " << info["motion_model"] << endl;
            return false;
        }

        motion_model_->set_state(state_);
        motion_model_->set_parent(parent);
        motion_model_->set_network(network);
        motion_model_->init(info, config_parse.params());
    }

    ////////////////////////////////////////////////////////////
    // sensor
    ////////////////////////////////////////////////////////////
    // The MissionParser appends the order number to the sensor (e.g., sensor0,
    // sensor1, etc.)
    int sensor_ct = 0;
    std::string sensor_order_name = std::string("sensor") +
        std::to_string(sensor_ct);

    while (info.count(sensor_order_name) > 0) {
        std::string sensor_name = info[sensor_order_name];
        SensorPtr sensor =
            std::dynamic_pointer_cast<Sensor>(
                plugin_manager->make_plugin("scrimmage::Sensor",
                                            sensor_name, *file_search,
                                            config_parse,
                                            overrides[sensor_order_name]));

        if (sensor == nullptr) {
            std::cout << "Failed to open sensor plugin: " << sensor_name
                      << std::endl;
            return false;
        }

        // Get sensor's offset from entity origin
        std::vector<double> tf_xyz = {0.0, 0.0, 0.0};
        auto it_xyz = overrides[sensor_order_name].find("xyz");
        if (it_xyz != overrides[sensor_order_name].end()) {
            str2vec(it_xyz->second, " ", tf_xyz, 3);
        }
        sensor->transform()->pos() << tf_xyz[0], tf_xyz[1], tf_xyz[2];

        // Get sensor's orientation relative to entity's coordinate frame
        std::vector<double> tf_rpy = {0.0, 0.0, 0.0};
        auto it_rpy = overrides[sensor_order_name].find("rpy");
        if (it_rpy != overrides[sensor_order_name].end()) {
            str2vec(it_rpy->second, " ", tf_rpy, 3);
        }
        sensor->transform()->quat().set(Angles::deg2rad(tf_rpy[0]),
                                 Angles::deg2rad(tf_rpy[1]),
                                 Angles::deg2rad(tf_rpy[2]));

        sensor->set_parent(parent);
        sensor->init(config_parse.params());
        sensors_[sensor_name + std::to_string(sensor_ct)] = sensor;

        sensor_order_name = std::string("sensor") + std::to_string(++sensor_ct);
    }

    ////////////////////////////////////////////////////////////
    // controller
    ////////////////////////////////////////////////////////////
    int ctrl_ct = 0;
    std::string ctrl_name = std::string("controller") + std::to_string(ctrl_ct);
    if (info.count(ctrl_name) == 0) {
        info[ctrl_name] = "SimpleAircraftControllerPID";
    }

    while (info.count(ctrl_name) > 0) {

        ControllerPtr controller =
            init_controller(info[ctrl_name], overrides[ctrl_name], motion_model_->vars());

        if (controller == nullptr) {
            std::cout << "Failed to open controller plugin: " << info[ctrl_name] << std::endl;
            return false;
        }

        controllers_.push_back(controller);
        ctrl_ct++;
        ctrl_name = std::string("controller") + std::to_string(ctrl_ct);
    }

    if (controllers_.empty()) {
        std::cout << "Error: no controllers specified" << std::endl;
        return false;
    }

    ////////////////////////////////////////////////////////////
    // autonomy
    ////////////////////////////////////////////////////////////
    int autonomy_ct = 0;
    std::string autonomy_name = std::string("autonomy") + std::to_string(autonomy_ct);

    while (info.count(autonomy_name) > 0) {
        AutonomyPtr autonomy =
            std::dynamic_pointer_cast<Autonomy>(
                plugin_manager->make_plugin("scrimmage::Autonomy",
                                            info[autonomy_name], *file_search, config_parse,
                                            overrides[autonomy_name]));

        if (autonomy == nullptr) {
            cout << "Failed to open autonomy plugin: " << info[autonomy_name] << endl;
            return false;
        }

        if (controllers_.size() > 0) {
            autonomy->vars().output_variable_index() = controllers_.front()->vars().input_variable_index();
            connect(autonomy->vars(), controllers_.front()->vars());
        }

        autonomy->set_rtree(rtree);
        autonomy->set_parent(parent);
        autonomy->set_projection(proj_);
        autonomy->set_network(network);
        autonomy->set_state(motion_model_->state());
        autonomy->set_contacts(contacts);
        autonomy->set_is_controlling(true);
        autonomy->init(config_parse.params());

        autonomies_.push_back(autonomy);
        autonomy_name = std::string("autonomy") + std::to_string(++autonomy_ct);
    }

    if (controllers_.size() > 0) {
        if (autonomies_.empty()) {
            controllers_.front()->set_desired_state(state_);
        } else {
            controllers_.front()->set_desired_state(autonomies_.front()->desired_state());
        }
    }

    return true;
}

bool Entity::parse_visual(std::map<std::string, std::string> &info,
                          MissionParsePtr mp,
                          std::map<std::string, std::string> &overrides) {
    visual_->set_id(id_.id());
    visual_->set_opacity(1.0);

    ConfigParse cv_parse;
    bool mesh_found, texture_found;
    auto it = info.find("visual_model");
    if (it == info.end()) {
        return true;
    }

    find_model_properties(it->second, cv_parse,
                          *file_search_, overrides, visual_,
                          mesh_found, texture_found);

    set(visual_->mutable_color(), mp->team_info()[id_.team_id()].color);

    std::string visual_model = boost::to_upper_copy(info["visual_model"]);

    if (mesh_found) {
        type_ = Contact::Type::MESH;
        visual_->set_visual_mode(texture_found ? scrimmage_proto::ContactVisual::TEXTURE : scrimmage_proto::ContactVisual::COLOR);
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
    auto all_ready = [&](auto &rng, auto &func) {
        return std::all_of(rng.begin(), rng.end(), func);
    };

    auto single_ready = [&](auto &plugin) {return plugin->ready();};
    auto values_single_ready = [&](auto &kv) {return kv.second->ready();};

    return all_ready(autonomies_, single_ready)
        && all_ready(controllers_, single_ready)
        && all_ready(sensors_, values_single_ready)
        && motion_model_->ready();
}

StatePtr &Entity::state() {return state_;}

std::vector<AutonomyPtr> &Entity::autonomies() {return autonomies_;}

MotionModelPtr &Entity::motion() {return motion_model_;}

std::vector<ControllerPtr> &Entity::controllers() {return controllers_;}

void Entity::set_id(ID &id) { id_ = id; }

ID &Entity::id() { return id_; }

void Entity::collision() { health_points_ -= 1e9; }

void Entity::hit() { health_points_--; }

void Entity::set_health_points(int health_points)
{ health_points_ = health_points; }

int Entity::health_points() { return health_points_; }

bool Entity::is_alive() {
    return (health_points_ > 0);
}

bool Entity::posthumous(double t) {
    bool any_autonomies =
        std::any_of(autonomies_.begin(), autonomies_.end(),
                    [t](AutonomyPtr &a) {return a->posthumous(t);});
    return any_autonomies && motion_model_->posthumous(t);
}

std::shared_ptr<GeographicLib::LocalCartesian> Entity::projection()
{ return proj_; }

MissionParsePtr Entity::mp() { return mp_; }

NetworkPtr Entity::network() { return network_; }

void Entity::set_random(RandomPtr random) { random_ = random; }

RandomPtr Entity::random() { return random_; }

Contact::Type Entity::type() { return type_; }

void Entity::set_visual_changed(bool visual_changed)
{ visual_changed_ = visual_changed; }

bool Entity::visual_changed() { return visual_changed_; }

scrimmage_proto::ContactVisualPtr &Entity::contact_visual()
{ return visual_; }

std::unordered_map<std::string, SensorPtr> &Entity::sensors() {
    return sensors_;
}

std::unordered_map<std::string, SensorPtr> Entity::sensors(const std::string &sensor_name) {
    std::unordered_map<std::string, SensorPtr> out;
    for (auto &kv : sensors_) {
        if (kv.first.find(sensor_name) != std::string::npos) {
            out[kv.first] = kv.second;
        }
    }
    return out;
}

SensorPtr Entity::sensor(const std::string &sensor_name) {
    std::unordered_map<std::string, SensorPtr> out = sensors(sensor_name);
    return out.empty() ? nullptr : out.begin()->second;
}

void Entity::set_active(bool active) { active_ = active; }

bool Entity::active() { return active_; }

void Entity::setup_desired_state() {
    if (controllers_.empty()) {
        return;
    }

    auto it = std::find_if(autonomies_.rbegin(), autonomies_.rend(),
        [&](auto autonomy) {return autonomy->get_is_controlling();});

    if (it != autonomies_.rend()) {
        controllers_.front()->set_desired_state((*it)->desired_state());
    }
}

std::unordered_map<std::string, Service> &Entity::services() {return services_;}

bool Entity::call_service(scrimmage::MessageBasePtr req,
        scrimmage::MessageBasePtr &res, const std::string &service_name) {

    auto it = services_.find(service_name);
    if (it == services_.end()) {
        std::cout << "request for service ("
            << service_name
            << ") that does not exist" << std::endl;
        std::cout << "services are: ";
        for (auto &kv : services_) {
            std::cout << kv.first << ", ";
        }
        std::cout << std::endl;
        return false;
    }

    Service &service = it->second;
    bool success = service(req, res);

    if (!success) {
        std::cout << "call to " << service_name << " failed" << std::endl;
        return false;
    } else {
        return true;
    }
}

void Entity::print(const std::string &msg) {
    std::cout << msg << std::endl;
}

void Entity::close(double t) {
    for (AutonomyPtr autonomy : autonomies_) {
        autonomy->close(t);
    }

    for (auto &kv : sensors_) {
        kv.second->close(t);
    }

    for (ControllerPtr ctrl : controllers_) {
        ctrl->close(t);
    }

    motion_model_->close(t);
}

std::unordered_map<std::string, MessageBasePtr> &Entity::properties() {
    return properties_;
}

ControllerPtr Entity::init_controller(
        const std::string &name,
        std::map<std::string, std::string> &overrides,
        VariableIO &next_io) {

    ConfigParse config_parse;
    ControllerPtr controller =
        std::static_pointer_cast<Controller>(
            plugin_manager_->make_plugin("scrimmage::Controller",
                name, *file_search_, config_parse, overrides));

    if (controller == nullptr) {
        std::cout << "Failed to open controller plugin: " << name << std::endl;
        return nullptr;
    }

    controller->set_state(state_);

    // The controller's variable indices should be the same as the motion
    // model's variable indices. This will speed up copying during the
    // simulation.
    controller->vars().output_variable_index() = next_io.input_variable_index();
    connect(controller->vars(), next_io);

    controller->set_parent(shared_from_this());
    controller->set_network(network_);
    controller->init(config_parse.params());
    return controller;
}
} // namespace scrimmage
