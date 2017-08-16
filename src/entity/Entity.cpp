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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/sensor/Sensable.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <iostream>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using std::cout;
using std::endl;
namespace fs = boost::filesystem;
namespace sp = scrimmage_proto;

namespace scrimmage {

bool Entity::init(AttributeMap &overrides,
                  std::map<std::string, std::string> &info,
                  ContactMapPtr &contacts,
                  MissionParsePtr mp,
                  std::shared_ptr<GeographicLib::LocalCartesian> proj,
                  int id, int sub_swarm_id,
                  PluginManagerPtr plugin_manager,
                  NetworkPtr network,
                  FileSearch &file_search,
                  RTreePtr &rtree) {
    contacts_ = contacts;
    rtree_ = rtree;

    id_.set_id(id);
    id_.set_sub_swarm_id(sub_swarm_id);
    id_.set_team_id(std::stoi(info["team_id"]));

    if (mp == nullptr) {
        mp_ = std::make_shared<MissionParse>();
    } else {
        mp_ = mp;
        parse_visual(info, mp_, file_search);
    }

    // Setup lat/long x/y converter
    proj_ = proj;

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

    double roll = Angles::deg2rad(get("roll", info, 0));
    double pitch = Angles::deg2rad(get("pitch", info, 0));
    double yaw = Angles::deg2rad(get("heading", info, 0));
    state_->quat().set(roll, pitch, yaw);

    EntityPtr parent = shared_from_this();

    ConfigParse config_parse;

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
                    info["motion_model"], file_search, config_parse,
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
                                            sensor_name, file_search,
                                            config_parse,
                                            overrides[sensor_order_name]));

        if (sensor == nullptr) {
            std::cout << "Failed to open sensor plugin: " << sensor_name
                      << std::endl;
            return false;
        }

        sensor->set_parent(parent);
        sensor->init(config_parse.params());
        sensors_[sensor_name] = sensor;

        sensor_order_name = std::string("sensor") + std::to_string(++sensor_ct);
    }

    ////////////////////////////////////////////////////////////
    // sensable
    ////////////////////////////////////////////////////////////
    int sensable_ct = 0;
    std::string sensable_name = std::string("sensable") + std::to_string(sensable_ct);

    while (info.count(sensable_name) > 0) {
        SensablePtr sensable =
            std::dynamic_pointer_cast<Sensable>(
                plugin_manager->make_plugin("scrimmage::Sensable",
                    info[sensable_name], file_search, config_parse,
                    overrides[sensable_name]));

        if (sensable == nullptr) {
            std::cout << "Failed to open sensable plugin: " << info[sensable_name] << std::endl;
            return false;
        }

        sensable->set_parent(parent);
        sensable->set_network(network);
        sensable->init(config_parse.params());
        sensables_[sensable->name()].push_back(sensable);

        sensable_name = std::string("sensable") + std::to_string(++sensable_ct);
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
                    info[autonomy_name], file_search, config_parse,
                    overrides[autonomy_name]));

        if (autonomy == nullptr) {
            cout << "Failed to open autonomy plugin: " << info[autonomy_name] << endl;
            return false;
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

    if (autonomies_.empty()) {
        cout << "Failed to initialize autonomy" << std::endl;
        return false;
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
            std::static_pointer_cast<Controller>(
                plugin_manager->make_plugin("scrimmage::Controller",
                    info[ctrl_name], file_search, config_parse,
                    overrides[ctrl_name]));

        if (controller == nullptr) {
            std::cout << "Failed to open controller plugin: " << info[ctrl_name] << std::endl;
            return false;
        }

        controller->set_state(state_);
        if (ctrl_ct == 0) {
            controller->set_desired_state(autonomies_.front()->desired_state());
        }
        controller->set_parent(parent);
        controller->set_network(network);
        controller->init(config_parse.params());

        controllers_.push_back(controller);

        ctrl_ct++;
        ctrl_name = std::string("controller") + std::to_string(ctrl_ct);
    }

    if (controllers_.empty()) {
        std::cout << "Error: no controllers specified" << std::endl;
        return false;
    }

    return true;
}

bool Entity::parse_visual(std::map<std::string, std::string> &info,
                          MissionParsePtr mp, FileSearch &file_search) {
    visual_->set_id(id_.id());
    visual_->set_opacity(1.0);

    ConfigParse cv_parse;
    bool mesh_found, texture_found;
    find_model_properties(info["visual_model"], cv_parse,
                          file_search, visual_,
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
    // Are all autonomies ready?
    for (AutonomyPtr &a : autonomies_) {
        if (!a->ready()) return false;
    }

    // Are all controllers ready?
    for (ControllerPtr &c : controllers_) {
        if (!c->ready()) return false;
    }

    // Are all sensors ready?
    for (auto &kv : sensors_) {
        // for (SensorPtr &s : kv.second) {
        if (!kv.second->ready()) return false;
            //}
    }

    // Are all sensables ready?
    for (auto &kv : sensables_) {
        for (SensablePtr &s : kv.second) {
            if (!s->ready()) return false;
        }
    }

    // Is the motion model ready?
    if (!motion_model_->ready()) return false;

    // Everything is ready if this point is reached
    return true;
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

void Entity::set_random(RandomPtr random) { random_ = random; }

RandomPtr Entity::random() { return random_; }

Contact::Type Entity::type() { return type_; }

void Entity::set_visual_changed(bool visual_changed)
{ visual_changed_ = visual_changed; }

bool Entity::visual_changed() { return visual_changed_; }

scrimmage_proto::ContactVisualPtr &Entity::contact_visual()
{ return visual_; }

std::unordered_map<std::string, std::list<SensablePtr> > &Entity::sensables() {
    return sensables_;
}

std::unordered_map<std::string, SensorPtr > &Entity::sensors() {
    return sensors_;
}

void Entity::set_active(bool active) { active_ = active; }

bool Entity::active() { return active_; }

void Entity::setup_desired_state() {
    if (controllers_.empty()) {
        return;
    }

    for (AutonomyPtr &autonomy : autonomies_) {
        if (autonomy->get_is_controlling()) {
            controllers_.front()->set_desired_state(autonomy->desired_state());
            break;
        }
    }
}

std::unordered_map<std::string, Service> &Entity::services() {return services_;}

bool Entity::call_service(scrimmage::MessageBasePtr req,
        scrimmage::MessageBasePtr &res, std::string service_name) {

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

} // namespace scrimmage
