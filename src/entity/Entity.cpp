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
#include <scrimmage/simcontrol/SimUtils.h>

#include <iostream>
#include <iomanip>
#include <memory>
#include <algorithm>

#include <boost/algorithm/cxx11/none_of.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/lexical_cast.hpp>

using std::cout;
using std::endl;

namespace sp = scrimmage_proto;
namespace br = boost::range;
namespace ba = boost::adaptors;

namespace scrimmage {

bool Entity::init(AttributeMap &overrides,
                  std::map<std::string, std::string> &info,
                  ContactMapPtr &contacts,
                  MissionParsePtr mp,
                  const std::shared_ptr<GeographicLib::LocalCartesian> &proj,
                  int id, int ent_desc_id,
                  PluginManagerPtr plugin_manager,
                  FileSearchPtr &file_search,
                  RTreePtr &rtree,
                  PubSubPtr &pubsub,
                  TimePtr &time,
                  const ParameterServerPtr &param_server,
                  const std::set<std::string> &plugin_tags,
                  std::function<void(std::map<std::string, std::string>&)> param_override_func) {

    pubsub_ = pubsub;
    time_ = time;
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

    // Save entity specific params in mp reference for later use
    mp_->entity_params()[id] = info;
    mp_->ent_id_to_block_id()[id] = ent_desc_id;

    ////////////////////////////////////////////////////////////
    // motion model
    ////////////////////////////////////////////////////////////
    bool init_empty_motion_model = true;
    if (info.count("motion_model") > 0) {
        ConfigParse config_parse;
        PluginStatus<MotionModel> status =
            plugin_manager->make_plugin<MotionModel>("scrimmage::MotionModel",
                                                     info["motion_model"],
                                                     *file_search,
                                                     config_parse,
                                                     overrides["motion_model"],
                                                     plugin_tags);
        if (status.status == PluginStatus<MotionModel>::cast_failed) {
            cout << "Failed to open motion model plugin: " << info["motion_model"] << endl;
            return false;
        } else if (status.status == PluginStatus<MotionModel>::loaded) {
            // We have created a valid motion model
            init_empty_motion_model = false;

            motion_model_ = status.plugin;
            motion_model_->set_state(state_);
            motion_model_->set_parent(parent);
            motion_model_->set_pubsub(pubsub);
            motion_model_->set_time(time);
            motion_model_->set_param_server(param_server);
            motion_model_->set_name(info["motion_model"]);
            param_override_func(config_parse.params());
            motion_model_->init(info, config_parse.params());
        }
    }

    if (init_empty_motion_model) {
        motion_model_ = std::make_shared<MotionModel>();
        motion_model_->set_state(state_);
        motion_model_->set_parent(parent);
        motion_model_->set_pubsub(pubsub);
        motion_model_->set_param_server(param_server);
        motion_model_->set_time(time);
        motion_model_->set_name("BLANK");
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
        ConfigParse config_parse;
        std::string sensor_name = info[sensor_order_name];
        PluginStatus<Sensor> status =
            plugin_manager->make_plugin<Sensor>("scrimmage::Sensor",
                                                sensor_name, *file_search,
                                                config_parse,
                                                overrides[sensor_order_name],
                                                plugin_tags);
        if (status.status == PluginStatus<Sensor>::cast_failed) {
            std::cout << "Failed to open sensor plugin: " << sensor_name
                      << std::endl;
            return false;
        } else if (status.status == PluginStatus<Sensor>::loaded) {
            SensorPtr sensor = status.plugin;

            // Get sensor's offset from entity origin
            std::vector<double> tf_xyz = {0.0, 0.0, 0.0};
            auto it_xyz = overrides[sensor_order_name].find("xyz");
            if (it_xyz != overrides[sensor_order_name].end()) {
                str2container(it_xyz->second, " ", tf_xyz, 3);
            }
            sensor->transform()->pos() << tf_xyz[0], tf_xyz[1], tf_xyz[2];

            // Get sensor's orientation relative to entity's coordinate frame
            std::vector<double> tf_rpy = {0.0, 0.0, 0.0};
            auto it_rpy = overrides[sensor_order_name].find("rpy");
            if (it_rpy != overrides[sensor_order_name].end()) {
                str2container(it_rpy->second, " ", tf_rpy, 3);
            }
            sensor->transform()->quat().set(Angles::deg2rad(tf_rpy[0]),
                                            Angles::deg2rad(tf_rpy[1]),
                                            Angles::deg2rad(tf_rpy[2]));

            sensor->set_parent(parent);
            sensor->set_pubsub(pubsub);
            sensor->set_time(time);
            sensor->set_param_server(param_server);
            sensor->set_name(sensor_name);
            param_override_func(config_parse.params());
            sensor->init(config_parse.params());
            sensors_[sensor_name + std::to_string(sensor_ct)] = sensor;
        }
        sensor_order_name = std::string("sensor") + std::to_string(++sensor_ct);
    }

    ////////////////////////////////////////////////////////////
    // controller
    ////////////////////////////////////////////////////////////
    int controller_ct = 0;
    std::string controller_name = std::string("controller") + std::to_string(controller_ct);

    while (info.count(controller_name) > 0) {
        ConfigParse config_parse;
        PluginStatus<Controller> status =
            plugin_manager_->make_plugin<Controller>("scrimmage::Controller",
                                                     info[controller_name],
                                                     *file_search,
                                                     config_parse,
                                                     overrides[controller_name],
                                                     plugin_tags);
        if (status.status == PluginStatus<Controller>::cast_failed) {
            std::cout << "Failed to open controller plugin: "
                      << controller_name << std::endl;
            return false;
        } else if (status.status == PluginStatus<Controller>::loaded) {
            ControllerPtr controller = status.plugin;
            controller->set_state(state_);

            // The controller's variable indices should be the same as the motion
            // model's variable indices. This will speed up copying during the
            // simulation.
            connect(controller->vars(), motion_model_->vars());

            controller->set_parent(shared_from_this());
            controller->set_time(time_);
            controller->set_param_server(param_server);
            controller->set_pubsub(pubsub_);
            controller->set_name(info[controller_name]);
            param_override_func(config_parse.params());
            controller->init(config_parse.params());
            controllers_.push_back(controller);
        }
        controller_name = std::string("controller") + std::to_string(++controller_ct);
    }

    // Configure the controller variableIO's, such that they run in series
    for (std::vector<ControllerPtr>::iterator it = controllers_.begin();
         it != controllers_.end(); ++it) {
        auto next = std::next(it);
        if (next == controllers_.end()) {
            // Last controller's variables connect to the motion model
            // Note that we do not need to make the connection if there is no
            // motion model. This would needlessly clear the IO of the
            // controller
            if (init_empty_motion_model) continue;

            connect((*it)->vars(), motion_model_->vars());

            // Make sure that the last controller provides the variables
            // required by the motion model
            if (!verify_io_connection((*it)->vars(), motion_model_->vars())) {
                std::cout << "VariableIO Error: "
                          << std::quoted((*it)->name())
                          << " does not provide inputs required by MotionModel "
                          << std::quoted(motion_model_->name())
                          << ": ";
                print_io_error(motion_model_->name(), motion_model_->vars());
                return false;
            }
        } else {
            // Controller's output variables connect to the next controller's
            // input variableIO
            connect((*it)->vars(), (*next)->vars());

            if (!verify_io_connection((*it)->vars(), (*next)->vars())) {
                std::cout << "VariableIO Error: "
                          << std::quoted((*it)->name())
                          << " does not provide inputs required by next controller "
                          << std::quoted((*next)->name())
                          << ": ";
                print_io_error((*next)->name(), (*next)->vars());
                return false;
            }
        }
    }

    ////////////////////////////////////////////////////////////
    // autonomy
    ////////////////////////////////////////////////////////////
    int autonomy_ct = 0;
    std::string autonomy_name = std::string("autonomy") + std::to_string(autonomy_ct);

    while (info.count(autonomy_name) > 0) {
        ConfigParse config_parse;
        PluginStatus<Autonomy> status =
            plugin_manager->make_plugin<Autonomy>("scrimmage::Autonomy",
                                                  info[autonomy_name],
                                                  *file_search,
                                                  config_parse,
                                                  overrides[autonomy_name],
                                                  plugin_tags);
        if (status.status == PluginStatus<Autonomy>::cast_failed) {
            cout << "Failed to open autonomy plugin: " << info[autonomy_name] << endl;
            return false;
        } else if (status.status == PluginStatus<Autonomy>::loaded) {
            AutonomyPtr autonomy = status.plugin;
            // Connect the autonomy to the first controller
            if (not controllers_.empty()) {
                connect(autonomy->vars(), controllers_.front()->vars());
            }
            autonomy->set_rtree(rtree);
            autonomy->set_parent(parent);
            autonomy->set_projection(proj_);
            autonomy->set_pubsub(pubsub);
            autonomy->set_time(time);
            autonomy->set_param_server(param_server);
            autonomy->set_state(motion_model_->state());
            autonomy->set_contacts(contacts);
            autonomy->set_is_controlling(true);
            autonomy->set_name(info[autonomy_name]);
            param_override_func(config_parse.params());
            autonomy->init(config_parse.params());

            autonomies_.push_back(autonomy);
        }
        autonomy_name = std::string("autonomy") + std::to_string(++autonomy_ct);
    }

    bool connect_entity = true;
    if (info.count("connect_entity") > 0) {
        connect_entity = boost::lexical_cast<bool>(info["connect_entity"]);
    }

    if (connect_entity && not controllers_.empty()) {
        auto verify_io = [&](auto &p) {return verify_io_connection(p->vars(), controllers_.front()->vars());};
        if (boost::algorithm::none_of(autonomies_, verify_io)) {
            auto out_it = std::ostream_iterator<std::string>(std::cout, ", ");
            std::cout << "VariableIO Error: "
                      << "no autonomies provide inputs required by Controller "
                      << std::quoted(controllers_.front()->name())
                      << ". Add VariableIO output declarations in ";
            auto get_name = [&](auto &p) {return p->name();};
            br::copy(autonomies_ | ba::transformed(get_name), out_it);
            std::cout << "as follows " << std::endl;

            print_io_error(controllers_.front()->name(), controllers_.front()->vars());
            return false;
        }
    }

    if (not controllers_.empty()) {
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

std::vector<ControllerPtr> &Entity::controllers() {
    return controllers_;
}

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

void Entity::set_mp(MissionParsePtr mp) { mp_ = mp; }
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
    if (controllers_.empty()) return;

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
        autonomy->close_plugin(t);
    }

    for (auto &kv : sensors_) {
        kv.second->close_plugin(t);
    }

    for (ControllerPtr controller: controllers_) {
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
    state_ = nullptr;
    properties_.clear();
    sensors_.clear();
    services_.clear();
    contacts_ = nullptr;
    rtree_ = nullptr;
    plugin_manager_ = nullptr;
    file_search_ = nullptr;
    pubsub_ = nullptr;
    time_ = nullptr;
}

std::unordered_map<std::string, MessageBasePtr> &Entity::properties() {
    return properties_;
}

void Entity::set_time_ptr(TimePtr t) {time_ = t;}

// cppcheck-suppress passedByValue
void Entity::set_projection(std::shared_ptr<GeographicLib::LocalCartesian> proj) {
    proj_ = proj;
}

void Entity::print_plugins(std::ostream &out) const {
    out << "----------- Sensor -------------" << endl;
    for (auto &kv : sensors_) {
        out << kv.second->name() << endl;
    }
    out << "---------- Autonomy ------------" << endl;
    for (AutonomyPtr a : autonomies_) {
        out << a->name() << endl;
    }
    out << "---------- Controller ----------" << endl;
    for (ControllerPtr c : controllers_) {
        out << c->name() << endl;
    }
    out << "----------- Motion -------------" << endl;
    if (motion_model_->name() != "BLANK") {
        out << motion_model_->name() << endl;
    }
}
} // namespace scrimmage
