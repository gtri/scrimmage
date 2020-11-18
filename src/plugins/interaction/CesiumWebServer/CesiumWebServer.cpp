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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/CesiumWebServer/CesiumWebServer.h>
#include <scrimmage/plugins/interaction/CesiumWebServer/json.hpp>

#include <GeographicLib/LocalCartesian.hpp>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>

#include <string>
#include <functional>

using namespace seasocks;
using json = nlohmann::json;

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
        scrimmage::interaction::CesiumWebServer,
        CesiumWebServer_plugin)

namespace scrimmage {
namespace interaction {

CesiumWebServer::CesiumWebServer() {
}

CesiumWebServer::~CesiumWebServer() {
  server_->terminate();
  serve_thread_->join();
}

bool CesiumWebServer::init(std::map<std::string, std::string> &mission_params,
                 std::map<std::string, std::string> &plugin_params) {

  // params
  web_src_dir_ = sc::get<std::string>("web_src_dir", plugin_params, "");
  web_host_port_ = sc::get<int>("web_host_port", plugin_params, 9090);

  // subscribe to scrimmage shapes
  subscribe<scrimmage_proto::ShapePtr>("GlobalNetwork", "shapes",
      std::bind(&CesiumWebServer::subShapes, this, std::placeholders::_1));

  // setup web server
  auto logger = std::make_shared<PrintfLogger>(Logger::Level::Debug);
  server_ = std::make_shared<seasocks::Server>(logger);
  ws_handler_ = std::make_shared<CesiumWebSocketHandler>(server_.get());
  server_->addWebSocketHandler("/ws", ws_handler_);

  // spawn thread to serve page
  serve_thread_ = std::make_shared<std::thread>(
      std::bind(&CesiumWebServer::serve_blocking, this));

  return true;
}


bool CesiumWebServer::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                          double t, double dt) {
  /* if (ents.empty()) { */
  /*   return true; */
  /* } */
  ents_ = ents;
  sendEntities();

  return true;
}

void CesiumWebServer::sendEntities() {
  for (auto entity : ents_) {
    const auto pos = entity->state_truth()->pos();
    const auto quat = entity->state_truth()->quat();
    json entity_json, msg;
    std::string full_id = "entity" + std::to_string(entity->id().id());

    // convert to global coordinates
    double lat, lon, alt;
    parent_->projection()->Reverse(pos[0], pos[1], pos[2], lat, lon, alt);
    entity_json["id"] = full_id;
    entity_json["name"] = full_id;
    entity_json["position"]["lat"] = lat;
    entity_json["position"]["lon"] = lon;
    entity_json["position"]["alt"] = alt;
    entity_json["orientation"]["yaw"] = quat.yaw();
    entity_json["orientation"]["pitch"] = quat.pitch();
    entity_json["orientation"]["roll"] = quat.roll();
    entity_json["isAlive"] = entity->is_alive();

    // pack websocket msg
    msg["topic"] = "vehicle";
    msg["data"] = entity_json;
    ws_handler_->send(msg.dump());
  }
}

void CesiumWebServer::serve_blocking() {
  server_->serve(web_src_dir_.c_str(), web_host_port_);
}

void CesiumWebServer::subShapes(scrimmage::MessagePtr<scrimmage_proto::ShapePtr> &msg) {
  json shape, json_msg;

  const auto rgb = msg->data->color();
  shape["id"] = msg->data->hash();
  if (msg->data->has_sphere()) {
    const double r = msg->data->sphere().radius();
    shape["name"] = "sphere";
    double lat, lon, alt;
    const auto center = msg->data->sphere().center();
    parent_->projection()->Reverse(center.x(), center.y(), center.z(), lat, lon, alt);
    shape["position"] = {lon, lat, alt};
    std::cout << "sphere position: " << shape["position"] << std::endl;
    shape["ellipsoid"]["radii"] = {r, r, r};
    shape["ellipsoid"]["material"] = {rgb.r()/255.0, rgb.g()/255.0, rgb.b()/255.0, msg->data->opacity()};
  } else if (msg->data->has_line()) {
    shape["name"] = "line";
    double slat, slon, salt, elat, elon, ealt;
    const auto start = msg->data->line().start();
    const auto end = msg->data->line().end();
    parent_->projection()->Reverse(start.x(), start.y(), start.z(), slat, slon, salt);
    parent_->projection()->Reverse(end.x(), end.y(), end.z(), elat, elon, ealt);
    shape["polyline"]["positions"] = {slon, slat, salt, elon, elat, ealt};
    shape["polyline"]["width"] = msg->data->line().width();
    shape["polyline"]["material"] = {rgb.r()/255.0, rgb.g()/255.0, rgb.b()/255.0, msg->data->opacity()};
  } else if (msg->data->has_cuboid()) {
    const auto cub = msg->data->cuboid();
    shape["name"] = "cuboid";
    double lat, lon, alt;
    const auto center = cub.center();
    parent_->projection()->Reverse(center.x(), center.y(), center.z(), lat, lon, alt);
    shape["position"] = {lon, lat, alt};
    shape["box"]["dimensions"] = {cub.x_length(), cub.y_length(), cub.z_length()};
    shape["box"]["material"] = {rgb.r()/255.0, rgb.g()/255.0, rgb.b()/255.0, msg->data->opacity()};
  } else if (msg->data->has_polyline()) {
    shape["name"] = "polyline";
    shape["polyline"]["positions"] = {};
    for (auto pt : msg->data->polyline().point()) {
      double lat, lon, alt;
      parent_->projection()->Reverse(pt.x(), pt.y(), pt.z(), lat, lon, alt);
      shape["polyline"]["positions"].push_back(lon);
      shape["polyline"]["positions"].push_back(lat);
      shape["polyline"]["positions"].push_back(alt);
      const auto ps = shape["polyline"]["positions"];
    }
    shape["polyline"]["width"] = 1;
    shape["polyline"]["material"] = {rgb.r()/255.0, rgb.g()/255.0, rgb.b()/255.0, msg->data->opacity()};
  }

  // send json msg
  if (shape != nullptr) {
    json_msg["topic"] = "shape";
    json_msg["data"] = shape;
    ws_handler_->send(json_msg.dump());
  }
}

} // namespace interaction
} // namespace scrimmage
