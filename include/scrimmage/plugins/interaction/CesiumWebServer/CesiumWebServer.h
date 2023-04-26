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

#ifndef INCLUDE_LCCM_SCRIMMAGE_PLUGINS_INTERACTION_CESIUMWEBSERVER_CESIUMWEBSERVER_H_
#define INCLUDE_LCCM_SCRIMMAGE_PLUGINS_INTERACTION_CESIUMWEBSERVER_CESIUMWEBSERVER_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include "scrimmage/plugins/interaction/CesiumWebServer/CesiumWebSocketHandler.h"
#include "seasocks/Server.h"
#include "seasocks/Connection.h"
#include "seasocks/PrintfLogger.h"
#include "seasocks/util/Json.h"

#include <map>
#include <memory>
#include <list>
#include <string>
#include <thread>

namespace scrimmage {
namespace interaction {

class CesiumWebServer : public scrimmage::EntityInteraction {
 public:
  CesiumWebServer();
  ~CesiumWebServer();
  bool init(std::map<std::string, std::string> &mission_params,
        std::map<std::string, std::string> &plugin_params) override;
  bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                 double t, double dt) override;
 protected:
 private:
  std::shared_ptr<seasocks::Server> server_;
  std::shared_ptr<std::thread> serve_thread_;
  std::shared_ptr<CesiumWebSocketHandler> ws_handler_;
  std::string web_src_dir_path_;
  int web_host_port_;
  void serve_blocking();
  void subShapes(scrimmage::MessagePtr<scrimmage_proto::ShapePtr> &msg);
  std::list<scrimmage::EntityPtr> ents_;
  void sendEntities();
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_LCCM_SCRIMMAGE_PLUGINS_INTERACTION_CESIUMWEBSERVER_CESIUMWEBSERVER_H_
