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
 * @author Edward Stevens <edward.stevens@gtri.gatech.edu>
 * @author William Syre <william.syre@gtri.gatech.edu>
 * @date 31 June 2022
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/TerrainCollision/TerrainCollision.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/pubsub/Message.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/GeoCoords.hpp>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::TerrainCollision,
                TerrainCollision_plugin)

namespace scrimmage {
namespace interaction {

TerrainCollision::TerrainCollision()    : remove_on_collision_(true),
                                          enable_startup_collisions_(true) {

}

bool TerrainCollision::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    
    std::string terrain_str = sc::get<std::string>("terrain_interaction", plugin_params, "grandcanyon.vtk");
    sc::common::TerrainEvaluator terrain;
    // terrain.terrain_utm_zone = sc::get<int>("utm_zone", plugin_params, 12);
    terrain.terrain_utm_zone = parent()->mp()->utm_terrain()->zone();
    std::cout << "[TC] Terrain UTM Zone: " << terrain.terrain_utm_zone << std::endl;
    std::cout << "[TC] Terrain file: " << parent()->mp()->utm_terrain()->poly_data_file() << std::endl;

    terrain.create_elevation_matrix(parent()->mp()->utm_terrain()->poly_data_file());

    //collision_pub_ = advertise("GlobalNetwork", "GroundCollision");

    return true;

}


bool TerrainCollision::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    sc::common::TerrainEvaluator terrain;
    for (sc::EntityPtr ent : ents)
    {
        if (ent->properties().count("Flying") == 0) {
            continue;
        }
        //Position conversion to UTM
        double x = ent->state()->pos().x();
        double y = ent->state()->pos().y();
        double z = ent->state()->pos().z();
        double lat,lon,alt;
        
        Eigen::Vector3d start = {x, y, z};
        Eigen::Vector3d end = {x, y, z+25};
        //Eigen::Vector3d new_start = {lat,lon,alt};

        parent_->projection()->Reverse(x,y,z,lat,lon,alt);
        
        GeographicLib::GeoCoords GC = GeographicLib::GeoCoords(lat,lon,terrain.terrain_utm_zone);


        // is_collision expects UTM coordinates be sure to convert position from ENU -> lat/lon/alt -> UTM before calling function
        if(terrain.is_collision(GC.Easting(), GC.Northing(), ent->state()->pos()(2))){

            ShapePtr segment = sc::shape::make_line(start, end, Eigen::Vector3d(255, 0, 0), 1);
            draw_shape(segment);
            std::cout << "[TC] Terrain Collision: Agent " << ent->id().id() << std::endl;

            ent->collision();

            //auto msg = std::make_shared<sc::Message<sm::GroundCollision>>();
            //msg->data.set_entity_id(ent->id().id());
            //collision_pub_->publish(msg);
        }
    }
    if (ents.empty()) {
        return true;
    }

    return true;
}
} // namespace interaction
} // namespace scrimmage
