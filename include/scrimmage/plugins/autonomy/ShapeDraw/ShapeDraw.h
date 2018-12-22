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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SHAPEDRAW_SHAPEDRAW_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SHAPEDRAW_SHAPEDRAW_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class ShapeDraw : public scrimmage::Autonomy {
 public:
    ShapeDraw();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    void draw_ellipse(double t, double dt);
    void draw_cuboid(double t, double dt);
    void draw_mesh(double t, double dt);
    int follow_id_;
    bool init_;

    scrimmage_proto::ShapePtr ellipse_shape_;
    scrimmage_proto::ShapePtr cuboid_shape_;
    scrimmage_proto::ShapePtr mesh_shape_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SHAPEDRAW_SHAPEDRAW_H_
