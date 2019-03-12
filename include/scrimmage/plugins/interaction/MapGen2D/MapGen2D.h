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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_MAPGEN2D_MAPGEN2D_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_MAPGEN2D_MAPGEN2D_H_

#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/proto/Shape.pb.h>

#include <list>
#include <map>
#include <string>
#include <memory>

#include <opencv2/core/core.hpp>

namespace scrimmage {
namespace interaction {
class MapGen2D : public scrimmage::EntityInteraction {
 public:
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(
        std::list<scrimmage::EntityPtr> &ents, double t, double dt) override;

 protected:
    std::shared_ptr<scrimmage_proto::Shape> connect_points(
        Eigen::Vector3d &p, Eigen::Vector3d &prev_p);

    std::list<cv::Rect> find_rectangles(cv::Mat &img, int threshold);

    Eigen::Vector3d img_xy_to_xyz(int x, int y, cv::Mat &img);

    bool show_map_debug_;
    bool enable_map_boundary_;

    double wall_bottom_z_;
    double wall_height_;
    double resolution_;
    double occupied_thresh_;

    double x_origin_;
    double y_origin_;
    double z_origin_;

    scrimmage::PublisherPtr pub_shape_gen_;
    scrimmage::PublisherPtr pub_map_2d_info_;

    bool map_info_published_ = false;

    cv::Mat map_img_;
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_MAPGEN2D_MAPGEN2D_H_
