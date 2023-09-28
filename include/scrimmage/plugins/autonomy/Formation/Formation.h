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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_FORMATION_FORMATION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_FORMATION_FORMATION_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>

#include <Eigen/Dense>

#include <map>
#include <string>
#include <memory>
#include <map>

namespace scrimmage {

namespace interaction {
class BoundaryBase;
}

namespace autonomy {
class Formation : public scrimmage::Autonomy{
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    bool leader_;
    double leader_speed_;
    int leader_id;
    float leader_x_pos;
    float leader_y_pos;
    float leader_z_pos;
    float safety_dist_;

    double follower_speed_;
    double follow_v_k_;
    double x_disp_;
    double y_disp_;
    double z_disp_;

    bool show_shapes_;
    scrimmage_proto::ShapePtr circle_shape_;

    scrimmage::PublisherPtr leader_pub_;
    scrimmage::PublisherPtr follower_track_pub_;

    int ent_id;
    //std::map<int,int> follower_track; //ent_id, follower number

    struct FollowerData {
      double x_pos;
      double y_pos;
      double z_pos;
      int ent_id;
    };

    std::map<int, FollowerData> follower_map;

    Eigen::Vector3d goal_;

    int frame_number_;

    bool enable_boundary_control_ = false;
    std::shared_ptr<scrimmage::interaction::BoundaryBase> boundary_;

    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;
    int desired_heading_idx_ = 0;

    scrimmage_proto::ShapePtr text_shape_;
    scrimmage_proto::ShapePtr sphere_shape_;

    bool noisy_state_set_ = false;
    State noisy_state_;

    ContactMap noisy_contacts_;

    double desired_z_ = 0;

    double prev_gen_time_ = -1.0;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_FORMATION_FORMATION_H_
