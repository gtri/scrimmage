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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_STRAIGHTVOINIT_STRAIGHTVOINIT_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_STRAIGHTVOINIT_STRAIGHTVOINIT_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>

#include <Eigen/Dense>

#include <map>
#include <string>
#include <memory>

namespace scrimmage {

namespace interaction {
class BoundaryBase;
}

namespace autonomy {

class InitManeuverType {
 public:
    bool active = false;
};

class StraightVOInit : public scrimmage::Autonomy{
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    double speed_;
    Eigen::Vector3d goal_;
    Eigen::Vector3d init_goal_;

    int frame_number_;
    bool show_camera_images_;
    bool save_camera_images_;
    bool show_text_label_;

    bool enable_boundary_control_ = false;
    std::shared_ptr<scrimmage::interaction::BoundaryBase> boundary_;

    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;
    int desired_heading_idx_ = 0;

    bool noisy_state_set_ = false;
    State noisy_state_;
    ContactMap noisy_contacts_;

    double desired_z_ = 0;
    Eigen::Vector3d prev_diff_ = {0,0,0};

    PublisherPtr init_maneuver_pub_;
    bool use_init_maneuver_ = false;
    bool init_maneuver_finished_ = true;
    std::deque<State> get_init_maneuver_positions();
    std::deque<State> man_positions_;
    Eigen::Vector3d init_man_goal_pos_;
    Quaternion init_man_goal_quat_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_STRAIGHTVOINIT_STRAIGHTVOINIT_H_
