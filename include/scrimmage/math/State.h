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

#ifndef INCLUDE_SCRIMMAGE_MATH_STATE_H_
#define INCLUDE_SCRIMMAGE_MATH_STATE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <scrimmage/math/Quaternion.h>

#include <memory>

namespace scrimmage {

class State {
 public:
    State();
    State(Eigen::Vector3d _pos, Eigen::Vector3d _vel, Quaternion _quat);

    Eigen::Vector3d &pos();
    Eigen::Vector3d &vel();
    Quaternion &quat();

    const Eigen::Vector3d &pos_const() const;
    const Eigen::Vector3d &vel_const() const;
    const Quaternion &quat_const() const;

    void set_pos(const Eigen::Vector3d &pos);
    void set_vel(const Eigen::Vector3d &vel);
    void set_quat(const Quaternion &quat);

    /*! \brief Returns true if other state is in field-of-view */
    bool InFieldOfView(State &other, double fov_width, double fov_height) const;

    /*! \brief convert the relative position to the local frame (the output
     * vector will point to the other state)
     */
    Eigen::Vector3d rel_pos_local_frame(Eigen::Vector3d &other) const;

    /*! \brief return position offset by trailing_distance in the direction of
     * velocity or orientation
     */
    Eigen::Vector3d pos_offset(double distance, bool offset_with_velocity = true) const;

    /*! \brief returns the vector extending forward */
    Eigen::Vector3d orient_global_frame() const;

    double rel_az(const Eigen::Vector3d &other) const;

    Eigen::Matrix4d tf_matrix(bool enable_translate = true);

 protected:
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Quaternion quat_;
};

using StatePtr = std::shared_ptr<State>;

} // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_MATH_STATE_H_
