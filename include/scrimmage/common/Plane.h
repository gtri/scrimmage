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

#ifndef INCLUDE_SCRIMMAGE_COMMON_PLANE_H_
#define INCLUDE_SCRIMMAGE_COMMON_PLANE_H_
#include <scrimmage/common/Shape.h>

namespace scrimmage {

class Plane : public Shape {
 public:
    Plane() : Shape() {
        center_ << 0, 0, 0;
        normal_ << 0, 0, 0;
    }

    void set_center(Eigen::Vector3d & center) { center_ = center; }
    void set_normal(Eigen::Vector3d & normal) { normal_ = normal; }

    Eigen::Vector3d & center() { return center_; }
    Eigen::Vector3d & normal() { return normal_; }

 protected:
    Eigen::Vector3d center_;
    Eigen::Vector3d normal_;
};

typedef std::shared_ptr<Plane> PlanePtr;

} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_PLANE_H_
