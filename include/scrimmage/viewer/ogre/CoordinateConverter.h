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
 * @brief Coordinate conversion utilities for ENU to Ogre3D transforms.
 * @section DESCRIPTION
 * SCRIMMAGE uses ENU (East/North/Up) with Z-up.
 * Ogre3D uses Y-up by default.
 * This converter handles the transformation between coordinate systems.
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_COORDINATECONVERTER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_COORDINATECONVERTER_H_

#include <Eigen/Dense>
#include <OGRE/Ogre.h>
#include "scrimmage/math/Quaternion.h"
#include "scrimmage/proto/Vector3d.pb.h"
#include "scrimmage/proto/Quaternion.pb.h"

namespace scrimmage {
namespace viewer {

/**
 * @brief Convert ENU (Z-up) coordinates to Ogre3D (Y-up) coordinates.
 *
 * SCRIMMAGE ENU: X=East, Y=North, Z=Up
 * Ogre3D default: X=Right, Y=Up, Z=Back (into screen)
 *
 * Mapping: ENU.X -> Ogre.X, ENU.Y -> Ogre.Z (negated), ENU.Z -> Ogre.Y
 */
class CoordinateConverter {
 public:
    /**
     * @brief Convert Eigen::Vector3d (ENU) to Ogre::Vector3 (Y-up).
     */
    static Ogre::Vector3 toOgre(const Eigen::Vector3d& enu) {
        // ENU: (East, North, Up) -> Ogre: (X, Y, Z) where Y is up
        // X stays X, Z becomes Y, Y becomes -Z
        return Ogre::Vector3(
            static_cast<Ogre::Real>(enu.x()),   // East -> X
            static_cast<Ogre::Real>(enu.z()),   // Up -> Y
            static_cast<Ogre::Real>(-enu.y())   // North -> -Z (Ogre looks down -Z)
        );
    }

    /**
     * @brief Convert protobuf Vector3d to Ogre::Vector3.
     */
    static Ogre::Vector3 toOgre(const scrimmage_proto::Vector3d& v) {
        return Ogre::Vector3(
            static_cast<Ogre::Real>(v.x()),
            static_cast<Ogre::Real>(v.z()),
            static_cast<Ogre::Real>(-v.y())
        );
    }

    /**
     * @brief Convert x, y, z components (ENU) to Ogre::Vector3.
     */
    static Ogre::Vector3 toOgre(double x, double y, double z) {
        return Ogre::Vector3(
            static_cast<Ogre::Real>(x),
            static_cast<Ogre::Real>(z),
            static_cast<Ogre::Real>(-y)
        );
    }

    /**
     * @brief Convert SCRIMMAGE Quaternion to Ogre::Quaternion.
     *
     * The quaternion represents rotation in the ENU frame.
     * We need to conjugate and reorder to match Ogre's Y-up convention.
     */
    static Ogre::Quaternion toOgre(const scrimmage::Quaternion& q) {
        // First create the basic quaternion mapping:
        // ENU quaternion components: (w, x, y, z)
        // Ogre uses the same order but different axes
        Ogre::Quaternion coordRotation(
            static_cast<Ogre::Real>(q.w()),
            static_cast<Ogre::Real>(q.x()),
            static_cast<Ogre::Real>(q.z()),
            static_cast<Ogre::Real>(-q.y())
        );
        return coordRotation;
    }

    /**
     * @brief Convert protobuf Quaternion to Ogre::Quaternion.
     */
    static Ogre::Quaternion toOgre(const scrimmage_proto::Quaternion& q) {
        return Ogre::Quaternion(
            static_cast<Ogre::Real>(q.w()),
            static_cast<Ogre::Real>(q.x()),
            static_cast<Ogre::Real>(q.z()),
            static_cast<Ogre::Real>(-q.y())
        );
    }

    /**
     * @brief Convert Ogre::Vector3 back to Eigen::Vector3d (ENU).
     */
    static Eigen::Vector3d toENU(const Ogre::Vector3& v) {
        return Eigen::Vector3d(v.x, -v.z, v.y);
    }

    /**
     * @brief Apply scale to a vector (uniform scaling is coordinate-independent).
     */
    static Ogre::Vector3 toOgreScale(double scale) {
        return Ogre::Vector3(
            static_cast<Ogre::Real>(scale),
            static_cast<Ogre::Real>(scale),
            static_cast<Ogre::Real>(scale)
        );
    }

    /**
     * @brief Apply non-uniform scale (x, y, z in ENU order).
     */
    static Ogre::Vector3 toOgreScale(double sx, double sy, double sz) {
        // ENU scale (sx, sy, sz) maps to Ogre (sx, sz, sy)
        return Ogre::Vector3(
            static_cast<Ogre::Real>(sx),
            static_cast<Ogre::Real>(sz),
            static_cast<Ogre::Real>(sy)
        );
    }
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_COORDINATECONVERTER_H_
