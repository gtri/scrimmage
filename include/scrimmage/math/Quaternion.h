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

#ifndef INCLUDE_SCRIMMAGE_MATH_QUATERNION_H_
#define INCLUDE_SCRIMMAGE_MATH_QUATERNION_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace scrimmage {

class Quaternion : public Eigen::Quaternion<double, Eigen::DontAlign> {
 public:
    /*! \name constructors */
    ///@{
    /*! \brief set the quaternion to rotate an object around the given vector
     * at given angle
     */
    Quaternion();

    Quaternion(const Quaternion &other);

    explicit Quaternion(const Eigen::Quaternion<double, Eigen::DontAlign> &other);
    Quaternion &operator=(const Eigen::Quaternion<double, Eigen::DontAlign> &other);

    Quaternion(const double &w, const double &x,
               const double &y, const double &z);

    Quaternion(double &w, double &x,
               double &y, double &z);

    Quaternion(const Eigen::Vector3d &vector, double angle_radians);

    /*! \brief set quaternion from euler angles */
    Quaternion(double roll, double pitch, double yaw);

    void set(double w_coeff, double x_coeff, double y_coeff, double z_coeff);

    ///@}

    /*! \name setters */
    ///@{
    /*! \brief set the quaternion to rotate an object around the given vector
     * at given angle
     */
    void set(const Eigen::Vector3d &vector, double angle_radians);

    /*! \brief set quaternion from euler angles */
    void set(double roll, double pitch, double yaw);
    ///@}

    /*! \name utilities */
    ///@{
    /*! \brief return euler angle roll */
    double roll() const;

    /*! \brief return euler angle pitch */
    double pitch() const;

    /*! \brief return euler angle yaw */
    double yaw() const;

    /*! \brief return the angle an input vector would be rotated by this
     * quaternion
     */
    double rotation_angle() const;

    /*! \brief rotate the input vector around the vec axis by an angle
     * given by RotationAngle(). This can also be used to convert a vector in
     * the local frame to a vector in the global frame.
     */
    Eigen::Vector3d rotate(const Eigen::Vector3d &vec) const;

    /*! \brief rotate the input vector around the vec axis by an angle
     * given by RotationAngle(). This can also be used to convert a vector in
     * the local frame to a vector in the global frame.
     */
    Eigen::Vector3d rotate_reverse(const Eigen::Vector3d &vec) const;
    ///@}

    uint8_t output_precision = 2;
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
};

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_MATH_QUATERNION_H_
