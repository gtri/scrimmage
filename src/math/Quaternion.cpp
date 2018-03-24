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

#include <scrimmage/math/Quaternion.h>

#include <iostream>
#include <iomanip>

namespace scrimmage {

Quaternion::Quaternion() : Eigen::Quaternion<double, Eigen::DontAlign>() {}

Quaternion::Quaternion(const Quaternion &other) : Eigen::Quaternion<double, Eigen::DontAlign>(other) {}

Quaternion::Quaternion(const Eigen::Quaternion<double, Eigen::DontAlign> &other) : Eigen::Quaternion<double, Eigen::DontAlign>(other) {}

Quaternion::Quaternion(const double &w, const double &x, const double &y, const double &z)
    : Eigen::Quaternion<double, Eigen::DontAlign>(w, x, y, z) {}

Quaternion::Quaternion(double &w, double &x, double &y, double &z)
    : Eigen::Quaternion<double, Eigen::DontAlign>(w, x, y, z) {}

Quaternion::Quaternion(const Eigen::Vector3d &vector, double angle_radians) {
    set(vector, angle_radians);
}

Quaternion::Quaternion(double roll, double pitch, double yaw) {
    set(roll, pitch, yaw);
}

Quaternion &Quaternion::operator=(const Eigen::Quaternion<double, Eigen::DontAlign> &other) {
    if (this != &other) {
        set(other.w(), other.x(), other.y(), other.z());
    }
    return *this;
}

void Quaternion::set(double w_coeff, double x_coeff, double y_coeff, double z_coeff) {
    w() = w_coeff;
    x() = x_coeff;
    y() = y_coeff;
    z() = z_coeff;
}

void Quaternion::set(const Eigen::Vector3d &vector, double angle_radians) {
    w() = cos(angle_radians / 2.0);
    Eigen::Vector3d unit_vec = vector.normalized() * sin(angle_radians / 2.0);
    x() = unit_vec.x();
    y() = unit_vec.y();
    z() = unit_vec.z();
}

void Quaternion::set(double roll, double pitch, double yaw) {
    double sr = sin(roll / 2);
    double cr = cos(roll / 2);
    double sy = sin(yaw / 2);
    double cy = cos(yaw / 2);
    double sp = sin(pitch / 2);
    double cp = cos(pitch / 2);

    w() = cr * cp * cy + sr * sp * sy;
    x() = sr * cp * cy - cr * sp * sy;
    y() = cr * sp * cy + sr * cp * sy;
    z() = cr * cp * sy - sr * sp * cy;
}

double Quaternion::roll() const {
    return atan2(2 * (w() * x() + y() * z()),
                 1 - 2 * (pow(x(), 2) + pow(y(), 2)));
}

double Quaternion::pitch() const {
    return asin(2 * (w() * y() - z() * x()));
}

double Quaternion::yaw() const {
    return atan2(2 * (w() * z() + x() * y()),
                 1 - 2 * (pow(y(), 2) + pow(z(), 2)));
}

double Quaternion::rotation_angle() const {
    return 2 * acos(w());
}

Eigen::Vector3d Quaternion::rotate(const Eigen::Vector3d &vec) const {
    Eigen::Quaternion<double, Eigen::DontAlign> pure_quat(0, vec.x(), vec.y(), vec.z());
    return (*this * pure_quat * inverse()).vec();
}

Eigen::Vector3d Quaternion::rotate_reverse(const Eigen::Vector3d &vec) const {
    Eigen::Quaternion<double, Eigen::DontAlign> pure_quat(0, vec.x(), vec.y(), vec.z());
    return (inverse() * pure_quat * *this).vec();
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << "w: " << std::setprecision(q.output_precision) << q.w()
       << ", x: " << std::setprecision(q.output_precision) << q.x()
       << ", y: " << std::setprecision(q.output_precision) << q.y()
       << ", z: " << std::setprecision(q.output_precision) << q.z();
    return os;
}

} // namespace scrimmage
