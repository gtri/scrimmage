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
 * @author Natalie Rakoski <natalie.rakoski@gtri.gatech.edu>
 * @date 27 January 2020
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_

#include <scrimmage/common/CSV.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugins/sensor/ROSIMUSensor/HG4930IMUBudget.h>
#include <scrimmage/plugins/sensor/ROSIMUSensor/IMUErrorSimulator.h>
#include <scrimmage/sensor/Sensor.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <GeographicLib/Geocentric.hpp>

namespace scrimmage {
namespace sensor {

class ROSIMUSensor : public scrimmage::Sensor {
 public:
    ROSIMUSensor();
    void init(std::map<std::string, std::string>& params) override;
    bool step() override;
    void close(double t) override;

 protected:
    std::string vehicle_name_ = "none";
    std::string ros_namespace_;
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher imu_pub_;
    double prev_time_ = 0.0;
    scrimmage::CSV csv;
    HG4930IMUBudget error_budget;
    IMUErrorSimulator* error_sim = nullptr;

    // WGS84 constants
    const double earth_radius = GeographicLib::Constants::WGS84_a();
    const double flattening = GeographicLib::Constants::WGS84_f();
    const double wgs84_grav = GeographicLib::Constants::WGS84_GM();
    const double earth_rate = GeographicLib::Constants::WGS84_omega();
    // derived
    const double omega2 = earth_rate * earth_rate;
    const double ecc2 = flattening * (2.0 - flattening);
    const double flattening_inverse = 1.0 / flattening;
    const double e_prime = sqrt(1.0 - ecc2);
    const double ae_squared = earth_radius * ecc2;
    const double k1 = 1 - ecc2;
    const double earth_semiminor_axis = earth_radius * (1.0 - flattening);
    const double a2 = earth_radius * earth_radius;
    const double b2 = earth_semiminor_axis * earth_semiminor_axis;
    const double e2 = a2 - b2;
    const double e = sqrt(e2);  // linear eccentricity

    // storage variables for previous frame data
    Eigen::Vector3d vel_t1;
    Eigen::Vector3d pos_ECEF_t1;
    Eigen::Matrix3d c_ECEF_To_NED_t1;
    Eigen::Quaterniond prev_qBody_to_ECEF;
    Eigen::Quaterniond qBody_to_ECEF_hat;
    Eigen::Vector3d p_hat;
    Eigen::Vector3d v_hat;
    bool first_sample_collected = false;
    Eigen::Vector3d lla_to_ecef(double lat, double lon, double alt);
    Eigen::Matrix3d enu_to_ecef_rotation(double lat, double lon);
    Eigen::Vector3d get_deltaV(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Quaterniond bodyToEcef, double deltaT);
    Eigen::Vector3d ecef_to_lla(Eigen::Vector3d ecef);
    Eigen::Matrix3d ecef_to_ned_rotation(double lat, double lon);
    Eigen::Vector3d gravity_ned_from_lla(Eigen::Vector3d lla);
    Eigen::Matrix3d skew_sym(Eigen::Vector3d vector);
    Eigen::Vector3d get_delta_theta(Eigen::Quaterniond qBodyToECEFt1Hat, Eigen::Quaterniond qBodyToECEFt2,
                                    double inertialDeltaT);
    Eigen::Vector3d inv_skew_sym(Eigen::Matrix3d inputMatrix);
    Eigen::Quaterniond propagate_quaternion(Eigen::Quaterniond qBodyToECEFt1Hat,
                                            Eigen::Vector3d deltaThetaBodyWRTInertialInBody, double inertialDeltaT);
    Eigen::Quaterniond integrate_quaternion(Eigen::Quaterniond qBToA, Eigen::Vector3d deltaThetasBFrame);
    Eigen::Quaterniond omega_to_q_dot(Eigen::Quaterniond qBToA, Eigen::Vector3d omegaABInB);

 private:
};
}  // namespace sensor
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_
