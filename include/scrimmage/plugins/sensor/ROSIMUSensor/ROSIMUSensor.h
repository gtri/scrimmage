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

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/math/Quaternion.h>

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/common/CSV.h>

#include <random>
#include <vector>
#include <map>
#include <string>
#include <memory>

namespace scrimmage {
namespace sensor {

class ROSIMUSensor : public scrimmage::Sensor {
 public:
  ROSIMUSensor();
  void init(std::map<std::string, std::string> &params) override;
  bool step() override;
  void close(double t) override;

 protected:
  std::string ros_namespace_;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher imu_pub_;
  double prev_time_ = 0.0;
  scrimmage::CSV csv;

  // WGS84 constants
  const double ecc = 8.1819190842621E-2;
  const double earth_radius = 6378137;
  const double flattening_inverse = 298.257223563;
  const double e_prime = sqrt(1 - ecc * ecc);
  const double ae_squared = earth_radius * ecc * ecc;
  const double k1 = 1 - ecc * ecc;
  const double earth_semiminor_axis = earth_radius * (1 - 1 / flattening_inverse); // aprox. 6356752.3142;
  const double a2 = earth_radius * earth_radius;
  const double b2 = earth_semiminor_axis * earth_semiminor_axis;
  const double e = 521854.0089737218;
  const double e2 = 272331606681.94534;
  const double wgs84_grav = 3986004.418e+08;
  const double earth_rate = 7.2921151467e-5;
  const double omega2 = earth_rate * earth_rate;

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
  Eigen::Matrix3d ned_to_ecef_rotation(double lat, double lon);
  Eigen::Vector3d get_deltaV(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Quaterniond bodyToEcef, double deltaT);
  Eigen::Vector3d ecef_to_lla(Eigen::Vector3d ecef);
  Eigen::Matrix3d get_rotation_matrix_from_ecef_to_ned(double lat, double lon);
  Eigen::Vector3d gravity_ned_from_lla(Eigen::Vector3d lla);
  Eigen::Matrix3d skew_sym(Eigen::Vector3d vector);
  Eigen::Vector3d get_delta_theta(Eigen::Quaterniond qBodyToECEFt1Hat, Eigen::Quaterniond qBodyToECEFt2, double inertialDeltaT);
  Eigen::Vector3d inv_skew_sym(Eigen::Matrix3d inputMatrix);
  Eigen::Quaterniond propagate_quaternion(Eigen::Quaterniond qBodyToECEFt1Hat, Eigen::Vector3d deltaThetaBodyWRTInertialInBody, double inertialDeltaT);
  Eigen::Quaterniond integrate_quaternion(Eigen::Quaterniond qBToA, Eigen::Vector3d deltaThetasBFrame);
  Eigen::Quaterniond omega_to_q_dot(Eigen::Quaterniond qBToA, Eigen::Vector3d omegaABInB);

 private:
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_ROSIMUSENSOR_ROSIMUSENSOR_H_
