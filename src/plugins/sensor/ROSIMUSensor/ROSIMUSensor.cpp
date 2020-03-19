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

#include <scrimmage/plugins/sensor/ROSIMUSensor/ROSIMUSensor.h>
#include <math.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/math/Angles.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <fstream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::ROSIMUSensor, ROSIMUSensor_plugin)

namespace scrimmage {
namespace sensor {

ROSIMUSensor::ROSIMUSensor() {}

void ROSIMUSensor::init(std::map<std::string, std::string> &params) {

  if (!ros::isInitialized()) {
    int argc = 0;
    // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
    ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
  }
  nh_ = std::make_shared<ros::NodeHandle>();

  // Setup robot namespace
  ros_namespace_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
  ros_namespace_ += std::to_string(parent_->id().id());

  // Create Publisher
  imu_pub_ = nh_->advertise<sensor_msgs::Imu>(ros_namespace_ + "/imu", 1);

    // Open imu_data CSV for append (app) and set column headers
    std::string csv_filename = parent_->mp()->log_dir() + "/imu_data_robot" + std::to_string(parent_->id().id()) + ".csv";
    if (!csv.open_output(csv_filename, std::ios_base::app)) std::cout << "Couldn't create csv file" << endl;
    if (!csv.output_is_open()) cout << "File isn't open. Can't write to CSV" << endl;

  csv.set_column_headers("time, dt, ECEF_POSX, ECEF_POSY, ECEF_POSZ, ECEF_VELX, ECEF_VELY, ECEF_VELZ, bodyToEcef_X, bodyToEcef_Y, bodyToEcef_Z, bodyToEcef_W, dV_X, dV_Y, dV_Z, dTheta_X, dTheta_Y, dTheta_Z");

  prev_time_ = time_->t();
  first_sample_collected = false;
}

Eigen::Vector3d ROSIMUSensor::lla_to_ecef(double lat, double lon, double alt) {
  Eigen::Vector3d ecef;

  double factor = sin(lat);
  factor = sqrt(1 - ecc * ecc * factor * factor);
  ecef(0) = (earth_radius / factor + alt) * cos(lat) * cos(lon);
  ecef(1) = (earth_radius / factor + alt) * cos(lat) * sin(lon);
  ecef(2) = ((earth_radius * (1 - ecc * ecc)) / factor + alt) * sin(lat);

  return ecef;
}

Eigen::Vector3d ROSIMUSensor::ecef_to_lla(Eigen::Vector3d ecef) {
  Eigen::Vector3d lla;
  double p = sqrt(ecef(0) * ecef(0) + ecef(1) * ecef(1));
  double zPrime = e_prime * ecef(2);
  double T = ecef(2) / (e_prime * p);
  // % Perform first iteration
  double k2 = T * T;
  double k3 = 1 + k2;
  double k4 = sqrt(k3);
  double k5 = ae_squared / (k4 * k3);
  T = (zPrime + k5 * k2 * T) / (p - k5);
  // % Perform second iteration now
  k2 = T * T;
  k3 = 1 + k2;
  k4 = sqrt(k3);
  k5 = ae_squared / (k4 * k3);
  T = (zPrime + k5 * k2 * T) / (p - k5);
  lla(0) = atan(T / e_prime);
  k2 = T * T;
  k3 = 1 + k2;
  k4 = sqrt(k3);
  if (p > ecef(2)) {
    lla(2) = (sqrt(k1 + k2) / e_prime) * (p - earth_radius / k4);
  } else {
    lla(2) = (sqrt(k1 + k2)) * (ecef(2) / T - earth_radius / k4);
  }
  lla(1) = atan2(ecef(1), ecef(0));
  return lla;
}

Eigen::Vector3d ROSIMUSensor::gravity_ned_from_lla(Eigen::Vector3d lla) {
  Eigen::Vector3d g;

  double sLat = sin(lla(0));
  double cLat = cos(lla(0));

  double N = earth_radius / sqrt(1.0 - ecc * ecc * sLat * sLat);
  double x = (N + lla(2)) * cLat * cos(lla(1));
  double y = (N + lla(2)) * cLat * sin(lla(1));
  double z = ((b2 / a2) * N + lla(2)) * sLat;

  double x2 = x * x;
  double y2 = y * y;
  double z2 = z * z;
  double r2 = x2 + y2 + z2 - e2;
  double u2 = 0.5 * r2 * (1.0 + sqrt(1.0 + 4.0 * e2 * z2 / (r2 * r2)));
  double u = sqrt(u2);
  double v2 = u2 + e2;
  double v = sqrt(v2);

  double Beta = atan2(z * v, u * sqrt(x2 + y2));
  double sBeta = sin(Beta);
  double sBeta2 = sBeta * sBeta;
  double cBeta = cos(Beta);

  double w = sqrt(u2 + e2 * sBeta2) / v;
  double q = 0.5 * ((1.0 + 3.0 * u2 / e2) * atan2(e, u) - 3.0 * u / e);
  double q0 = 0.5 * ((1.0 + 3.0 * b2 / e2) * atan2(e, earth_semiminor_axis) - 3.0 * earth_semiminor_axis / e);
  double qp = 3.0 * (1.0 + u2 / e2) * (1.0 - (u / e) * atan2(e, u)) - 1.0;

  // %   //Mass Attraction Only
  double Gu = -(wgs84_grav / v2 + omega2 * a2 * e * qp * (0.5 * sBeta2 - 1.0 / 6.0) / (q0 * v2)) / w;
  double Gb = omega2 * sBeta * cBeta * a2 * q / (w * q0 * v);
  double Psi = atan2(z, sqrt(x2 + y2));
  double sPsi = sin(Psi);
  double cPsi = cos(Psi);
  double Alfa = lla(0) - Psi;
  double sAlfa = sin(Alfa);
  double cAlfa = cos(Alfa);
  double Gr = Gu * (cPsi * cBeta * u / v + sPsi * sBeta) / w + Gb * (sPsi * cBeta * u / v - cPsi * sBeta) / w;
  double Gp = -Gu * (sPsi * cBeta * u / v - cPsi * sBeta) / w + Gb * (cPsi * cBeta * u / v + sPsi * sBeta) / w;

  g(0) = -Gr * sAlfa + Gp * cAlfa;
  g(1) = 0.0;
  g(2) = -Gr * cAlfa - Gp * sAlfa;

  return g;
}

Eigen::Matrix3d ROSIMUSensor::ned_to_ecef_rotation(double lat, double lon) {
  double sLat = sin(lat);
  double sLon = sin(lon);
  double cLat = cos(lat);
  double cLon = cos(lon);
  Eigen::Matrix3d rotationMatrix;
  rotationMatrix(0, 0) = -sLat * cLon;
  rotationMatrix(0, 1) = -sLat * sLon;
  rotationMatrix(0, 2) = cLat;
  rotationMatrix(1, 0) = -sLon;
  rotationMatrix(1, 1) = cLon;
  rotationMatrix(1, 2) = 0.0;
  rotationMatrix(2, 0) = -cLat * cLon;
  rotationMatrix(2, 1) = -cLat * sLon;
  rotationMatrix(2, 2) = -sLat;
  return rotationMatrix;
}

Eigen::Matrix3d ROSIMUSensor::get_rotation_matrix_from_ecef_to_ned(double lat, double lon) {
  double sLat = sin(lat);
  double sLon = sin(lon);
  double cLat = cos(lat);
  double cLon = cos(lon);

  Eigen::Matrix3d CEcefToNed;
  CEcefToNed(0, 0) = -sLat * cLon;
  CEcefToNed(0, 1) = -sLat * sLon;
  CEcefToNed(0, 2) = cLat;

  CEcefToNed(1, 0) = -sLon;
  CEcefToNed(1, 1) = cLon;
  CEcefToNed(1, 2) = 0.0;

  CEcefToNed(2, 0) = -cLat * cLon;
  CEcefToNed(2, 1) = -cLat * sLon;
  CEcefToNed(2, 2) = -sLat;

  return CEcefToNed;
}

Eigen::Matrix3d ROSIMUSensor::skew_sym(Eigen::Vector3d vector) {
  Eigen::Matrix3d outputMatrix;
  outputMatrix(0, 0) = 0.0;
  outputMatrix(0, 1) = -vector(2);
  outputMatrix(0, 2) = vector(1);
  outputMatrix(1, 0) = vector(2);
  outputMatrix(1, 1) = 0.0;
  outputMatrix(1, 2) = -vector(0);
  outputMatrix(2, 0) = -vector(1);
  outputMatrix(2, 1) = vector(0);
  outputMatrix(2, 2) = 0.0;

  return outputMatrix;
}

Eigen::Vector3d ROSIMUSensor::inv_skew_sym(Eigen::Matrix3d inputMatrix) {
  Eigen::Vector3d OutputVector;
  OutputVector(0) = 0.5 * (-inputMatrix(1, 2) + inputMatrix(2, 1));
  OutputVector(1) = 0.5 * (inputMatrix(0, 2) - inputMatrix(2, 0));
  OutputVector(2) = 0.5 * (-inputMatrix(0, 1) + inputMatrix(1, 0));

  return OutputVector;
}

Eigen::Matrix3d ROSIMUSensor::enu_to_ecef_rotation(double lat, double lon) {
  Eigen::Matrix3d enuToNed;
  enuToNed << 0, 1, 0,
      1, 0, 0,
      0, 0, -1;
  Eigen::Matrix3d rotationMatrix = get_rotation_matrix_from_ecef_to_ned(lat, lon);
  // note don't use .transpose(), bad things happen https://eigen.tuxfamily.org/dox/group__TopicAliasing.html
  rotationMatrix.transposeInPlace();
  rotationMatrix = (rotationMatrix * enuToNed).eval();
  return rotationMatrix;
}

Eigen::Vector3d ROSIMUSensor::get_delta_theta(Eigen::Quaterniond qBodyToECEFt1Hat, Eigen::Quaterniond qBodyToECEFt2, double inertialDeltaT) {
  Eigen::Matrix3d CBodyToECEFt1Hat(qBodyToECEFt1Hat);
  Eigen::Matrix3d CBodyToECEFt2(qBodyToECEFt2);
  Eigen::Matrix3d CBodyt2ToBodyt1 = CBodyToECEFt1Hat.transpose() * CBodyToECEFt2;

  Eigen::Matrix3d Ak = CBodyt2ToBodyt1;
  Eigen::Vector3d dTHat = inv_skew_sym(CBodyt2ToBodyt1);
  Eigen::Vector3d errVect;
  for (int nn = 0; nn < 3; nn++) {
    Eigen::Matrix3d AkHat = skew_sym(dTHat).exp();
    errVect = inv_skew_sym(AkHat.transpose() * Ak);
    dTHat = errVect + dTHat;
  }
  // % .0035 deg/Hr is the quoted gyro bias for the HG9900, a navigation grade
  // % IMU
  if (errVect.norm() >= 0.0035 * (M_PI / 180.0) * (1 / 3600.0) * (inertialDeltaT) * (1 / 100.0)) {
    cout << "Warning: Check delta-theta calculations inside get_delta_theta!!!" << endl;
  }
  Eigen::Vector3d DeltaThetaBodyWRTECEFInBody = dTHat;

  Eigen::Vector3d EarthRateVector;
  EarthRateVector << 0, 0, earth_rate;
  Eigen::Vector3d deltaThetaBodyWRTInertialInBody = DeltaThetaBodyWRTECEFInBody + inertialDeltaT * CBodyToECEFt1Hat.transpose() * EarthRateVector;

  return deltaThetaBodyWRTInertialInBody;
}

Eigen::Quaterniond ROSIMUSensor::omega_to_q_dot(Eigen::Quaterniond qBToA, Eigen::Vector3d omegaABInB) {
  Eigen::Quaterniond Temp4By1;
  Temp4By1.w() = 0;
  Temp4By1.vec() = omegaABInB;
  Eigen::Quaterniond qBToADot = qBToA * Temp4By1;
  qBToADot.coeffs() *= 0.5;
  return qBToADot;
}

Eigen::Quaterniond ROSIMUSensor::integrate_quaternion(Eigen::Quaterniond qBToA, Eigen::Vector3d deltaThetasBFrame) {
  Eigen::Quaterniond QPredictor(qBToA.coeffs() + omega_to_q_dot(qBToA, deltaThetasBFrame).coeffs());
  Eigen::Quaterniond QCorrector(qBToA.coeffs() + omega_to_q_dot(QPredictor, deltaThetasBFrame).coeffs());
  // Perform the addition
  Eigen::Quaterniond sum;
  sum.w() = QPredictor.w() + QCorrector.w();           // Add the scalar portion
  sum.vec() = QPredictor.vec() + QCorrector.vec();     // Add the vector portion
  Eigen::Quaterniond qBToAUpdated(0.5 * sum.coeffs()); // (QPredictor + QCorrector);
  qBToAUpdated.coeffs() = qBToAUpdated.coeffs() / qBToAUpdated.norm();
  return qBToAUpdated;
}

Eigen::Quaterniond ROSIMUSensor::propagate_quaternion(Eigen::Quaterniond qBodyToECEFt1Hat, Eigen::Vector3d deltaThetaBodyWRTInertialInBody, double inertialDeltaT) {
  Eigen::Matrix3d CBodyToECEFt1Hat(qBodyToECEFt1Hat);

  Eigen::Vector3d EarthRateVector;
  EarthRateVector << 0, 0, earth_rate;
  Eigen::Vector3d deltaThetaBodyWRTECEFInBodyHat = deltaThetaBodyWRTInertialInBody - inertialDeltaT * CBodyToECEFt1Hat.transpose() * EarthRateVector;
  Eigen::Quaterniond qBodyToECEFt2 = integrate_quaternion(qBodyToECEFt1Hat, deltaThetaBodyWRTECEFInBodyHat);
  return qBodyToECEFt2;
}

Eigen::Vector3d ROSIMUSensor::get_deltaV(Eigen::Vector3d posECEF, Eigen::Vector3d velECEF, Eigen::Quaterniond qBodyToECEF, double InertialDeltaT) {
  double k1 = -1, k2 = -1.25;

  Eigen::Vector3d Acct1 = (velECEF - vel_t1) / InertialDeltaT;
  Eigen::Vector3d PosLLAt1 = ecef_to_lla(pos_ECEF_t1);
  c_ECEF_To_NED_t1 = get_rotation_matrix_from_ecef_to_ned(PosLLAt1(0), PosLLAt1(1));
  Eigen::Vector3d GravityNED = gravity_ned_from_lla(PosLLAt1);

  Eigen::Vector3d GravityECEF = c_ECEF_To_NED_t1.transpose() * GravityNED;
  Eigen::Matrix3d CECEFToBodyTruth(prev_qBody_to_ECEF);
  CECEFToBodyTruth.transposeInPlace();

  Eigen::Vector3d EarthRateVector;
  EarthRateVector << 0, 0, earth_rate;
  Eigen::Matrix3d EarthRateSkewSym = skew_sym(EarthRateVector);
  Eigen::Vector3d deltaV = CECEFToBodyTruth * (Acct1 + 2.0 * EarthRateSkewSym * vel_t1 + EarthRateSkewSym * EarthRateSkewSym * pos_ECEF_t1 - GravityECEF);
  deltaV = deltaV * InertialDeltaT;

  Eigen::Matrix3d CBodyToECEFt1Hat(qBody_to_ECEF_hat);
  Eigen::Vector3d LLA = ecef_to_lla(p_hat);
  GravityNED = gravity_ned_from_lla(LLA);

  Eigen::Matrix3d CECEFToNED = get_rotation_matrix_from_ecef_to_ned(LLA(0), LLA(1));
  GravityECEF = CECEFToNED.transpose() * GravityNED;

  Eigen::Vector3d PErr = pos_ECEF_t1 - p_hat;
  Eigen::Vector3d VErr = vel_t1 - v_hat;

  Eigen::Vector3d AAdd = -k1 * PErr - k2 * VErr;
  deltaV = (deltaV + InertialDeltaT * CBodyToECEFt1Hat.transpose() * AAdd);

  Eigen::Vector3d Ahat = CBodyToECEFt1Hat * deltaV / InertialDeltaT - 2 * EarthRateSkewSym * v_hat - EarthRateSkewSym * EarthRateSkewSym * p_hat + GravityECEF;
  Eigen::Vector3d tmpVhat = v_hat + (InertialDeltaT)*Ahat;
  p_hat = p_hat + (InertialDeltaT)*v_hat + 0.5 * (InertialDeltaT * InertialDeltaT) * Ahat;
  v_hat = tmpVhat;

  return deltaV;
}

bool ROSIMUSensor::step() {
  // Obtain current state information
  sc::StatePtr &state = parent_->state_truth();
  double time_now = time_->t();
  double dt = time_now - prev_time_;
  prev_time_ = time_now;

  // Fill IMU Message
  sensor_msgs::Imu imu_msg;

  // Header
  std_msgs::Header header; // empty header
  // TODO: header.frame = ? // No system for ROS Frame ID's yet
  header.stamp = ros::Time::now(); // time
  imu_msg.header = header;

  // get the geodetic origin specified in the mission file
  MissionParsePtr mp_ = parent_->mp();
  double lat = mp_->latitude_origin();
  double lon = mp_->longitude_origin();
  double alt = mp_->altitude_origin();

  // convert origin to ECEF and derive a rotation between ENU and ECEF coordinates
  Eigen::Vector3d ecefOrigin = lla_to_ecef(lat, lon, alt);
  Eigen::Matrix3d enuToEcef = enu_to_ecef_rotation(lat, lon);
  Eigen::Quaterniond qEnuToEcef(enuToEcef);

  // convert the current position (ENU) to ECEF
  Eigen::Vector3d ecefPos = state->pos();
  ecefPos = enuToEcef * ecefPos;  // rotate
  ecefPos = ecefPos + ecefOrigin; // translate

  // convert (ENU) velocity to ECEF
  Eigen::Vector3d ecefVel = state->vel();
  ecefVel = enuToEcef * ecefVel; // rotate

  // calculate the body to ECEF orientation
  Eigen::Quaterniond qbodyToEnu = state->quat();
  qbodyToEnu = qbodyToEnu.inverse();
  Eigen::Quaterniond qbodyToECEF = qEnuToEcef * qbodyToEnu;

  // calculate angular velocity and linear acceleration based on this and the previous frames
  if (!first_sample_collected) { // for the first sample, just store it, we need 2 samples to perform a calculation
    pos_ECEF_t1 = ecefPos;
    vel_t1 = ecefVel;
    prev_qBody_to_ECEF = qbodyToECEF;
    p_hat = pos_ECEF_t1;
    v_hat = vel_t1;
    qBody_to_ECEF_hat = qbodyToECEF;
    first_sample_collected = true; // first sample has been collected
  } else {
    // calculate deltas
    Eigen::Vector3d deltaV = get_deltaV(ecefPos, ecefVel, qbodyToECEF, dt);
    Eigen::Vector3d deltaTheta = get_delta_theta(qBody_to_ECEF_hat, qbodyToECEF, dt);

    // update values for next frame
    qBody_to_ECEF_hat = propagate_quaternion(qBody_to_ECEF_hat, deltaTheta, dt);
    pos_ECEF_t1 = ecefPos;
    vel_t1 = ecefVel;
    prev_qBody_to_ECEF = qbodyToECEF;

    // Populate the message with orientation
    imu_msg.orientation.x = qbodyToECEF.x();
    imu_msg.orientation.y = qbodyToECEF.y();
    imu_msg.orientation.z = qbodyToECEF.z();
    imu_msg.orientation.w = qbodyToECEF.w();

    // Angular Velocity
    // cout << "D Theta X: " << (state->quat().roll() - prev_quat_.roll()) / dt << endl;
    imu_msg.angular_velocity.x = deltaTheta.x();
    imu_msg.angular_velocity.y = deltaTheta.y();
    imu_msg.angular_velocity.z = deltaTheta.z();

    // Linear Acceleration
    // cout << "D Vel X: " << (state->vel().x() - prev_vel_.x()) / dt << endl;
    imu_msg.linear_acceleration.x = deltaV.x();
    imu_msg.linear_acceleration.y = deltaV.y();
    imu_msg.linear_acceleration.z = deltaV.z();

    // Publish Compass information
    imu_pub_.publish(imu_msg);

    // Write IMU data to CSV
    // Write the CSV file to the root log directory file name = imu_data.csv
    if (!csv.output_is_open()) {
      cout << "File isn't open. Can't append to CSV" << endl;
    }
    csv.append(sc::CSV::Pairs{
                   {"time", time_now},
                   {"dt", dt},
                   {"ECEF_POSX", ecefPos.x()},
                   {"ECEF_POSY", ecefPos.y()},
                   {"ECEF_POSZ", ecefPos.z()},
                   {"ECEF_VELX", ecefVel.x()},
                   {"ECEF_VELY", ecefVel.y()},
                   {"ECEF_VELZ", ecefVel.z()},
                   {"bodyToEcef_X", qbodyToECEF.x()},
                   {"bodyToEcef_Y", qbodyToECEF.y()},
                   {"bodyToEcef_Z", qbodyToECEF.z()},
                   {"bodyToEcef_W", qbodyToECEF.w()},
                   {"dV_X", deltaV.x()},
                   {"dV_Y", deltaV.y()},
                   {"dV_Z", deltaV.z()},
                   {"dTheta_X", deltaTheta.x()},
                   {"dTheta_Y", deltaTheta.y()},
                   {"dTheta_Z", deltaTheta.z()}},
               true, true);
  }
  return true;
}

void ROSIMUSensor::close(double t) {
  csv.close_output();
}
} // namespace sensor
} // namespace scrimmage
