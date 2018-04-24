/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2018 by the Georgia Tech Research Institute (GTRI)
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
 * @date April 10, 2018
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_COMMON_POSITIONTRACKER_H_
#define INCLUDE_SCRIMMAGE_COMMON_POSITIONTRACKER_H_

#include <scrimmage/common/KalmanFilter.h>
#include <Eigen/Dense>

namespace scrimmage {
class PositionTracker {
 public:
    PositionTracker();

    void init(double dt);
    void set_measurement(Eigen::Vector3d m);
    void predict();
    Eigen::Vector3d get_position();
    Eigen::Vector3d get_velocity();
    Eigen::MatrixXd get_meas_covariance();
    void set_position(Eigen::Vector3d p);
    void set_meas_variance(double r);
    void set_state_variance(double p);
    void set_process_variance(double q);
    void set_dt(double dt);
    Eigen::MatrixXf get_meas_variance();
    Eigen::MatrixXf get_covariance();
    void print();
    bool initialized() { return initialized_; }

 protected:
    KalmanFilter kf_;       // Kalman model from KalmanFilter.cpp
    Eigen::MatrixXf A_;     // system matrix
    Eigen::MatrixXf B_;     // input matrix
    Eigen::MatrixXf H_;     // measurement matrix
    Eigen::MatrixXf Q_;     // process noise matrix
    Eigen::MatrixXf R_;     // measurement noise matrix

    Eigen::MatrixXf x0_;    // initial position
    Eigen::MatrixXf P_;     // initial state covariance matrix

    Eigen::MatrixXf u_;     // Input acceleration
    Eigen::MatrixXf z_;     // Measurement (position)

    bool initialized_;
};
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_POSITIONTRACKER_H_
