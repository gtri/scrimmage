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

#include <scrimmage/common/PositionTracker.h>
#include <iostream>

using std::cout;
using std::endl;

namespace scrimmage {

PositionTracker::PositionTracker() : initialized_(false) {
}

void PositionTracker::init(double dt) {
    A_.resize(6, 6);     // State transition
    B_.resize(6, 3);     // Input matrix
    H_.resize(3, 6);     // Measurement matrix
    Q_.resize(6, 6);     // Process noise
    R_.resize(3, 3);     // Measurement noise
    x0_.resize(6, 1);    // Initial state
    P_.resize(6, 6);     // Covariance matrix

    // make sure dt value makes sense
    if (dt <= 0) {
        dt = 0.066666667;
        cout << "WARNING: dt has to be greater than zero. Setting to 15Hz." << endl;
    }

    // main matrix
    A_ << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // Inputs are from velocity
    B_ << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    // Measurements are from x/y positions
    H_ << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt;

    double q = 1 * dt;
    Q_ = Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * q;

    double r = 10;
    R_ << r, 0, 0,
        0, r, 0,
        0, 0, r;

    z_.resize(3, 1);
    z_ << 0, 0, 0;

    u_.resize(3, 1);
    u_ << 0, 0, 0;

    double p = 10;
    P_ << Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * p;

    // set the Kalman Model (KalmanFilter.cpp)
    kf_.setModel(A_, B_, H_, Q_, R_);
}

Eigen::MatrixXf PositionTracker::get_covariance() {
    return kf_.get_covariance();
}

Eigen::MatrixXf PositionTracker::get_meas_variance() {
    return kf_.get_meas_variance();
}

void PositionTracker::set_process_variance(double q) {
    Q_ = Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * q;
    kf_.set_process_variance(Q_);
}

void PositionTracker::set_meas_variance(double r) {
    R_ = Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * r;
    kf_.set_meas_variance(R_);
}

void PositionTracker::set_state_variance(double p) {
    P_ << Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * p;
    kf_.set_state_variance(P_);
}

void PositionTracker::set_dt(double dt) {
    init(dt);
}

void PositionTracker::set_measurement(Eigen::Vector3d m) {
    if (initialized_) {
        z_ << m.x(), m.y(), m.z();
        kf_.update(z_);
    } else {
        initialized_ = true;
        x0_ << m.x(), m.y(), m.z(), 0, 0, 0;
        kf_.init(x0_, P_);
    }
}

void PositionTracker::predict() {
    if (initialized_) {
        kf_.predict(u_);
    }
}

Eigen::Vector3d PositionTracker::get_position() {
    Eigen::MatrixXf s = kf_.get_state();
    Eigen::Vector3d p(s(0, 0), s(1, 0), s(2, 0));
    return p;
}

Eigen::Vector3d PositionTracker::get_velocity() {
    Eigen::MatrixXf s = kf_.get_state();
    Eigen::Vector3d v(s(3, 0), s(4, 0), s(5, 0));
    return v;
}

Eigen::MatrixXd PositionTracker::get_meas_covariance() {
    return kf_.get_meas_covariance().cast<double>();
}

void PositionTracker::set_position(Eigen::Vector3d p) {
    Eigen::MatrixXf x(6, 1);
    x << p.x(), p.y(), p.z(), 0, 0, 0;
    kf_.set_state(x);
}

void PositionTracker::print() {
    kf_.print();
}
} // namespace scrimmage
