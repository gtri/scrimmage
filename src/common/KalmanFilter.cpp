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

#include <scrimmage/common/KalmanFilter.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using std::cout;
using std::endl;

namespace scrimmage {

KalmanFilter::KalmanFilter() {
}

KalmanFilter::KalmanFilter(const Eigen::MatrixXf &F,
                           const Eigen::MatrixXf &B,
                           const Eigen::MatrixXf &H,
                           const Eigen::MatrixXf &Q,
                           const Eigen::MatrixXf &R) {
    setModel(F, B, H, Q, R);
}

int KalmanFilter::setModel(const Eigen::MatrixXf &F,
                           const Eigen::MatrixXf &B,
                           const Eigen::MatrixXf &H,
                           const Eigen::MatrixXf &Q,
                           const Eigen::MatrixXf &R) {
    F_ = F;
    B_ = B;
    H_ = H;
    Q_ = Q;
    R_ = R;
    eye_ = Eigen::MatrixXf::Identity(F.rows(), F.cols());

    return 0;
}

int KalmanFilter::init(const Eigen::MatrixXf &x0,
                       const Eigen::MatrixXf &P0) {
    x_ = x0;
    P_ = P0;
    return 0;
}

int KalmanFilter::predict(const Eigen::MatrixXf &u) {
    x_ = F_*x_ + B_*u;
    P_ = F_*P_*F_.transpose() + Q_;
    return 0;
}

int KalmanFilter::update(const Eigen::MatrixXf &z) {
    Eigen::MatrixXf M = get_meas_covariance();
    K_ = P_ * H_.transpose() * M.inverse();
    x_ = x_ + K_*(z - H_*x_);
    P_ = (eye_ - K_*H_)*P_;
    return 0;
}

void KalmanFilter::set_state(const Eigen::MatrixXf &x) {
    x_ = x;
}

Eigen::MatrixXf KalmanFilter::get_state() const {
    return x_;
}

Eigen::MatrixXf KalmanFilter::get_covariance() const {
    return P_;
}

Eigen::MatrixXf KalmanFilter::get_meas_variance() const {
    return R_;
}

Eigen::MatrixXf KalmanFilter::get_meas_covariance() {
    return H_ * P_ * H_.transpose() + R_;
}

void KalmanFilter::print() {
    cout << "State Vector: " << endl << x_ << endl;
    cout << "Covariance Matrix: " << endl << P_ << endl;
    cout << "Measurement Variance: " << endl << R_ << endl;
}
} // namespace scrimmage
