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

#ifndef INCLUDE_SCRIMMAGE_COMMON_KALMANFILTER_H_
#define INCLUDE_SCRIMMAGE_COMMON_KALMANFILTER_H_

#include <Eigen/Dense>

namespace scrimmage {
class KalmanFilter {
 protected:
    Eigen::MatrixXf F_; // System dynamics
    Eigen::MatrixXf B_; // Control matrix
    Eigen::MatrixXf H_; // Measurement matrix

    Eigen::MatrixXf Q_; // Process variance     ; V*V'
    Eigen::MatrixXf R_; // Measurement variance ; W*W'

    // Matrices / vectors for Kalman step
    Eigen::MatrixXf x_; // Estimated state
    Eigen::MatrixXf P_; // State variance matrix
    Eigen::MatrixXf K_; // Kalman gain
    Eigen::MatrixXf eye_; // identity matrix

 public:
    KalmanFilter();
    KalmanFilter(const Eigen::MatrixXf &F, const Eigen::MatrixXf &B,
                 const Eigen::MatrixXf &H, const Eigen::MatrixXf &Q,
                 const Eigen::MatrixXf &R);

    int setModel(const Eigen::MatrixXf &F, const Eigen::MatrixXf &B,
                 const Eigen::MatrixXf &H, const Eigen::MatrixXf &Q,
                 const Eigen::MatrixXf &R);

    int init(const Eigen::MatrixXf &x0, const Eigen::MatrixXf &P0);

    void set_state(const Eigen::MatrixXf &x0);

    int predict(const Eigen::MatrixXf &u);
    int update(const Eigen::MatrixXf &z);

    Eigen::MatrixXf get_state() const;
    Eigen::MatrixXf get_covariance() const;
    Eigen::MatrixXf get_meas_variance() const;
    Eigen::MatrixXf get_meas_covariance();

    void set_meas_variance(Eigen::MatrixXf R) { R_ = R; }
    void set_state_variance(Eigen::MatrixXf P) { P_ = P; }
    void set_process_variance(Eigen::MatrixXf Q) { Q_ = Q; }

    void print();
};
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_KALMANFILTER_H_
