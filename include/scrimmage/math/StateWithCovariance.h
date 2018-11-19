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

#ifndef INCLUDE_SCRIMMAGE_MATH_STATEWITHCOVARIANCE_H_
#define INCLUDE_SCRIMMAGE_MATH_STATEWITHCOVARIANCE_H_

#include <scrimmage/math/State.h>

#include <memory>

namespace scrimmage {
class StateWithCovariance : public scrimmage::State {
 public:
    StateWithCovariance();
    explicit StateWithCovariance(const scrimmage::State &state);
    StateWithCovariance(const scrimmage::State &state,
                        const int &cov_num_rows, const int &cov_num_cols,
                        const double &cov_diag);

    void set_covariance(const Eigen::MatrixXd &covariance);
    const Eigen::MatrixXd &covariance();

 protected:
    Eigen::MatrixXd covariance_;
};
using StateWithCovariancePtr = std::shared_ptr<StateWithCovariance>;
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_MATH_STATEWITHCOVARIANCE_H_
