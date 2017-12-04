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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_MULTIROTOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_MULTIROTOR_H_

#include <scrimmage/plugins/motion/Multirotor/Rotor.h>

#include <scrimmage/math/State.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/common/CSV.h>

#include <map>
#include <string>
#include <vector>

namespace scrimmage {
namespace motion {
class Multirotor : public scrimmage::MotionModel{
 public:
    enum ModelParams {
        U = 0,
        V,
        W,
        P,
        Q,
        R,
        U_dot,
        V_dot,
        W_dot,
        P_dot,
        Q_dot,
        R_dot,
        Uw,
        Vw,
        Ww,
        Xw,
        Yw,
        Zw,
        q0,
        q1,
        q2,
        q3,
        MODEL_NUM_ITEMS
    };


    Multirotor();

    virtual bool init(std::map<std::string, std::string> &info,
                      std::map<std::string, std::string> &params);
    virtual bool step(double time, double dt);

    virtual void model(const vector_t &x , vector_t &dxdt , double t);

    class Controller : public scrimmage::Controller {
     public:
        virtual Eigen::VectorXd &u() = 0;
    };

    std::vector<Rotor> & rotors() { return rotors_; }

    double omega_max() { return wmax_; }
    double omega_min() { return wmin_; }

 protected:
    scrimmage::Quaternion quat_world_;
    scrimmage::Quaternion quat_local_;
    Eigen::VectorXd ctrl_u_;

    // World parameters
    double g_ = 9.81;

    // multirotor parameters
    Eigen::Matrix3d I_;
    Eigen::Matrix3d I_inv_;
    double c_D_ = 0.058; // drag coeff ( D = 0.5 * cd(i) * Vmag * V(i) ).

    std::vector<Rotor> rotors_;

    // Rotor parameters
    double c_T_ = 5.45e-6;  // thrust = ct*w^2 (N)
    double c_Q_ = 2.284e-7; // torque = cq*w^2 (Nm)
    double wmax_ = 1200.0;  //   1440000 max omega^2
	double wmin_ = 346.41;  //   120000
    double w0_ =  734.847;  // 540000 initial omega^2

    // Logging utility
    bool write_csv_;
    CSV csv_;

 private:
};
} // namespace motion
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_MOTION_MULTIROTOR_MULTIROTOR_H_
