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

#include <scrimmage/plugins/motion/DubinsAirplane3D/DubinsAirplane3D.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <boost/algorithm/clamp.hpp>
#include <chrono> // NOLINT

#include <string>
using std::cout;
using namespace std::chrono;

REGISTER_PLUGIN(scrimmage::MotionModel, scrimmage::motion::DubinsAirplane3D, DubinsAirplane3D_plugin)

namespace scrimmage {
namespace motion {

namespace sc = scrimmage;

namespace pl = std::placeholders;

enum ModelParams {
    U = 0,
    V,
    W,
    P, // roll rate
    Q, // pitch rate
    R, // yaw rate
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

bool DubinsAirplane3D::init(std::map<std::string, std::string> &info,
                            std::map<std::string, std::string> &params) {

    write_csv_ = sc::get<bool>("write_csv", params, false);

    // Model limits
    speed_max_ = sc::get<double>("speed_max", params, speed_max_);
    speed_min_ = sc::get<double>("speed_min", params, speed_min_);

    // Directly set speed, pitch, and roll
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::In);
    desired_pitch_idx_ = vars_.declare(VariableIO::Type::desired_pitch, VariableIO::Direction::In);
    desired_roll_idx_ = vars_.declare(VariableIO::Type::desired_roll, VariableIO::Direction::In);
    
    std::cout << "Resizing the odeval vector" << std::endl;

    x_.resize(MODEL_NUM_ITEMS);
    odeVal_.resize(MODEL_NUM_ITEMS); // I had this set to 9 because I thought it was related to the number of doubles in the vector... but that was causing memory chunk control structure fields in the adjacent following chunk are being overwritten due to out-of-bounds access by the code
    Eigen::Vector3d &pos = state_->pos();
    quat_world_ = state_->quat();
    quat_world_.normalize();

    quat_world_inverse_ = quat_world_.inverse();
    quat_world_inverse_.normalize();

    //Initializing odeVal_ vector
    std::cout << "Initializing the odeval vector" << std::endl;
    odeVal_[q0] = 0;
    odeVal_[q1] = 0;
    odeVal_[q2] = 0;
    odeVal_[q3] = 0;

    odeVal_[U] = 0;
    odeVal_[V] = 0;
    odeVal_[W] = 0;

    odeVal_[Xw] = 0;
    odeVal_[Yw] = 0;
    odeVal_[Zw] = 0;
    std::cout << "Did the odeval vector cause it to die?" << std::endl;

    x_[U] = 0;
    x_[V] = 0;
    x_[W] = 0;

    x_[P] = 0;
    x_[Q] = 0;
    x_[R] = 0;

    x_[Uw] = 0;
    x_[Vw] = 0;
    x_[Ww] = 0;

    x_[Xw] = pos(0);
    x_[Yw] = pos(1);
    x_[Zw] = pos(2);

    // Initial Local orientation (no rotation)
    quat_local_.w() = 1;
    quat_local_.x() = 0;
    quat_local_.y() = 0;
    quat_local_.z() = 0;
    quat_local_.normalize();
    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    if (write_csv_) {
        csv_.open_output(parent_->mp()->log_dir() + "/"
                         + std::to_string(parent_->id().id())
                         + "-dubins-airplane3d-states.csv");

        csv_.set_column_headers(sc::CSV::Headers{"t",
                        "x", "y", "z",
                        "U", "V", "W",
                        "P", "Q", "R",
                        "roll", "pitch", "yaw",
                        "speed",
                        "Uw", "Vw", "Ww"});
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
// Trying one ode step function
///////////////////////////////////////////////////////////////////////////////////////
bool DubinsAirplane3D::offset_step(double t, double dt) {
    std::cout << "In the offset step function" << std::endl;

    // Get inputs and saturate
    speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
    pitch_ = vars_.input(desired_pitch_idx_);
    roll_ = vars_.input(desired_roll_idx_);

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    // state_->quat().normalize();
    state_->quat().set(roll_, pitch_, state_->quat().yaw());
    quat_local_ = state_->quat() * quat_world_inverse_;
    quat_local_.normalize();
    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    Eigen::Vector3d force_body = quat_local_.rotate_reverse(ext_force_);
    Eigen::Vector3d ext_moment_body = ext_moment_;
    ext_force_ = Eigen::Vector3d::Zero();
    ext_moment_ = Eigen::Vector3d::Zero();

    x_[U] = speed_ + force_body(0) / mass_;
    x_[V] = force_body(1) / mass_;
    x_[W] = force_body(2) / mass_;

    double turn_rate = 0;
    if (std::abs(speed_) >= std::numeric_limits<double>::epsilon()) {
        turn_rate = -g_ / speed_ * tan(roll_);
    }

    x_[P] = ext_moment_body(0) / mass_;
    x_[Q] = ext_moment_body(1) / mass_;
    x_[R] = ext_moment_body(2) / mass_ + turn_rate;

    // Setting the odeVals vector to values before ode_step
    odeVal_[q0] = x_[q0];
    odeVal_[q1] = x_[q1];
    odeVal_[q2] = x_[q2];
    odeVal_[q3] = x_[q3];

    odeVal_[U] = x_[U];
    odeVal_[V] = x_[V];
    odeVal_[W] = x_[W];

    odeVal_[Xw] = x_[Xw];
    odeVal_[Yw] = x_[Yw];
    odeVal_[Zw] = x_[Zw];

    ode_step(dt);

    // Normalize quaternion
    quat_local_.w() = x_[q0];
    quat_local_.x() = x_[q1];
    quat_local_.y() = x_[q2];
    quat_local_.z() = x_[q3];
    quat_local_.normalize();

    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    Eigen::Vector3d vel_local(x_[U], x_[V], x_[W]);

    // Convert local coordinates to world coordinates
    state_->quat() = quat_local_ * quat_world_;
    state_->quat().normalize();
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << state_->quat().toRotationMatrix() * vel_local;

    speed_ = vel_local.norm();

    // Updating the odeVals vector with the difference that occured from the step function
    odeVal_[q0] = x_[q0] - odeVal_[q0];
    odeVal_[q1] = x_[q1] - odeVal_[q1];
    odeVal_[q2] = x_[q2] - odeVal_[q2];
    odeVal_[q3] = x_[q3] - odeVal_[q3];

    odeVal_[U] = x_[U] - odeVal_[U];
    odeVal_[V] = x_[V] - odeVal_[V];
    odeVal_[W] = x_[W] - odeVal_[W];

    odeVal_[Xw] = x_[Xw] - odeVal_[Xw];
    odeVal_[Yw] = x_[Yw] - odeVal_[Yw];
    odeVal_[Zw] = x_[Zw] - odeVal_[Zw];

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"speed", speed_},
                {"Uw", state_->vel()(0)},
                {"Vw", state_->vel()(1)},
                {"Ww", state_->vel()(2)}});
    }

    return true;
}

bool DubinsAirplane3D::step(double t, double dt, vector_t odevect_) {
    std::cout << "In the vector offset applied for state change step function" << std::endl;

    // Get inputs and saturate
    speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
    pitch_ = vars_.input(desired_pitch_idx_);
    roll_ = vars_.input(desired_roll_idx_);

    //Final value that can be fed into the state position vector... maybe we just add to the state pos vector instead of this and then feeding it in...
    x_[Xw] = state_->pos()(0) + odevect_[Xw];
    x_[Yw] = state_->pos()(1) + odevect_[Yw];
    x_[Zw] = state_->pos()(2) + odevect_[Zw];

    //Final value that can be fed into the state position vector
    state_->quat().set(roll_, pitch_, state_->quat().yaw());
    quat_local_ = state_->quat() * quat_world_inverse_;
    quat_local_.normalize();
    quat_local_.w() += odevect_[q0];
    quat_local_.x() += odevect_[q1];
    quat_local_.y() += odevect_[q2];
    quat_local_.z() += odevect_[q3];
    quat_local_.normalize();

    Eigen::Vector3d force_body = quat_local_.rotate_reverse(ext_force_);
    Eigen::Vector3d ext_moment_body = ext_moment_;
    ext_force_ = Eigen::Vector3d::Zero();
    ext_moment_ = Eigen::Vector3d::Zero();

    //Final value that can be fed into the local velocity calculation
    x_[U] = (speed_ + force_body(0) / mass_) + odevect_[U];
    x_[V] = (force_body(1) / mass_) + odevect_[V];
    x_[W] = (force_body(2) / mass_) + odevect_[W];

    Eigen::Vector3d vel_local(x_[U], x_[V], x_[W]);

    // Convert local coordinates to world coordinates
    state_->quat() = quat_local_ * quat_world_; //Need to check if the quat world value is updated by the step function... otherwise need to hold the difference updated in the vector
    state_->quat().normalize();
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << state_->quat().toRotationMatrix() * vel_local;

    speed_ = vel_local.norm();

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"speed", speed_},
                {"Uw", state_->vel()(0)},
                {"Vw", state_->vel()(1)},
                {"Ww", state_->vel()(2)}});
    }

    return true;
}

std::vector<double> DubinsAirplane3D::getOdeStepVal(){
    return odeVal_;
}

///////////////////////////////////////////////////////////////////////////////////////
// Original ode step function
///////////////////////////////////////////////////////////////////////////////////////
bool DubinsAirplane3D::step(double t, double dt) {
    std::cout << "In the original step function" << std::endl;

    // Get inputs and saturate
    speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
    pitch_ = vars_.input(desired_pitch_idx_);
    roll_ = vars_.input(desired_roll_idx_);

    x_[Uw] = state_->vel()(0);
    x_[Vw] = state_->vel()(1);
    x_[Ww] = state_->vel()(2);

    x_[Xw] = state_->pos()(0);
    x_[Yw] = state_->pos()(1);
    x_[Zw] = state_->pos()(2);

    // state_->quat().normalize();
    state_->quat().set(roll_, pitch_, state_->quat().yaw());
    quat_local_ = state_->quat() * quat_world_inverse_;
    quat_local_.normalize();
    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    Eigen::Vector3d force_body = quat_local_.rotate_reverse(ext_force_);
    Eigen::Vector3d ext_moment_body = ext_moment_;
    ext_force_ = Eigen::Vector3d::Zero();
    ext_moment_ = Eigen::Vector3d::Zero();

    x_[U] = speed_ + force_body(0) / mass_;
    x_[V] = force_body(1) / mass_;
    x_[W] = force_body(2) / mass_;

    double turn_rate = 0;
    if (std::abs(speed_) >= std::numeric_limits<double>::epsilon()) {
        turn_rate = -g_ / speed_ * tan(roll_);
    }

    x_[P] = ext_moment_body(0) / mass_;
    x_[Q] = ext_moment_body(1) / mass_;
    x_[R] = ext_moment_body(2) / mass_ + turn_rate;

    ode_step(dt);

    // Normalize quaternion
    quat_local_.w() = x_[q0];
    quat_local_.x() = x_[q1];
    quat_local_.y() = x_[q2];
    quat_local_.z() = x_[q3];
    quat_local_.normalize();

    x_[q0] = quat_local_.w();
    x_[q1] = quat_local_.x();
    x_[q2] = quat_local_.y();
    x_[q3] = quat_local_.z();

    Eigen::Vector3d vel_local(x_[U], x_[V], x_[W]);

    // Convert local coordinates to world coordinates
    state_->quat() = quat_local_ * quat_world_;
    state_->quat().normalize();
    state_->pos() << x_[Xw], x_[Yw], x_[Zw];
    state_->vel() << state_->quat().toRotationMatrix() * vel_local;

    speed_ = vel_local.norm();

    if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"speed", speed_},
                {"Uw", state_->vel()(0)},
                {"Vw", state_->vel()(1)},
                {"Ww", state_->vel()(2)}});
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
// Only returning true to see how run-time differs
///////////////////////////////////////////////////////////////////////////////////////
//bool DubinsAirplane3D::step(double t, double dt) { return true; }

///////////////////////////////////////////////////////////////////////////////////////
// Executing simple array index updates to see how run-time differs
///////////////////////////////////////////////////////////////////////////////////////
// bool DubinsAirplane3D::step(double t, double dt) {

//     x_[Uw] = 1;
//     x_[Vw] = 1;
//     x_[Ww] = 1;

//     x_[Xw] = 1;
//     x_[Yw] = 1;
//     x_[Zw] = 1;

//     x_[U] = 1;
//     x_[V] = 1;
//     x_[W] = 1;

//     x_[P] = 1;
//     x_[Q] = 1;
//     x_[R] = 1;

//     x_[q0] = 1;
//     x_[q1] = 1;
//     x_[q2] = 1;
//     x_[q3] = 1;

//     return true;
// }

///////////////////////////////////////////////////////////////////////////////////////
// Original with timing incorporated
///////////////////////////////////////////////////////////////////////////////////////
// bool DubinsAirplane3D::step(double t, double dt) {
//     auto startTot = high_resolution_clock::now();

//     //Commenting out lines that do not directly affect the state only decreases the wall time by 0.023 seconds
//     auto start = high_resolution_clock::now();

//     // Get inputs and saturate
//     speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
//     pitch_ = vars_.input(desired_pitch_idx_);
//     roll_ = vars_.input(desired_roll_idx_);

//     // Nat - These are not used anywhere, but maybe like below Xw they are somewhere
//     x_[Uw] = state_->vel()(0);
//     x_[Vw] = state_->vel()(1);
//     x_[Ww] = state_->vel()(2);
//     // Nat - end

//     // Nat - no further calculations are done on Xw, Yw, and Zw. Then, state_->pos is set back
//     // to these values... is this necessary?
//     x_[Xw] = state_->pos()(0);
//     x_[Yw] = state_->pos()(1);
//     x_[Zw] = state_->pos()(2);

//     //std::cout << "State position is: (" << x_[Xw] << ", " << x_[Yw] << ", " << x_[Zw] << ") for id: " << parent()->id().id() << std::endl;

//     // Nat - end

//     //state_->quat().normalize();
//     state_->quat().set(roll_, pitch_, state_->quat().yaw()); // Does setting the state quaternion update the state pos? Maybe that is why it is updating and I'm not seeing the change in value.. but the state quaternion isnt changing value here... so I'm not sure if that's the reason why
//     quat_local_ = state_->quat() * quat_world_inverse_;
//     quat_local_.normalize();
//     x_[q0] = quat_local_.w();
//     x_[q1] = quat_local_.x();
//     x_[q2] = quat_local_.y();
//     x_[q3] = quat_local_.z();

//     Eigen::Vector3d force_body = quat_local_.rotate_reverse(ext_force_);
//     Eigen::Vector3d ext_moment_body = ext_moment_;
//     ext_force_ = Eigen::Vector3d::Zero();
//     ext_moment_ = Eigen::Vector3d::Zero();

//     x_[U] = speed_ + force_body(0) / mass_;
//     x_[V] = force_body(1) / mass_;
//     x_[W] = force_body(2) / mass_;

//     double turn_rate = 0;
//     if (std::abs(speed_) >= std::numeric_limits<double>::epsilon()) {
//         turn_rate = -g_ / speed_ * tan(roll_);
//     }

//     x_[P] = ext_moment_body(0) / mass_;   
//     x_[Q] = ext_moment_body(1) / mass_;
//     x_[R] = ext_moment_body(2) / mass_ + turn_rate;
    
//     auto stop = high_resolution_clock::now();
        
//     auto startode = high_resolution_clock::now();

//     auto duration = duration_cast<microseconds>(stop - start);
//     std::cout << "Execution time before ode_step: " << duration.count() << std::endl;
    
//     std::cout << "State position before ODE is: (" << x_[Xw] << ", " << x_[Yw] << ", " << x_[Zw] << ") for id: " << parent()->id().id() << std::endl;
//     //std::cout << "State position value: (" << state_->pos()(0) << ", " << state_->pos()(1) << ", " << state_->pos()(2) << ")" << std::endl; //Cannot use the state variable in the matrix, have to use the local x[<blah>] value because that is what gets updated by the ode step function
    
//     //This ODE step function is what causes the local variables to update
//     ode_step(dt); //see if this needs to be called for each entity, or if it can be called once
    
    
//     std::cout << "State position after ODE is: (" << x_[Xw] << ", " << x_[Yw] << ", " << x_[Zw] << ") for id: " << parent()->id().id() << std::endl;

//      auto stopode = high_resolution_clock::now();
//      auto startlast = high_resolution_clock::now();

//      auto durationode = duration_cast<microseconds>(stopode - startode);
//      std::cout << "Execution time of ode_step: " << durationode.count() << std::endl;

//     //std::cout << "State position value: (" << state_->pos()(0) << ", " << state_->pos()(1) << ", " << state_->pos()(2) << ")" << std::endl;

//     // Normalize quaternion
//     quat_local_.w() = x_[q0]; // Nat - the back and forth of setting these variables seems redundant, why do we do this?
//     quat_local_.x() = x_[q1];
//     quat_local_.y() = x_[q2];
//     quat_local_.z() = x_[q3];
//     quat_local_.normalize();

//     x_[q0] = quat_local_.w(); // Nat - what are these local variables used for? Not seeing anything else in this function... I looked into entity interaction plugins, and am not seeing use of x_[q0] through q3
//     x_[q1] = quat_local_.x(); // the couts are not picking up any changes in values, maybe it is because they are only going straight and not turning?
//     x_[q2] = quat_local_.y();
//     x_[q3] = quat_local_.z();

//     Eigen::Vector3d vel_local(x_[U], x_[V], x_[W]);

//     // Convert local coordinates to world coordinates
//     state_->quat() = quat_local_ * quat_world_;
//     state_->quat().normalize(); // Must be normalized in order to compute vector rotations and be represented as a valid 3D orientation
//     state_->pos() << x_[Xw], x_[Yw], x_[Zw];

//     state_->vel() << state_->quat().toRotationMatrix() * vel_local;

//     speed_ = vel_local.norm();

//     if (write_csv_) {
//         // Log state to CSV
//         csv_.append(sc::CSV::Pairs{
//                 {"t", t},
//                 {"x", x_[Xw]},
//                 {"y", x_[Yw]},
//                 {"z", x_[Zw]},
//                 {"U", x_[U]},
//                 {"V", x_[V]},
//                 {"W", x_[W]},
//                 {"P", x_[P]},
//                 {"Q", x_[Q]},
//                 {"R", x_[R]},
//                 {"roll", state_->quat().roll()},
//                 {"pitch", state_->quat().pitch()},
//                 {"yaw", state_->quat().yaw()},
//                 {"speed", speed_},
//                 {"Uw", state_->vel()(0)},
//                 {"Vw", state_->vel()(1)},
//                 {"Ww", state_->vel()(2)}});
//     }

//     auto stoplast = high_resolution_clock::now();
//     auto durationlast = duration_cast<microseconds>(stoplast - startlast);
//     std::cout << "Execution time after ode_step: " << durationlast.count() << std::endl;

//     auto stopTot = high_resolution_clock::now();
//     auto durationfinal = duration_cast<microseconds>(stopTot - startTot);
//     std::cout << "Total execution time: " << durationfinal.count() << std::endl;

//     return true;
// }

bool DubinsAirplane3D::step(double t, double dt, int iteration) {
        
    //Commenting out lines that do not directly affect the state only decreases the wall time by 0.023 seconds
    
    // Get inputs and saturate
    if(iteration == 1){
        std::cout << "In the first iteration" << std::endl;
        speed_ = boost::algorithm::clamp(vars_.input(desired_speed_idx_), speed_min_, speed_max_);
        pitch_ = vars_.input(desired_pitch_idx_);
        roll_ = vars_.input(desired_roll_idx_);

        // Nat - These are not used anywhere, but maybe like below Xw they are somewhere
        x_[Uw] = state_->vel()(0);
        x_[Vw] = state_->vel()(1);
        x_[Ww] = state_->vel()(2);
        // Nat - end

        // Nat - no further calculations are done on Xw, Yw, and Zw. Then, state_->pos is set back
        // to these values... is this necessary?
        x_[Xw] = state_->pos()(0);
        x_[Yw] = state_->pos()(1);
        x_[Zw] = state_->pos()(2);

        //std::cout << "State position is: (" << x_[Xw] << ", " << x_[Yw] << ", " << x_[Zw] << ") for id: " << parent()->id().id() << std::endl;

        // Nat - end

        // state_->quat().normalize();
        state_->quat().set(roll_, pitch_, state_->quat().yaw()); // Does setting the state quaternion update the state pos? Maybe that is why it is updating and I'm not seeing the change in value.. but the state quaternion isnt changing value here... so I'm not sure if that's the reason why
        quat_local_ = state_->quat() * quat_world_inverse_;
        quat_local_.normalize();
        x_[q0] = quat_local_.w();
        x_[q1] = quat_local_.x();
        x_[q2] = quat_local_.y();
        x_[q3] = quat_local_.z();

        Eigen::Vector3d force_body = quat_local_.rotate_reverse(ext_force_);
        Eigen::Vector3d ext_moment_body = ext_moment_;
        ext_force_ = Eigen::Vector3d::Zero();
        ext_moment_ = Eigen::Vector3d::Zero();

        x_[U] = speed_ + force_body(0) / mass_;
        x_[V] = force_body(1) / mass_;
        x_[W] = force_body(2) / mass_;

        double turn_rate = 0;
        if (std::abs(speed_) >= std::numeric_limits<double>::epsilon()) {
            turn_rate = -g_ / speed_ * tan(roll_);
        }

        x_[P] = ext_moment_body(0) / mass_;
        x_[Q] = ext_moment_body(1) / mass_;
        x_[R] = ext_moment_body(2) / mass_ + turn_rate;

        std::cout << "State position before ODE is: (" << x_[Xw] << ", " << x_[Yw] << ", " << x_[Zw] << ") for id: " << parent()->id().id() << std::endl;
        //std::cout << "State position value: (" << state_->pos()(0) << ", " << state_->pos()(1) << ", " << state_->pos()(2) << ")" << std::endl; //Cannot use the state variable in the matrix, have to use the local x[<blah>] value because that is what gets updated by the ode step function
        
        //Only call the ODE step once and see if it updates for all variables
        //I called the ODE only once, using the last entity as reference, and it does not update
        // all variables... it must be called by each object
            //This ODE step function is what causes the local variables to update

        ode_step(dt); //see if this needs to be called for each entity, or if it can be called once
    }

    else {

        std::cout << "In the second iteration" << std::endl;
        std::cout << "State position after ODE is: (" << x_[Xw] << ", " << x_[Yw] << ", " << x_[Zw] << ") for id: " << parent()->id().id() << std::endl;
        //std::cout << "State position value: (" << state_->pos()(0) << ", " << state_->pos()(1) << ", " << state_->pos()(2) << ")" << std::endl;


        // Normalize quaternion
        quat_local_.w() = x_[q0]; // Nat - the back and forth of setting these variables seems redundant, why do we do this?
        quat_local_.x() = x_[q1];
        quat_local_.y() = x_[q2];
        quat_local_.z() = x_[q3];
        quat_local_.normalize();

        x_[q0] = quat_local_.w(); 
        x_[q1] = quat_local_.x(); 
        x_[q2] = quat_local_.y();
        x_[q3] = quat_local_.z();

        Eigen::Vector3d vel_local(x_[U], x_[V], x_[W]);

        // Convert local coordinates to world coordinates
        state_->quat() = quat_local_ * quat_world_;
        state_->quat().normalize(); // Must be normalized in order to compute vector rotations and be represented as a valid 3D orientation
        state_->pos() << x_[Xw], x_[Yw], x_[Zw];

        state_->vel() << state_->quat().toRotationMatrix() * vel_local;

        speed_ = vel_local.norm();

        if (write_csv_) {
        // Log state to CSV
        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", x_[Xw]},
                {"y", x_[Yw]},
                {"z", x_[Zw]},
                {"U", x_[U]},
                {"V", x_[V]},
                {"W", x_[W]},
                {"P", x_[P]},
                {"Q", x_[Q]},
                {"R", x_[R]},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"speed", speed_},
                {"Uw", state_->vel()(0)},
                {"Vw", state_->vel()(1)},
                {"Ww", state_->vel()(2)}});
    }
    }

    return true;
}

void DubinsAirplane3D::model(const vector_t &x , vector_t &dxdt , double t) {
    dxdt[U] = 0;
    dxdt[V] = 0;
    dxdt[W] = 0;

    dxdt[P] = 0;
    dxdt[Q] = 0;
    dxdt[R] = 0;
    double lambda = 1 - (pow(x[q0], 2) + pow(x[q1], 2) + pow(x[q2], 2) + pow(x[q3], 2));
    dxdt[q0] = -0.5 * (x[q1]*x[P] + x[q2]*x[Q] + x[q3]*x[R]) + lambda * x[q0];
    dxdt[q1] = +0.5 * (x[q0]*x[P] + x[q2]*x[R] - x[q3]*x[Q]) + lambda * x[q1];
    dxdt[q2] = +0.5 * (x[q0]*x[Q] + x[q3]*x[P] - x[q1]*x[R]) + lambda * x[q2];
    dxdt[q3] = +0.5 * (x[q0]*x[R] + x[q1]*x[Q] - x[q2]*x[P]) + lambda * x[q3];

    // Local position / velocity to global
    // Normalize quaternion
    sc::Quaternion quat(x[q0], x[q1], x[q2], x[q3]);
    quat.normalize();

    quat = quat * quat_world_;
    quat.normalize();

    // Convert local positions and velocities into global coordinates
    Eigen::Matrix3d rot = quat.toRotationMatrix();

    Eigen::Vector3d vel_local(x[U], x[V], x[W]);
    Eigen::Vector3d vel_world = rot * vel_local;
    dxdt[Xw] = vel_world(0);
    dxdt[Yw] = vel_world(1);
    dxdt[Zw] = vel_world(2);
}
} // namespace motion
} // namespace scrimmage
