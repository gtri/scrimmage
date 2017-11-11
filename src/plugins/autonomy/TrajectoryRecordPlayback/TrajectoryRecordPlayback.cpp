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
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugins/autonomy/TrajectoryRecordPlayback/TrajectoryRecordPlayback.h>
#include <scrimmage/plugins/autonomy/TrajectoryRecordPlayback/TrajectoryPoint.h>

#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::TrajectoryRecordPlayback,
                TrajectoryRecordPlayback_plugin)

namespace scrimmage {
namespace autonomy {

TrajectoryRecordPlayback::TrajectoryRecordPlayback()
    : enable_playback_(false), remove_at_end_(true),
      trajectory_filename_("trajectory.txt") { }

TrajectoryRecordPlayback::~TrajectoryRecordPlayback() {
    if (file_out_.is_open()) {
        file_out_.close();
    }

    if (file_in_.is_open()) {
        file_in_.close();
    }
}

void TrajectoryRecordPlayback::init(std::map<std::string,
                                    std::string> &params) {

    trajectory_filename_ = sc::get<std::string>("trajectory_filename", params,
                                                "trajectory.txt");
    trajectory_filename_ = parent_->mp()->root_log_dir() + "/" +
        trajectory_filename_;

    remove_at_end_ = sc::get<bool>("remove_at_end", params, true);

    enable_playback_ = sc::get<bool>("enable_playback", params, false);
    if (!enable_playback_) {
        this->set_is_controlling(false);

        file_out_.open(trajectory_filename_, std::ios::out | std::ios::trunc);
        if (!file_out_.is_open()) {
            cout << "Unable to open log file" << endl;
            return;
        }

        file_out_ << "t, x, y, z, vx, vy, vz, r, p, y, x_d, y_d, z_d, vx_d, vy_d, vz_d, r_d, p_d, y_d" << endl;

    } else {
        this->set_is_controlling(true);

        // Load the trajectory from a file
        file_in_.open(trajectory_filename_, std::ios::in);

        if (!file_in_.is_open()) {
            cout << "Unable to open log file" << endl;
            return;
        }

        std::string line;

        // Ignore the first line, it is a header
        std::getline(file_in_, line);

        while (std::getline(file_in_, line)) {
            std::vector<std::string> tokens;
            boost::split(tokens, line, boost::is_any_of(","));

            if (tokens.size() == 19) {
                scrimmage::State desired_state;
                desired_state.pos() << std::stod(tokens[10]),
                    std::stod(tokens[11]), std::stod(tokens[12]);

                desired_state.vel() << std::stod(tokens[13]),
                    std::stod(tokens[14]), std::stod(tokens[15]);

                scrimmage::Quaternion quat(std::stod(tokens[16]),
                                           std::stod(tokens[17]),
                                           std::stod(tokens[18]));

                TrajectoryPoint traj;
                traj.set_desired_state(desired_state);
                trajs_.push_back(traj);
            }
        }
        it_traj_ = trajs_.begin();
    }
}

bool TrajectoryRecordPlayback::step_autonomy(double t, double dt) {
    if (!enable_playback_) {
        // Get previous autonomy's desired_state
        auto it = std::find_if(parent_->autonomies().rbegin(),
                               parent_->autonomies().rend(),
            [&](auto autonomy) {return autonomy->get_is_controlling();});

        // Record the trajectory
        file_out_ << std::to_string(t) << ", "
                  << std::to_string(state_->pos()(0)) << ", "
                  << std::to_string(state_->pos()(1)) << ", "
                  << std::to_string(state_->pos()(2)) << ", "
                  << std::to_string(state_->vel()(0)) << ", "
                  << std::to_string(state_->vel()(1)) << ", "
                  << std::to_string(state_->vel()(2)) << ", "
                  << std::to_string(state_->quat().roll()) << ", "
                  << std::to_string(state_->quat().pitch()) << ", "
                  << std::to_string(state_->quat().yaw()) << ", "
                  << std::to_string((*(*it)->desired_state()).pos()(0)) << ", "       // 10
                  << std::to_string((*(*it)->desired_state()).pos()(1)) << ", "       // 11
                  << std::to_string((*(*it)->desired_state()).pos()(2)) << ", "       // 12
                  << std::to_string((*(*it)->desired_state()).vel()(0)) << ", "       // 13
                  << std::to_string((*(*it)->desired_state()).vel()(1)) << ", "       // 14
                  << std::to_string((*(*it)->desired_state()).vel()(2)) << ", "       // 15
                  << std::to_string((*(*it)->desired_state()).quat().roll()) << ", "  // 16
                  << std::to_string((*(*it)->desired_state()).quat().pitch()) << ", " // 17
                  << std::to_string((*(*it)->desired_state()).quat().yaw())           // 18
                  << endl;

        return true;
    }

    // Playback the trajectory
    (*desired_state_) = it_traj_->desired_state();

    // Increment trajectory reference
    ++it_traj_;

    if (it_traj_ == trajs_.end()) {
        if (remove_at_end_) {
            // Remove entity from simulation
            parent_->set_health_points(0);
        } else {
            // Remain at last trajectory command if end is reached
            --it_traj_;
        }
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
