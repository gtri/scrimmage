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
    csv_.close_output();
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

        if (!csv_.open_output(trajectory_filename_)) {
            cout << "Unable to open log file" << endl;
            return;
        }
        csv_.set_column_headers("t, x, y, z, vx, vy, vz,"
                                "roll, pitch, yaw,"
                                "x_d, y_d, z_d, vx_d, vy_d, vz_d,"
                                "roll_d, pitch_d, yaw_d");

    } else {
        this->set_is_controlling(true);

        if (!csv_.read_csv(trajectory_filename_)) {
            cout << "Failed to read CSV file: " << trajectory_filename_
                 << endl;
        }

        for (size_t r = 0; r < csv_.rows(); r++) {
            scrimmage::State desired_state;
            desired_state.pos() << csv_.at(r, "x_d"),
                csv_.at(r, "y_d"),
                csv_.at(r, "z_d");

            desired_state.vel() << csv_.at(r, "vx_d"),
                csv_.at(r, "vy_d"),
                csv_.at(r, "vz_d");

            scrimmage::Quaternion quat(csv_.at(r, "roll_d"),
                                       csv_.at(r, "pitch_d"),
                                       csv_.at(r, "yaw_d"));

            desired_state.set_quat(quat);

            TrajectoryPoint traj;
            traj.set_t(csv_.at(r, "t"));
            traj.set_desired_state(desired_state);
            trajs_.push_back(traj);
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

        csv_.append(sc::CSV::Pairs{
                {"t", t},
                {"x", state_->pos()(0)},
                {"y", state_->pos()(1)},
                {"z", state_->pos()(2)},
                {"vx", state_->vel()(0)},
                {"vy", state_->vel()(1)},
                {"vz", state_->vel()(2)},
                {"roll", state_->quat().roll()},
                {"pitch", state_->quat().pitch()},
                {"yaw", state_->quat().yaw()},
                {"x_d", (*(*it)->desired_state()).pos()(0)},
                {"y_d", (*(*it)->desired_state()).pos()(1)},
                {"z_d", (*(*it)->desired_state()).pos()(2)},
                {"vx_d", (*(*it)->desired_state()).vel()(0)},
                {"vy_d", (*(*it)->desired_state()).vel()(1)},
                {"vz_d", (*(*it)->desired_state()).vel()(2)},
                {"roll_d", (*(*it)->desired_state()).quat().roll()},
                {"pitch_d", (*(*it)->desired_state()).quat().pitch()},
                {"yaw_d", (*(*it)->desired_state()).quat().yaw()},
            });

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
