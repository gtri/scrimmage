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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAJECTORYRECORDPLAYBACK_TRAJECTORYRECORDPLAYBACK_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAJECTORYRECORDPLAYBACK_TRAJECTORYRECORDPLAYBACK_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/CSV.h>
#include <scrimmage/plugins/autonomy/TrajectoryRecordPlayback/TrajectoryPoint.h>

#include <fstream>
#include <list>
#include <map>
#include <string>

namespace scrimmage {
namespace autonomy {
class TrajectoryRecordPlayback : public scrimmage::Autonomy {
 public:
    TrajectoryRecordPlayback();
    ~TrajectoryRecordPlayback();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
 protected:
    std::list<TrajectoryPoint> trajs_;
    std::list<TrajectoryPoint>::iterator it_traj_;
    bool enable_playback_;
    bool remove_at_end_;
    std::string trajectory_filename_;

    CSV csv_;

 private:
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_TRAJECTORYRECORDPLAYBACK_TRAJECTORYRECORDPLAYBACK_H_
