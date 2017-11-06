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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOOSAUTONOMY_MOOSNODE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOOSAUTONOMY_MOOSNODE_H_

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <scrimmage/math/State.h>
#include <string.h>

#include <string>
#include <mutex> // NOLINT

namespace scrimmage {
namespace autonomy {
class MOOSNode : public CMOOSApp {
 public:
     // standard construction and destruction
	MOOSNode();
	virtual ~MOOSNode();

    bool ready();

    scrimmage::State desired_state();

    typedef enum NodeReportType {
        OWNSHIP = 0,
        TRUTH_CONTACT,
        SENSOR_CONTACT
    } NodeReportType_t;

    void set_time_warp(double warp);

    bool PublishNodeReport(NodeReportType_t report_type, std::string id,
                           std::string sensor_id,
                           double nav_x, double nav_y, double speed,
                           double heading, double depth, std::string type,
                           std::string mode, double time,
                           std::string frame_number);

 protected:
	double appTick_;
	double commsTick_;
    double time_warp_;

    bool deployed_;
    std::mutex deployed_mutex_;

	// where we handle new mail
	bool OnNewMail(MOOSMSG_LIST &NewMail);

	// where we do the work
	bool Iterate();

	// called when we connect to the server
	bool OnConnectToServer();

	// called when we are starting up..
	bool OnStartUp();

    void DoRegistrations();

    std::mutex desired_mutex_;
    scrimmage::State desired_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MOOSAUTONOMY_MOOSNODE_H_
