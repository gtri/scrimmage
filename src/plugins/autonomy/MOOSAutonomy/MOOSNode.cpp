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


#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <NodeRecord.h>

#include <scrimmage/plugins/autonomy/MOOSAutonomy/MOOSNode.h>

#include <string>

namespace sc = scrimmage;

namespace scrimmage {
namespace autonomy {

MOOSNode::MOOSNode() : appTick_(1), commsTick_(1), time_warp_(1),
                       deployed_(false) {
}

MOOSNode::~MOOSNode() {}

bool MOOSNode::ready() {
    deployed_mutex_.lock();
    bool ready = deployed_;
    deployed_mutex_.unlock();
    return ready;
}

sc::State MOOSNode::desired_state() {
    desired_mutex_.lock();
    sc::State s = desired_;
    desired_mutex_.unlock();
    return s;
}

void MOOSNode::set_time_warp(double warp) {
    time_warp_ = warp;
}

bool MOOSNode::OnNewMail(MOOSMSG_LIST &Mail) {
    MOOSMSG_LIST::iterator q;
    for (q = Mail.begin(); q != Mail.end(); ++q) {
        CMOOSMsg &msg = *q;
        std::string key = msg.GetKey();
        double dval = msg.GetDouble();

        if (key == "DESIRED_HEADING") {
            desired_mutex_.lock();
            desired_.quat().set(0, 0, sc::Angles::deg2rad(dval));
            desired_mutex_.unlock();
        } else if (key == "DESIRED_SPEED") {
            desired_mutex_.lock();
            desired_.vel() = Eigen::Vector3d::UnitX()*dval;
            desired_mutex_.unlock();
        } else if (key == "DESIRED_DEPTH") {
            desired_mutex_.lock();
            desired_.pos() = -Eigen::Vector3d::UnitZ()*dval;
            desired_mutex_.unlock();
        }  else if (key == "IVPHELM_STATE") {
            if (!deployed_) {
                Notify("DEPLOY", "true");
                Notify("RETURN", "false");
                Notify("MOOS_MANUAL_OVERRIDE", "false");
                deployed_mutex_.lock();
                deployed_ = true;
                deployed_mutex_.unlock();
            }
        }
    }
    return(true);
}

/*
  called by the base class when the application has made contact with
  the MOOSDB and a channel has been opened . Place code to specify what
  notifications you want to receive here .
*/
bool MOOSNode::OnConnectToServer() {
    DoRegistrations();
    SetMOOSTimeWarp(time_warp_);
    return true;
}

/*
  Called by the base class periodically. This is where you place code
  which does the work of the application
*/
bool MOOSNode::Iterate() {
    return true;
}

/*
  called by the base class before the first :: Iterate is called . Place
  startup code here − especially code which reads configuration data from the
  mission file
*/
bool MOOSNode::OnStartUp() {
    appTick_ = 1;
    commsTick_ = 1;

    if (!m_MissionReader.GetConfigurationParam("AppTick", appTick_)) {
        MOOSTrace("Warning, AppTick not set.\n");
    }

    if (!m_MissionReader.GetConfigurationParam("CommsTick", commsTick_)) {
        MOOSTrace("Warning, CommsTick not set.\n");
    }

    SetAppFreq(appTick_);
    SetCommsFreq(commsTick_);

    DoRegistrations();

    return true;
}

void MOOSNode::DoRegistrations() {
    Register("DESIRED_HEADING", 0.0);
    Register("DESIRED_SPEED", 0.0);
    Register("DESIRED_DEPTH", 0.0);
    Register("IVPHELM_STATE", 0.0);
}

bool MOOSNode::PublishNodeReport(NodeReportType_t report_type, std::string id,
                                 std::string sensor_id,
                                 double nav_x, double nav_y, double speed,
                                 double heading, double depth,
                                 std::string type, std::string mode,
                                 double time, std::string frame_number) {
    NodeRecord record;
    record.setName(id);
    record.setX(nav_x);
    record.setY(nav_y);
    record.setSpeed(speed);
    record.setHeading(heading);
    record.setDepth(depth);
    record.setType(type);
    record.setTimeStamp(MOOSTime());
    record.setProperty("FRAMENUM", frame_number);

    std::string moos_var;
    switch (report_type) {
    case OWNSHIP:
        moos_var = "NODE_REPORT_LOCAL";
        Notify("NAV_X", nav_x);
        Notify("NAV_Y", nav_y);
        Notify("NAV_HEADING", heading);
        Notify("NAV_SPEED", speed);
        Notify("NAV_DEPTH", depth);
        break;

    case TRUTH_CONTACT:
        moos_var = "NODE_REPORT";
        Notify(moos_var, record.getSpec());
        break;

    case SENSOR_CONTACT:
        record.setProperty("SENSOR_ID", sensor_id);
        moos_var = "SENSOR_CONTACT";
        Notify(moos_var, record.getSpec());
        break;

    default:
        break;
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
