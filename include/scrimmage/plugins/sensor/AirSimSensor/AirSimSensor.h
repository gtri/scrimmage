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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_

#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/Contact.h>

#include <iostream>
#include <random>
#include <list>
#include <map>
#include <string>

#include <opencv2/core/core.hpp>

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

using std::cout;
using std::endl;

class CameraConfig {
 public:
        msr::airlib::VehicleCameraBase::ImageType img_type =
            msr::airlib::VehicleCameraBase::ImageType::Scene;;
        std::string name = "none";
        int number = 0;
        int height = 144;
        int width = 256;

        friend std::ostream& operator<<(std::ostream& os,
                                        const CameraConfig& c) {
            os << "Name=" << c.name;
            os << ", Number=" << c.number;
            os << ", Height=" << c.height;
            os << ", Width=" << c.width;
            return os;
        }
};

class AirSimSensorType {
 public:
    cv::Mat img;
    CameraConfig camera_config;
};

class AirSimSensor : public scrimmage::Sensor {
 public:
    AirSimSensor();
    virtual void init(std::map<std::string, std::string> &params);
    virtual boost::optional<scrimmage::MessageBasePtr> sensor_msg(double t);
 protected:
    std::shared_ptr<msr::airlib::MultirotorRpcLibClient> sim_client_;
    bool client_connected_;
    std::string airsim_ip_;
    int airsim_port_;
    int airsim_timeout_ms_;
    std::list<CameraConfig> cam_configs_;

 private:
};

#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_
