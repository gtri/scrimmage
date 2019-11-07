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
#include <scrimmage/math/Angles.h>
#include <scrimmage/common/CSV.h>

#include <random>
#include <list>
#include <map>
#include <string>
#include <vector>
#include <memory>

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
using ang = scrimmage::Angles;

namespace scrimmage {
namespace sensor {
class CameraConfig {
 public:
        msr::airlib::ImageCaptureBase::ImageType img_type =
            msr::airlib::ImageCaptureBase::ImageType::Scene;

        std::string name = "none";
        int number = 0;
        int height = 144;
        int width = 256;
        std::string img_type_name = "none";

        friend std::ostream& operator<<(std::ostream& os,
                                        const CameraConfig& c) {
            os << "Name=" << c.name;
            os << ", Number=" << c.number;
            os << ", Height=" << c.height;
            os << ", Width=" << c.width;
            os << ", Image_Type=" << c.img_type_name;
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
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;
    void close(double t) override;

 protected:
    std::thread request_images_thread_;
    void request_images();
    scrimmage::MessagePtr<std::vector<AirSimSensorType>> img_msg_ = nullptr;
    std::mutex img_msg_mutex_;

    bool running_ = true;
    std::mutex running_mutex_;

    std::shared_ptr<msr::airlib::MultirotorRpcLibClient> sim_client_;
    bool client_connected_;
    std::string airsim_ip_;
    int airsim_port_;
    int airsim_timeout_ms_;
    std::list<CameraConfig> cam_configs_;
    scrimmage::Angles enu_to_ned_yaw_;
    PublisherPtr pub_;

    bool save_images(MessagePtr<std::vector<AirSimSensorType>>& msg, StatePtr& state);

    bool save_airsim_data_ = false; // should be set to false by default but xml isn't working
    int airsim_frame_num_ = 0;
    scrimmage::CSV csv;

 private:
};
} // namespace sensor
} // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_
