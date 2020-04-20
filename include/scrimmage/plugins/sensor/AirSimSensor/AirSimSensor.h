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

        std::string cam_name = "none";
        std::string img_type_name = "none";
        int img_type_number = 0;
        int height = 144; // 288
        int width = 256;  // 512
        Eigen::Matrix<float, 3, 1> cam_position_ENU; // cam_position{0.0, 0.0, 0.0};
        Eigen::Quaternion<float, 2> cam_orientation_ENU;
        bool pixels_as_float = false;

        friend std::ostream& operator<<(std::ostream& os,
                                        const CameraConfig& c) {
            os << ", Camera_Name=" << c.cam_name;
            os << ", Image_Type_Num=" << c.img_type_number;
            os << ", Image_Type_Name=" << c.img_type_name;
            os << ", Height=" << c.height;
            os << ", Width=" << c.width;
            os << ", Position= " << c.cam_position_ENU;
            os << ", Orientation= " << c.cam_orientation_ENU;
            return os;
        }
};

class AirSimImageType {
 public:
    cv::Mat img;
    CameraConfig camera_config;
    std::string vehicle_name;
};

class AirSimLidarType {
 public:
    msr::airlib::LidarData lidar_data;
    std::string vehicle_name;
    std::string lidar_name;
    Eigen::Matrix<float, 3, 1> lidar_position_ENU{0.0, 0.0, 0.0};
    Eigen::Quaternion<float, 2> lidar_orientation_ENU{0.0, 0.0, 0.0, 0.0};
};

class AirSimSensor : public scrimmage::Sensor {
 public:
    AirSimSensor();
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;
    void close(double t) override;

 protected:
    std::thread request_images_thread_;
    void parse_camera_configs(std::map<std::string, std::string> &params);
    void request_images();
    scrimmage::MessagePtr<std::vector<AirSimImageType>> img_msg_ = nullptr;
    scrimmage::MessagePtr<AirSimLidarType> lidar_msg_ = nullptr;
    std::mutex img_msg_mutex_;
    std::mutex lidar_msg_mutex_;

    bool running_ = true;
    std::mutex running_mutex_;

//    std::shared_ptr<msr::airlib::MultirotorRpcLibClient> sim_client_;
    std::shared_ptr<msr::airlib::RpcLibClientBase> sim_client_;
    bool client_connected_;
    std::string airsim_ip_;
    uint16_t airsim_port_;
    float airsim_timeout_s_;
    std::list<CameraConfig> cam_configs_;

    std::string vehicle_name_ = "robot1";
    std::string lidar_name_ = "lidar1";

    scrimmage::Angles enu_to_ned_yaw_;

    PublisherPtr img_pub_;
    PublisherPtr lidar_pub_;

    bool save_data(MessagePtr<std::vector<AirSimImageType>>& im_msg, StatePtr& state, int frame_num);

    bool save_airsim_data_ = true;
    bool get_image_data_ = true;
    bool get_lidar_data_ = true;

    bool new_lidar_ = false;
    std::mutex new_lidar_mutex_;
    bool new_image_ = false;
    std::mutex new_image_mutex_;

    // period at which the data acquisition is run [seconds]
    double data_acquisition_period_ = .1;

    int airsim_frame_num_ = 0;
    scrimmage::CSV csv;

 private:
};
} // namespace sensor
} // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_
