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
 * @author Natalie Rakoski <natalie.rakoski@gtri.gatech.edu>
 * @date 06 May 2020
 * @version 0.2.8
 * @brief Requests AirSim data over RPC and publishes it as SCRIMMAGE messages.
 * @section Requests AirSim data over RPC and publishes it as SCRIMMAGE messages.
 * Requests AirSim data over RPC and publishes it as SCRIMMAGE messages.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_

#include <scrimmage/common/CSV.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/sensor/Sensor.h>

#include <list>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <random>
#include <string>
#include <vector>

// Eigen libraries
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
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

    std::string vehicle_name = "none";
    std::string cam_name = "none";
    std::string img_type_name = "none";
    int img_type_number = 0;
    int height = 144;  // 288
    int width = 256;   // 512
    int fov = 60;
    bool pixels_as_float = false;

    friend std::ostream& operator<<(std::ostream& os, const CameraConfig& c) {
        os << "Camera_Name=" << c.cam_name;
        os << ", Image_Type_Num=" << c.img_type_number;
        os << ", Image_Type_Name=" << c.img_type_name;
        os << ", Height=" << c.height;
        os << ", Width=" << c.width;
        os << ", FOV=" << c.fov;
        return os;
    }
};

class AirSimImageType {
 public:
    cv::Mat img;
    CameraConfig camera_config;
    std::string vehicle_name;
    Eigen::Isometry3f vehicle_pose_world_NED;
    Eigen::Isometry3f camera_pose_world_NED;
    ~AirSimImageType();
};
AirSimImageType::~AirSimImageType(void) {
    this->img.release();
}

class AirSimLidarType {
 public:
    msr::airlib::LidarData lidar_data;
    std::string vehicle_name;
    std::string lidar_name;
    Eigen::Isometry3f vehicle_pose_world_NED;
    Eigen::Isometry3f lidar_pose_world_NED;
};

class AirSimImuType {
 public:
    msr::airlib::ImuBase::Output imu_data;
    int frame_num = 0;
    std::string vehicle_name;
    std::string imu_name;
    Eigen::Isometry3f vehicle_pose_world_NED;
    Eigen::Isometry3f imu_pose_world_NED;
};

class AirSimSensor : public scrimmage::Sensor {
 public:
    AirSimSensor();
    void init(std::map<std::string, std::string>& params) override;
    bool step() override;
    void close(double t) override;

 protected:
    std::string vehicle_name_ = "none";
    bool save_data(MessagePtr<std::vector<AirSimImageType>>& im_msg, StatePtr& state,
                   int frame_num);
    scrimmage::CSV csv;
    int airsim_frame_num_ = 0;

    // Images
    void parse_camera_configs(std::map<std::string, std::string>& params);
    std::list<CameraConfig> cam_configs_;
    std::thread request_images_thread_;
    void request_images();
    PublisherPtr img_pub_;
    scrimmage::MessagePtr<std::vector<AirSimImageType>> img_msg_ = nullptr;
    scrimmage::MessagePtr<std::vector<AirSimImageType>> im_msg = nullptr;
    std::mutex img_msg_mutex_;
    bool new_image_ = false;
    std::mutex new_image_mutex_;

    // LIDAR
    void parse_lidar_configs(std::map<std::string, std::string>& params);
    std::vector<std::string> lidar_names_;
    std::thread request_lidar_thread_;
    void request_lidar();
    PublisherPtr lidar_pub_;
    scrimmage::MessagePtr<std::vector<AirSimLidarType>> lidar_msg_ = nullptr;
    scrimmage::MessagePtr<std::vector<AirSimLidarType>> lidar_msg = nullptr;
    std::mutex lidar_msg_mutex_;
    bool new_lidar_ = false;
    std::mutex new_lidar_mutex_;

    // IMU
    void parse_imu_configs(std::map<std::string, std::string>& params);
    std::vector<std::string> imu_names_;
    std::thread request_imu_thread_;
    void request_imu();
    PublisherPtr imu_pub_;
    scrimmage::MessagePtr<std::vector<AirSimImuType>> imu_msg_ = nullptr;
    scrimmage::MessagePtr<std::vector<AirSimImuType>> imu_msg = nullptr;
    std::mutex imu_msg_mutex_;
    bool new_imu_ = false;
    std::mutex new_imu_mutex_;

    // Step FN
    bool running_ = true;
    std::mutex running_mutex_;
    scrimmage::Angles enu_to_ned_yaw_;

    // RPC
    std::shared_ptr<msr::airlib::RpcLibClientBase> sim_client_;
    bool client_connected_;
    std::string airsim_ip_;
    uint16_t airsim_port_;
    float airsim_timeout_s_;

    // Mission File Variables
    bool save_airsim_data_ = true;
    bool get_image_data_ = true;
    bool get_lidar_data_ = true;
    bool get_imu_data_ = true;
    // period at which the data acquisition is run [seconds]
    // double data_acquisition_period_ = .1;
    double image_acquisition_period_ = .1;
    double lidar_acquisition_period_ = .1;
    double imu_acquisition_period_ = .1;

 private:
};
}  // namespace sensor
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_AIRSIMSENSOR_AIRSIMSENSOR_H_
