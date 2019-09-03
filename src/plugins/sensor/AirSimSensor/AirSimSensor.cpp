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

#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/Angles.h>

#include <iostream>
#include <memory>
#include <limits>
#include <chrono> // NOLINT
#include <thread> // NOLINT
#include <mutex> // NOLINT

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp> // for boost::split

using std::cout;
using std::endl;

using ang = scrimmage::Angles;

namespace sc = scrimmage;
namespace ma = msr::airlib;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::AirSimSensor, AirSimSensor_plugin)

namespace scrimmage {
namespace sensor {

AirSimSensor::AirSimSensor() : client_connected_(false),
    airsim_ip_("localhost"), airsim_port_(41451), airsim_timeout_ms_(60000) {

    enu_to_ned_yaw_.set_input_clock_direction(ang::Rotate::CCW);
    enu_to_ned_yaw_.set_input_zero_axis(ang::HeadingZero::Pos_X);
    enu_to_ned_yaw_.set_output_clock_direction(ang::Rotate::CW);
    enu_to_ned_yaw_.set_output_zero_axis(ang::HeadingZero::Pos_Y);
}

void AirSimSensor::init(std::map<std::string, std::string> &params) {
    airsim_ip_ = sc::get<std::string>("airsim_ip", params, "localhost");
    airsim_port_ = sc::get<int>("airsim_port", params, 41451);
    airsim_timeout_ms_ = sc::get<int>("airsim_timeout_ms", params, 60000);

    // Parse the camera config string.
    // The string is a list of camera configs of the form:
    // [CameraName ImageType CameraNumber Width Height]
    std::string camera_config = sc::get<std::string>("camera_config", params, "");
    std::vector<std::string> tokens_1;
    boost::split(tokens_1, camera_config, boost::is_any_of("[]"));
    for (std::string &t_1 : tokens_1) {
        if (t_1.length() > 0) {
            std::vector<std::string> tokens_2;
            boost::split(tokens_2, t_1, boost::is_any_of(" ,"));

            if (tokens_2.size() == 5) {
                try {
                    CameraConfig c;
                    c.name = tokens_2[0];
                    c.number = boost::lexical_cast<int>(tokens_2[2]);
                    c.width = std::stoi(tokens_2[3]);
                    c.height = std::stoi(tokens_2[4]);

                    if (tokens_2[1] == "Scene") {
                        c.img_type = ma::ImageCaptureBase::ImageType::Scene;
                    } else if (tokens_2[1] == "DepthPlanner") {
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthPlanner;
                    } else if (tokens_2[1] == "DepthPerspective") {
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthPerspective;
                    } else if (tokens_2[1] == "DepthVis") {
                        c.img_type = ma::ImageCaptureBase::ImageType::DepthVis;
                    } else if (tokens_2[1] == "DisparityNormalized") {
                        c.img_type = ma::ImageCaptureBase::ImageType::DisparityNormalized;
                    } else if (tokens_2[1] == "Segmentation") {
                        c.img_type = ma::ImageCaptureBase::ImageType::Segmentation;
                    } else if (tokens_2[1] == "SurfaceNormals") {
                        c.img_type = ma::ImageCaptureBase::ImageType::SurfaceNormals;
                    } else {
                        cout << "Error: Unknown image type: " << tokens_2[1] << endl;
                        c.img_type = ma::ImageCaptureBase::ImageType::Scene;
                    }
                    cout << "Adding Camera: " << c << endl;
                    cam_configs_.push_back(c);
                } catch (boost::bad_lexical_cast) {
                    // Parsing whitespace and possibily malformed XML.
                }
            }
        }
    }

    // Start the image request thread
    request_images_thread_ = std::thread(&AirSimSensor::request_images, this);

    pub_ = advertise("LocalNetwork", "AirSim");
    return;
}

void AirSimSensor::request_images() {
    std::shared_ptr<msr::airlib::MultirotorRpcLibClient> img_client =
        std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                     airsim_port_,
                                                     airsim_timeout_ms_);
    bool connected = false;
    while (!connected) {
        img_client->confirmConnection();

        // If we haven't been able to connect to AirSim, warn the user
        if (img_client->getConnectionState() !=
            ma::RpcLibClientBase::ConnectionState::Connected) {
            cout << "Warning: Image client not connected to AirSim." << endl;
        } else {
            connected = true;
        }
    }

    running_mutex_.lock();
    bool running = running_;
    running_mutex_.unlock();
    while (running) {
        auto msg = std::make_shared<sc::Message<std::vector<AirSimSensorType>>>();
        for (CameraConfig c : cam_configs_) {
            cv::Mat img(c.height, c.width, CV_8UC4);

            // get uncompressed rgba array bytes
            const int cam_forw = c.number; // AirSim v1.1.8
            // const std::string& cam_forw = c.name; // AirSim v1.2.0
            std::vector<ma::ImageCaptureBase::ImageRequest> request = {
                ma::ImageCaptureBase::ImageRequest(
                    cam_forw, c.img_type, false, false
                    )
            };

            const std::vector<ma::ImageCaptureBase::ImageResponse>& response
                = img_client->simGetImages(request);

            if (response.size() > 0) {
                auto& im_vec = response[0].image_data_uint8;

                // todo, no memcpy, just set image ptr to underlying data then do conversion
                // image = cv::Mat(144, 256, CV_8UC3, static_cast<uint8_t*>(im_vec.data()));
                memcpy(img.data, im_vec.data(), im_vec.size() * sizeof(uint8_t));

                AirSimSensorType a;
                a.camera_config = c;
                cv::cvtColor(img, a.img, CV_RGBA2BGR, 3);
                msg->data.push_back(a);
            }
        }
        img_msg_mutex_.lock();
        img_msg_ = msg;
        img_msg_mutex_.unlock();

        running_mutex_.lock();
        running = running_;
        running_mutex_.unlock();
    }
}

bool AirSimSensor::step() {
    ///////////////////////////////////////////////////////////////////////////
    /// Client Connection / Disconnection Handling
    ///////////////////////////////////////////////////////////////////////////

    // If we aren't connected to AirSim, try to connect.
    if (!client_connected_) {
        cout << "Connecting to AirSim: ip " << airsim_ip_ << ", port " << airsim_port_ << endl;
        sim_client_ = std::make_shared<ma::MultirotorRpcLibClient>(airsim_ip_,
                                                                   airsim_port_,
                                                                   airsim_timeout_ms_);
        sim_client_->confirmConnection();

        // If we haven't been able to connect to AirSim, warn the user and return.
        if (sim_client_->getConnectionState() !=
            ma::RpcLibClientBase::ConnectionState::Connected) {
            client_connected_ = false;
            cout << "Warning: not connected to AirSim." << endl;

            // return std::make_shared<sc::MessageBase>();
            return true;
        }
        client_connected_ = true;
        sim_client_->enableApiControl(true);
    }

    ///////////////////////////////////////////////////////////////////////////
    /// Set pose of vehicle in AirSim
    ///////////////////////////////////////////////////////////////////////////

    // Setup state information for AirSim
    sc::StatePtr &state = parent_->state_truth();
    // convert from ENU to NED frame
    ma::Vector3r pos(state->pos()(1), state->pos()(0), -state->pos()(2));

    enu_to_ned_yaw_.set_angle(ang::rad2deg(state->quat().yaw()));
    double airsim_yaw_rad = ang::deg2rad(enu_to_ned_yaw_.angle());

    // pitch, roll, yaw
    // note, the negative pitch and yaw are required because of the wsu coordinate frame
    ma::Quaternionr qd = ma::VectorMath::toQuaternion(-state->quat().pitch(),
                                                      state->quat().roll(),
                                                      airsim_yaw_rad);

    // Send state information to AirSim
    sim_client_->simSetPose(ma::Pose(pos, qd), true);

    // Get the camera images from the other thread
    sc::MessagePtr<std::vector<AirSimSensorType>> msg;
    img_msg_mutex_.lock();
    msg = img_msg_;
    img_msg_mutex_.unlock();

    pub_->publish(msg);

    return true;
}

void AirSimSensor::close(double t) {
    // Safely join with the image client thread
    running_mutex_.lock();
    running_ = false;
    running_mutex_.unlock();

    request_images_thread_.join();
}

} // namespace sensor
} // namespace scrimmage
