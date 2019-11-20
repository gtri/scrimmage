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

#include <scrimmage/plugins/autonomy/ROSAirSim/ROSAirSim.h>

#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::ROSAirSim,
                ROSAirSim_plugin)

namespace scrimmage {
namespace autonomy {

void ROSAirSim::init(std::map<std::string, std::string> &params) {
    show_camera_images_ = scrimmage::get<bool>("show_camera_images", params, true);

    // initialize ros
    if (!ros::isInitialized()) {
        int argc = 0;
        // scrimmage handles it's own SIGINT/SIGTERM shutdown in main.cpp
        ros::init(argc, NULL, "scrimmage", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_shared<ros::NodeHandle>();

    // Setup robot namespace
    ros_namespace_ = sc::get<std::string>("ros_namespace_prefix", params, "robot");
    ros_namespace_ += std::to_string(parent_->id().id());

    // image_transport::ImageTransport it(nh_);
    it_ = std::make_shared<image_transport::ImageTransport>(*nh_);

    // airsim image callback
    auto airsim_cb = [&](auto &msg) {
        for (sc::sensor::AirSimSensorType a : msg->data) {
            // look for existing publisher on the named topic
            std::string topic_name = ros_namespace_ + "/" + a.camera_config.name;
            bool published = false;
            for (auto pub : img_publishers_) {
                if (pub.getTopic() == topic_name) {
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", a.img).toImageMsg();
                    pub.publish(msg);
                    published = true;
                }
            }
            // if it doesn't exist, create it.
            // TODO: you lose a frame in this construction
            if (false == published) {
                img_publishers_.push_back(it_->advertise(topic_name, 1));
            }

            // draw image
            if (show_camera_images_) {
                cv::imshow(a.camera_config.name.c_str(), a.img);
                cv::waitKey(1);
            }
        }
    };
    subscribe<std::vector<sensor::AirSimSensorType>>("LocalNetwork", "AirSim", airsim_cb);
}

bool ROSAirSim::step_autonomy(double t, double dt) {

    ros::spinOnce(); // check for new ROS messages

    return true;
}
} // namespace autonomy
} // namespace scrimmage
