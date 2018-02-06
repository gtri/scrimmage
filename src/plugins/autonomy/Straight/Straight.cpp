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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/sensor/Sensor.h>

#if ENABLE_OPENCV == 1
#include <scrimmage/plugins/sensor/ContactBlobCamera/ContactBlobCameraType.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#if ENABLE_AIRSIM == 1
#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>
#endif

#include <scrimmage/plugins/interaction/Boundary/BoundaryInfo.h>
#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;

#include <scrimmage/plugins/autonomy/Straight/Straight.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
namespace fs = boost::filesystem;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::Straight, Straight_plugin)

namespace scrimmage {
namespace autonomy {

void Straight::init(std::map<std::string, std::string> &params) {

    std::cout << "straight:  " << std::endl;
    std::cout << "filename:  " << params["XML_FILENAME"] << std::endl;
    std::cout << "xml_dir:  " << params["XML_DIR"] << std::endl;
    speed_ = scrimmage::get("speed", params, 0.0);
    show_camera_images_ = scrimmage::get<bool>("show_camera_images", params, false);
    save_camera_images_ = scrimmage::get<bool>("save_camera_images", params, false);
    show_text_label_ = scrimmage::get<bool>("show_text_label", params, false);

    if (save_camera_images_) {
        /////////////////////////////////////////////////////////
        // Remove all img files from previous run
        if (fs::exists("./imgs")) {
            fs::recursive_directory_iterator it("./imgs");
            fs::recursive_directory_iterator endit;
            std::list<fs::path> paths_to_rm;
            while (it != endit) {
                fs::path path = it->path();
                if (fs::is_regular_file(*it) && path.extension() == ".png") {
                    paths_to_rm.push_back(path);
                }
                ++it;
            }
            for (fs::path p : paths_to_rm) {
                fs::remove(p);
            }
        } else {
            fs::create_directories("./imgs");
        }
        /////////////////////////////////////////////////////////
    }

    desired_state_->vel() = speed_*Eigen::Vector3d::UnitX();
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = state_->pos()(2)*Eigen::Vector3d::UnitZ();

    // Project goal in front...
    Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
    Eigen::Vector3d unit_vector = rel_pos.normalized();
    unit_vector = state_->quat().rotate(unit_vector);
    goal_ = state_->pos() + unit_vector * rel_pos.norm();

    frame_number_ = 0;

    if (show_text_label_) {
        // Draw a text label 30 meters in front of vehicle:
        Eigen::Vector3d in_front = state_->pos() + unit_vector * 30;

        sc::ShapePtr shape(new sp::Shape());
        shape->set_type(sp::Shape::Text);
        sc::set(shape->mutable_color(), 255, 255, 255);
        sc::set(shape->mutable_center(), in_front);
        shape->set_persistent(true);
        shape->set_text("Hello SCRIMMAGE!");
        shape->set_opacity(1.0);
        shapes_.push_back(shape);
    }

    enable_boundary_control_ = scrimmage::get<bool>("enable_boundary_control",
                                                    params, false);

    sub_boundary_info_ = create_subscriber("Boundary");

    alt_idx_ = vars_.declare("altitude", VariableIO::Direction::Out);
}

bool Straight::step_autonomy(double t, double dt) {

    // Read data from sensors...
    sc::State own_state = *state_;

    for (auto kv : parent_->sensors()) {
        if (kv.first == "NoisyState0") {
            auto msg = kv.second->sense<sc::State>(t);
            if (msg) {
                own_state = msg->data;
            }
        } else if (kv.first == "NoisyContacts0") {
            auto msg = kv.second->sense<std::list<sc::Contact>>(t);
        } else if (kv.first == "AirSimSensor0") {
#if (ENABLE_OPENCV == 1 && ENABLE_AIRSIM == 1)
            auto msg = kv.second->sense<std::vector<sc::sensor::AirSimSensorType>>(t);
            if (msg) {
                for (sc::sensor::AirSimSensorType a : msg->data) {
                    if (show_camera_images_) {
                        cv::imshow(a.camera_config.name.c_str(), a.img);
                        cv::waitKey(1);
                    }
                }
            }
#endif
        } else if (kv.first == "ContactBlobCamera0") {
#if ENABLE_OPENCV == 1
            if (show_camera_images_ || save_camera_images_) {
                auto msg = kv.second->sense<sc::sensor::ContactBlobCameraType>(t);
                if (msg) {
                    if (save_camera_images_) {
                        std::string img_name = "./imgs/camera_" +
                            std::to_string(frame_number_++) + ".png";
                        cv::imwrite(img_name, msg->data.frame);
                    }

                    if (show_camera_images_) {
                        cv::imshow("Camera Sensor", msg->data.frame);
                        cv::waitKey(1);
                    }
                }
            }
#endif
        }
    }

    for (auto &msg : sub_boundary_info_->msgs<sc::Message<sci::BoundaryInfo>>()) {
        if (msg->data.type == sci::BoundaryInfo::Type::Cuboid) {
            std::shared_ptr<sci::Cuboid> cuboid = std::make_shared<sci::Cuboid>();
            cuboid->set_points(msg->data.points);
            boundary_ = cuboid;
        }
    }

    if (boundary_ != nullptr && enable_boundary_control_) {
        if (!boundary_->contains(own_state.pos())) {
            // Project goal through center of boundary
            Eigen::Vector3d center = boundary_->center();
            center(2) = own_state.pos()(2); // maintain altitude
            Eigen::Vector3d diff = center - own_state.pos();
            goal_ = own_state.pos() + diff.normalized() * 1e6;
        }
    }

    Eigen::Vector3d diff = goal_ - own_state.pos();
    Eigen::Vector3d v = speed_ * diff.normalized();

    ///////////////////////////////////////////////////////////////////////////
    // Convert desired velocity to desired speed, heading, and pitch controls
    ///////////////////////////////////////////////////////////////////////////
    desired_state_->vel()(0) = v.norm();

    // Desired heading
    double heading = scrimmage::Angles::angle_2pi(atan2(v(1), v(0)));

    // Desired pitch
    Eigen::Vector2d xy(v(0), v(1));
    double pitch = scrimmage::Angles::angle_2pi(atan2(v(2), xy.norm()));

    // Set Desired Altitude to goal's z-position
    desired_state_->pos()(2) = goal_(2);

    vars_.output(alt_idx_, goal_(2));

    // Set the desired pitch and heading
    desired_state_->quat().set(0, pitch, heading);

    return true;
}
} // namespace autonomy
} // namespace scrimmage
