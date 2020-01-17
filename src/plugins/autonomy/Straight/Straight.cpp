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
#include <scrimmage/common/Shape.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/sensor/Sensor.h>

#if ENABLE_OPENCV == 1
#include <scrimmage/plugins/sensor/ContactBlobCamera/ContactBlobCameraType.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#if ENABLE_AIRSIM == 1
#include <scrimmage/plugins/sensor/AirSimSensor/AirSimSensor.h>
#endif

#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;

#include <scrimmage/plugins/autonomy/Straight/Straight.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
namespace fs = boost::filesystem;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::Straight,
                Straight_plugin)

namespace scrimmage {
namespace autonomy {

void Straight::init(std::map<std::string, std::string> &params) {
    speed_ = scrimmage::get("speed", params, 0.0);
    show_camera_images_ = scrimmage::get<bool>("show_camera_images", params, false);
    save_camera_images_ = scrimmage::get<bool>("save_camera_images", params, false);
    show_text_label_ = scrimmage::get<bool>("show_text_label", params, false);

    // Project goal in front...
    Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
    Eigen::Vector3d unit_vector = rel_pos.normalized();
    unit_vector = state_->quat().rotate(unit_vector);
    goal_ = state_->pos() + unit_vector * rel_pos.norm();

    // Set the desired_z to our initial position.
    // desired_z_ = state_->pos()(2);

    // Register the desired_z parameter with the parameter server
    auto param_cb = [&](const double &desired_z) {
        std::cout << "desired_z param changed at: " << time_->t()
        << ", with value: " << desired_z << endl;
    };
    register_param<double>("desired_z", goal_(2), param_cb);

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

    frame_number_ = 0;

    if (show_text_label_) {
        // Draw a text label (white text) 30 meters in front of vehicle:
        Eigen::Vector3d in_front = state_->pos() + unit_vector * 30;
        text_shape_ = sc::shape::make_text("Hello, SCRIMMAGE!", in_front,
                                           Eigen::Vector3d(255, 255, 255));

        // Draw the shape in the 3D viewer
        draw_shape(text_shape_);
    }

    enable_boundary_control_ = get<bool>("enable_boundary_control", params, false);

    auto bd_cb = [&](auto &msg) {boundary_ = sci::Boundary::make_boundary(msg->data);};
    subscribe<sp::Shape>("GlobalNetwork", "Boundary", bd_cb);

    auto state_cb = [&](auto &msg) {
        noisy_state_set_ = true;
        noisy_state_ = msg->data;
    };
    subscribe<StateWithCovariance>("LocalNetwork", "StateWithCovariance", state_cb);

    auto cnt_cb = [&](scrimmage::MessagePtr<ContactMap> &msg) {
        noisy_contacts_ = msg->data; // Save map of noisy contacts
    };
    subscribe<ContactMap>("LocalNetwork", "ContactsWithCovariances", cnt_cb);

#if (ENABLE_OPENCV == 1 && ENABLE_AIRSIM == 1)
    auto airsim_cb = [&](auto &msg) {
        for (sc::sensor::AirSimImageType a : msg->data) {
            if (show_camera_images_) {
                // Get Camera Name
                std::string window_name = a.camera_config.cam_name + "_" + a.camera_config.img_type_name;
                // for depth images CV imshow expects grayscale image values to be between 0 and 1.
                if (a.camera_config.img_type_name == "DepthPerspective" || a.camera_config.img_type_name == "DepthPlanner") {
                    // Worked before building with ROS
                    cv::Mat tempImage;
                    a.img.convertTo(tempImage, CV_32FC1, 1.f/255);
                    // cv::normalize(a.img, tempImage, 0, 1, cv::NORM_MINMAX);
                    // cout << tempImage << endl;
                    cv::imshow(window_name, tempImage);
                } else {
                    // other image types are int 0-255.
                    cv::imshow(window_name, a.img);
                }
                cv::waitKey(1);
            }
        }
    };
    subscribe<std::vector<sensor::AirSimImageType>>("LocalNetwork", "AirSimImages", airsim_cb);
#endif

#if ENABLE_OPENCV == 1
    auto blob_cb = [&](auto &msg) {
        if (save_camera_images_) {
            std::string img_name = "./imgs/camera_" +
                std::to_string(frame_number_++) + ".png";
            cv::imwrite(img_name, msg->data.frame);
        }

        if (show_camera_images_) {
            cv::imshow("Camera Sensor", msg->data.frame);
            cv::waitKey(1);
        }
    };
    subscribe<sc::sensor::ContactBlobCameraType>("LocalNetwork", "ContactBlobCamera", blob_cb);
#endif

    gen_ents_ = sc::get("generate_entities", params, gen_ents_);
    if (gen_ents_) {
        pub_gen_ents_ = advertise("GlobalNetwork", "GenerateEntity");
    }

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
}

bool Straight::step_autonomy(double t, double dt) {
    if (gen_ents_) {
        if (time_->t() > (prev_gen_time_ + 2.0)) {
            prev_gen_time_ = time_->t();

            // Create a state for the new entity
            State s(*state_);

            // Rotate the heading of the new entity by 90 degrees and offset
            // the initial position to the left of our current position to
            // avoid collisions.
            Eigen::AngleAxisd rot_90_z(M_PI/2.0, Eigen::Vector3d::UnitZ());
            s.pos() = s.pos() + s.quat() * rot_90_z * (Eigen::Vector3d::UnitX() * 10);
            s.quat() = rot_90_z * s.quat();

            // Create the GenerateEntity message
            auto msg = std::make_shared<Message<scrimmage_msgs::GenerateEntity>>();
            sc::set(msg->data.mutable_state(), s); // Copy the new state

            // The entity_tag must match the "tag" XML attribute of the entity
            // to be generated in the mission file.
            msg->data.set_entity_tag("gen_straight");

            // Modify the entity's color
            auto kv_color = msg->data.add_entity_param();
            kv_color->set_key("color");
            kv_color->set_value("255, 255, 0");

            // Modify the entity's visual model
            auto kv_visual = msg->data.add_entity_param();
            kv_visual->set_key("visual_model");
            kv_visual->set_value("sphere");

            pub_gen_ents_->publish(msg); // Publish the GenerateEntity message
        }
    }

    if (show_text_label_) {
        // An example of changing a shape's property
        if (t > 1.0 && t < (1.0 + dt)) {
            text_shape_->set_opacity(0.5);
            text_shape_->mutable_text()->set_text("Goodbye, SCRIMMAGE!");
            draw_shape(text_shape_);
        }

        // An example of removing a shape
        if (t > 5.0 && t < (5.0 + dt)) {
            text_shape_->set_persistent(false);
            draw_shape(text_shape_);
        }
    }

    // Read data from sensors...
    if (!noisy_state_set_) {
        noisy_state_ = *state_;
    }

    if (boundary_ != nullptr && enable_boundary_control_) {
        if (!boundary_->contains(noisy_state_.pos())) {
            // Project goal through center of boundary
            Eigen::Vector3d center = boundary_->center();
            center(2) = noisy_state_.pos()(2); // maintain altitude
            Eigen::Vector3d diff = center - noisy_state_.pos();
            goal_ = noisy_state_.pos() + diff.normalized() * 1e6;
        }
    }

    Eigen::Vector3d diff = goal_ - noisy_state_.pos();
    Eigen::Vector3d v = speed_ * diff.normalized();

    ///////////////////////////////////////////////////////////////////////////
    // Convert desired velocity to desired speed, heading, and pitch controls
    ///////////////////////////////////////////////////////////////////////////
    double heading = Angles::angle_2pi(atan2(v(1), v(0)));
    vars_.output(desired_alt_idx_, goal_(2));
    vars_.output(desired_speed_idx_, v.norm());
    vars_.output(desired_heading_idx_, heading);

    noisy_state_set_ = false;
    return true;
}
} // namespace autonomy
} // namespace scrimmage
