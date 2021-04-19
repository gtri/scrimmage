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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @date 10 April 2021
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

#include <scrimmage/plugins/autonomy/Square/Square.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
namespace fs = boost::filesystem;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::Square,
                Square_plugin)

namespace scrimmage {
namespace autonomy {

std::deque<State> Square::get_init_maneuver_positions() {

        std::deque<State> man_positions;

        // get the starting position
        sc::StatePtr &state = parent_->state_truth();
        State init_state(*state);
        State current_state(*state);
        man_positions.push_back(current_state);
//        double delta = 0.05;
        double delta = 1.0;

        // Rise up to 5 meters above the starting position
        for (double i=0.0; i<5.0; i+=delta) {
            current_state.pos()(2) = current_state.pos()(2) + delta;
            man_positions.push_back(current_state);
        }

        // repeat the below right-left movements 1 times
        for (int repeat = 0; repeat<1; repeat++){
            // move 2 meters to the right, back to the center
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(1) = current_state.pos()(1) + delta;;
                man_positions.push_back(current_state);
            }
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(1) = current_state.pos()(1) - delta;
                man_positions.push_back(current_state);
            }
            // move 2 meters to the left, back to the center
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(1) = current_state.pos()(1) - delta;
                man_positions.push_back(current_state);
            }
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(1) = current_state.pos()(1) + delta;
                man_positions.push_back(current_state);
            }
        }

        // repeat the below up-down movements 3 times
        for (int repeat = 0; repeat<1; repeat++){
            // move 1 meter up, back to the center
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(2) = current_state.pos()(2) + delta;
                man_positions.push_back(current_state);
            }
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(2) = current_state.pos()(2) - delta;
                man_positions.push_back(current_state);
            }
            // move 1 meter down, back to the center
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(2) = current_state.pos()(2) - delta;
                man_positions.push_back(current_state);
            }
            for (double i=0.0; i<5.0; i+=delta) {
                current_state.pos()(2) = current_state.pos()(2) + delta;
                man_positions.push_back(current_state);
            }
        }

//        // move in a 4mx2m rectangle parallel to the ground 3 times
//        for (int repeat = 0; repeat<1; repeat++){
//            // move 1 meter in the +y direction
//            for (double i=0.0; i<5.0; i+=delta) {
//                current_state.pos()(1) = current_state.pos()(1) + delta;
//                man_positions.push_back(current_state);
//            }
//            // move 1 meter in the +x direction
//            for (double i=0.0; i<5.0; i+=delta) {
//                current_state.pos()(0) = current_state.pos()(0) + delta;
//                man_positions.push_back(current_state);
//            }
//            // move 2 meters in the -y direction
//            for (double i=0.0; i<10.0; i+=delta) {
//                current_state.pos()(1) = current_state.pos()(1) - delta;
//                man_positions.push_back(current_state);
//            }
//            // move 1 meters in the -x direction
//            for (double i=0.0; i<5.0; i+=delta) {
//                current_state.pos()(0) = current_state.pos()(0) - delta;
//                man_positions.push_back(current_state);
//            }
//            // move 1 meter in the +y direction
//            for (double i=0.0; i<5.0; i+=delta) {
//                current_state.pos()(1) = current_state.pos()(1) + delta;
//                man_positions.push_back(current_state);
//            }
//        }

        // place back at beginning for Straight Plugin
        man_positions.push_back(init_state);

        return man_positions;
}

void Square::init(std::map<std::string, std::string> &params) {
    speed_ = scrimmage::get("speed", params, 0.0);
    show_camera_images_ = scrimmage::get<bool>("show_camera_images", params, false);
    save_camera_images_ = scrimmage::get<bool>("save_camera_images", params, false);
    sq_side_length_m_ = scrimmage::get<double>("sq_side_length_m", params, 100);

    // record the initial state:
    init_man_goal_pos_ = state_->pos();
    init_man_goal_quat_ = state_->quat();

    // Project goal in front...
//    Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
//    Eigen::Vector3d unit_vector = rel_pos.normalized();
//    unit_vector = state_->quat().rotate(unit_vector);
//    goal_ = state_->pos() + unit_vector * rel_pos.norm();
//    goal_(2) = state_->pos()(2);

    cout << "speed: " << speed_ << endl;

    // initialize the corner locations of the square
    // positve x direction
    goal1_ = state_->pos();
    goal1_(0) = goal1_(0) + sq_side_length_m_;
    cout << "goal1: " << goal1_ << endl;
    // positve y direction
    goal2_ = goal1_;
    goal2_(1) = goal2_(1) + sq_side_length_m_;
    cout << "goal2: " << goal2_ << endl;
    // negative x direction
    goal3_ = goal2_;
    goal3_(0) = goal3_(0) - sq_side_length_m_;
    cout << "goal3: " << goal3_ << endl;
    // negative y direction
    goal4_ = goal3_;
    goal4_(1) = goal4_(1) - sq_side_length_m_;
    cout << "goal4: " << goal4_ << endl;

    goal_ = goal1_;

    // Set the desired_z to our initial position.
    // desired_z_ = state_->pos()(2);
    init_goal_ = state_->pos();
    cout << "original init_goal_: " << init_goal_ << endl;

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

    // If using an initialization maneuver, then compile the position vector
    use_init_maneuver_ = sc::get<bool>("use_init_maneuver", params, "false");
    if (use_init_maneuver_) {
        man_positions_ = get_init_maneuver_positions();
        cout << "[Square] Start VO Initialization Maneuver." << endl;
    }

//    enable_boundary_control_ = get<bool>("enable_boundary_control", params, false);
//
//    auto bd_cb = [&](auto &msg) {boundary_ = sci::Boundary::make_boundary(msg->data);};
//    subscribe<sp::Shape>("GlobalNetwork", "Boundary", bd_cb);

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
                std::string window_name = a.vehicle_name + "_" + a.camera_config.cam_name + "_" + a.camera_config.img_type_name;
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
                    if (a.img.channels() == 4) {
                        cout << "image channels: " << a.img.channels() << endl;
                        cout << "Warning: Old AirSim Linux Asset Environments have 4 channels. Color images will not display correctly." << endl;
                        cout << "Warning: Use Asset Environment versions Linux-v1.3.1+." << endl;
                        cv::Mat tempImage;
                        cv::cvtColor(a.img , tempImage, CV_RGBA2RGB);
                        cv::imshow(window_name, tempImage);
                    } else {
                        cv::imshow(window_name, a.img);
                    }
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

//    gen_ents_ = sc::get("generate_entities", params, gen_ents_);
//    if (gen_ents_) {
//        pub_gen_ents_ = advertise("GlobalNetwork", "GenerateEntity");
//    }

    desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
    desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
}

bool Square::step_autonomy(double t, double dt) {

    // Read data from sensors...
    if (!noisy_state_set_) {
        noisy_state_ = *state_;
    }

//    if (boundary_ != nullptr && enable_boundary_control_) {
//        if (!boundary_->contains(noisy_state_.pos())) {
//            // Project goal through center of boundary
//            Eigen::Vector3d center = boundary_->center();
//            center(2) = noisy_state_.pos()(2); // maintain altitude
//            Eigen::Vector3d diff = center - noisy_state_.pos();
//            goal_ = noisy_state_.pos() + diff.normalized() * 1e6;
//        }
//    }

    Eigen::Vector3d diff;
    Eigen::Vector3d v;
    double altitude = goal_(2);
    // double heading = init_man_goal_quat_.yaw();


    if (man_positions_.size() == 0 && use_init_maneuver_) {
        init_maneuver_finished_ = true;
        goal_ = init_goal_;
        cout << "init maneuver finished" << endl;
    }

    if (!init_maneuver_finished_ && use_init_maneuver_) {

        // Pop a state off the front of man_positions_ vector and delete the element
        float distance = sqrt(pow((init_man_goal_pos_(0) - noisy_state_.pos()(0)), 2) + pow((init_man_goal_pos_(1) - noisy_state_.pos()(1)), 2));
        if (distance < 0.05) {
            State next_state = man_positions_.front();
            man_positions_.pop_front();
            // cout << "remaining man_pos: " << man_positions_.size() << endl;
            init_man_goal_pos_ = next_state.pos();
            init_man_goal_quat_ = next_state.quat();
        }

        cout << "remaining man_pos: " << man_positions_.size() << endl;

        cout << "next_state: " << init_man_goal_pos_ << endl;
        // create desired goal and velocity
        diff = init_man_goal_pos_ - noisy_state_.pos();
        v = 5.0 * diff.normalized();
        // heading = init_man_goal_quat_.yaw();
        altitude = init_man_goal_pos_(2);

    } else {

        // if init maneuver was used, start the drones at the starting point
        if(square_side_ == 0) {
            if(use_init_maneuver_) {
                float distance = sqrt(pow((goal_(0) - noisy_state_.pos()(0)), 2) + pow((goal_(1) - noisy_state_.pos()(1)), 2));
                if (distance < 5) {
                    square_side_ += 1;
                    goal_ = goal1_;
                }
            }
            else{
                square_side_ += 1;
            }
        }

        // check if we have completed this side
        float distance = sqrt(pow((init_goal_(0) - state_->pos()(0)), 2) + pow((init_goal_(1) - state_->pos()(1)), 2));
//        cout << "distance: " << distance << endl;
//        cout << "sq_side_length_m_: " << sq_side_length_m_ << endl;
//        cout << "square_side_: " << square_side_ << endl;
//        cout << "position: " << state_->pos()<< endl;
        if (distance >= sq_side_length_m_ && square_side_ > 0){
            // update which side of the square we are on.
            square_side_ += 1;
            init_goal_ = goal_;

            // rotate the goal
            int current_side = square_side_ % 4;
            switch(current_side) {
                case 1:
                  {
                      // cout << '1' << endl; // prints "1"
                      // Project goal in front...
                      // Move in positive X direction
                        goal_ = goal1_;
                      break;
                  }
                case 2 :
                  {
                      // cout << '2' << endl;
                      // Project goal in front...
                      // Move in positive Y direction
                      goal_ = goal2_;
                      break;
                  }
                case 3:
                  {
                      // cout << '3' << endl;
                      // Project goal in front...
                      // Move in negative X direction
                      goal_ = goal3_;
                      break;
                  }
                case 0:
                  {
                      // cout << '4' << endl;
                      // Project goal in front...
                      // Move in negative Y direction
                      goal_ = goal4_;
                      break;
                  }
            }
            cout << "goal: " << goal_ << endl;
        }

     // create desired goal and velocity
    diff = goal_ - noisy_state_.pos();
    v = speed_ * diff.normalized();
    altitude = goal_(2);
    // heading = Angles::angle_2pi(atan2(v(1), v(0)));
    }

    ///////////////////////////////////////////////////////////////////////////
    // Convert desired velocity to desired speed, heading, and pitch controls
    ///////////////////////////////////////////////////////////////////////////
    double heading = Angles::angle_2pi(atan2(v(1), v(0)));
    vars_.output(desired_alt_idx_, altitude);
    vars_.output(desired_speed_idx_, v.norm());
    vars_.output(desired_heading_idx_, heading);

    noisy_state_set_ = false;
    return true;
}
} // namespace autonomy
} // namespace scrimmage
