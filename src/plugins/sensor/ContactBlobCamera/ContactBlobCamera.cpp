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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/sensor/ContactBlobCamera/ContactBlobCamera.h>
#include <scrimmage/common/ID.h>
#include <vector>
#include <list>
#include <utility>
#include <scrimmage/common/RTree.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <scrimmage/plugins/sensor/ContactBlobCamera/ContactBlobCameraType.h>

REGISTER_PLUGIN(scrimmage::Sensor, ContactBlobCamera, ContactBlobCamera_plugin)

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

using std::cout;
using std::endl;

void ContactBlobCamera::init(std::map<std::string,std::string> &params)
{
    gener_ = parent_->random()->gener();

    img_width_ = sc::get<int>("img_width", params, 800);
    img_height_ = sc::get<int>("img_height", params, 600);

    max_detect_range_ = sc::get<double>("max_detect_range", params, 1000);
    focal_length_ = sc::get<double>("focal_length", params, 1.0);
    fps_ = sc::get<double>("frames_per_second", params, 10);
    az_thresh_ = sc::Angles::deg2rad(sc::get<double>("azimuth_fov", params, 360));
    el_thresh_ = sc::Angles::deg2rad(sc::get<double>("elevation_fov", params, 360));

    canvas_width_ = 2 * focal_length_ * tan(az_thresh_ / 2.0);
    canvas_height_ = 2 * focal_length_ * tan(el_thresh_ / 2.0);

    for (int i = 0; i < 3; i++) {
        std::string tag_name = "pos_noise_" + std::to_string(i);
        std::vector<double> vec;
        bool status = sc::get_vec(tag_name, params, " ", vec, 2);
        if (status) {
            pos_noise_.push_back(parent_->random()->make_rng_normal(vec[0], vec[1]));
        } else {
            pos_noise_.push_back(parent_->random()->make_rng_normal(0, 1));
        }
    }

    for (int i = 0; i < 3; i++) {
        std::string tag_name = "orient_noise_" + std::to_string(i);
        std::vector<double> vec;
        bool status = sc::get_vec(tag_name, params, " ", vec, 2);
        if (status) {
            orient_noise_.push_back(parent_->random()->make_rng_normal(vec[0], vec[1]));
        } else {
            orient_noise_.push_back(parent_->random()->make_rng_normal(0, 1));
        }
    }

    return;
}

boost::optional<sc::MessageBasePtr> ContactBlobCamera::sensor_msg(double t)
{
    auto msg = std::make_shared<sc::Message<ContactBlobCameraType>>();

    if ((t - last_frame_t_) < 1.0 / fps_) {
        return boost::optional<sc::MessageBasePtr>{};
    }

    msg->data.frame = cv::Mat::zeros(img_height_, img_width_, CV_8UC3);

    for (auto &kv : *(parent_->contacts())) {
        // Filter out (skip) own contact
        if (kv.second.id().id() == parent_->id().id()) continue;

        // Filter out contacts out of range, should use RTree, but rtree still
        // requires querying of ID from contact lists. (TODO)
        if ((kv.second.state()->pos() - parent_->state()->pos()).norm() >
            max_detect_range_) {
            continue;
        }

        // Transform contact into "camera" coordinate system
        Eigen::Vector3d rel_pos = parent_->state()->rel_pos_local_frame(kv.second.state()->pos());

        if (!in_field_of_view(rel_pos)) continue;

        // Convert 3D relative position to 2d image plane position
        Eigen::Vector2d raster_center = project_rel_3d_to_2d(rel_pos);

        double object_radius = kv.second.radius();

        // Get angle between (vector between camera and object center) and
        // (vector between object center and line tangent to object circle)
        double beta = acos(object_radius/ rel_pos.norm());

        // Get vector pointing from contact to camera
        Eigen::Vector2d v_r(-rel_pos(0), -rel_pos(1));

        // Normalize to unit vector and Make length equal to object radius
        v_r = v_r.normalized() * object_radius;

        // Rotate one direction and add circle's center point
        Eigen::Vector2d v_r_1 = v_r, v_r_2 = v_r;
        Eigen::Rotation2D<double> rot1(beta);
        Eigen::Rotation2D<double> rot2(-beta);
        v_r_1 = rot1.toRotationMatrix() * v_r_1;
        v_r_2 = rot2.toRotationMatrix() * v_r_2;

        // Get 3D relative position of rotated vectors
        Eigen::Vector3d p1 (rel_pos(0) + v_r_1(0),
                            rel_pos(1) + v_r_1(1),
                            rel_pos(2));

        Eigen::Vector3d p2 (rel_pos(0) + v_r_2(0),
                            rel_pos(1) + v_r_2(1),
                            rel_pos(2));

        // Get 2D position of object boundaries
        Eigen::Vector2d r1 = project_rel_3d_to_2d(p1);
        Eigen::Vector2d r2 = project_rel_3d_to_2d(p2);

        // Calculate image radius using distance between object boundaries
        double object_img_radius = (r1 - r2).norm() / 2.0;

        // Add noise to position in 2D image plane
        raster_center(0) += (*pos_noise_[0])(*gener_);
        raster_center(1) += (*pos_noise_[1])(*gener_);

        // Draw bounding box afloor object
        cv::Rect rect(std::floor(raster_center(0))-std::floor(object_img_radius),
                      std::floor(raster_center(1))-std::floor(object_img_radius),
                      std::floor(object_img_radius*2)+1,
                      std::floor(object_img_radius*2)+1);
        
        msg->data.bounding_boxes[kv.second.id().id()] = rect;

        // Draw object (without bounding box first)
        if (object_img_radius > 0) {
            // Floor raster_center position:
            Eigen::Vector2i center(std::floor(raster_center(0)),
                                   std::floor(raster_center(1)));

            cv::circle(msg->data.frame, cv::Point(center(0), center(1)),
                       std::floor(object_img_radius), cv::Scalar(255,255,255),
                       -1, 8, 0);                        
        }
    }

    // Draw bounding boxes on top of objects
    for (auto &kv : msg->data.bounding_boxes) {
        // Draw object as circle
        cv::Rect rect = kv.second;
        cv::rectangle(msg->data.frame, rect, cv::Scalar(0,0,255), 1, 8, 0);
    }

    frame_ = msg->data.frame;
    last_frame_t_ = t;

    return boost::optional<sc::MessageBasePtr>(msg);
}

Eigen::Vector2d ContactBlobCamera::project_rel_3d_to_2d(Eigen::Vector3d rel_pos)
{
    // 3D to 2D transforms are described here:
    //http://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points

    //Convert relative position of contact into computer vision coordinates
    //for relative coordinates:
    Eigen::Vector3d pos(rel_pos(1), rel_pos(2), rel_pos(0));

    // Transform to screen / canvas space
    Eigen::Vector2d s(focal_length_ * -pos(0) / pos(2),
                      focal_length_ * pos(1) / pos(2));

    // Transform to Normalized Device Coordinate (NDC) space
    Eigen::Vector2d ndc((s(0) + canvas_width_/2.0) / canvas_width_,
                        (s(1) + canvas_height_/2.0) / canvas_height_);

    // Tranform to raster space
    Eigen::Vector2d r(ndc(0) * img_width_,
                      (1-ndc(1)) * img_height_);
    return r;
}

bool ContactBlobCamera::in_field_of_view(Eigen::Vector3d rel_pos)
{
    double az = atan2(rel_pos(1), rel_pos(0));
    double norm_xy = sqrt(pow(rel_pos(0), 2) + pow(rel_pos(1), 2));
    double el = atan2(rel_pos(2), norm_xy);
    if (std::abs(az) > (az_thresh_ / 2) ||
        std::abs(el) > (el_thresh_ / 2)) {
        return false;
    }
    return true;
}
