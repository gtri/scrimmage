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
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugins/sensor/ContactBlobCamera/ContactBlobCameraType.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>

#include <list>
#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::ContactBlobCamera, ContactBlobCamera_plugin)

namespace scrimmage {
namespace sensor {

void ContactBlobCamera::init(std::map<std::string, std::string> &params) {
    std::stringstream ss;
    ss << parent_->id().id();
    parameters_file_.open(parent_->mp()->log_dir() + "/blob_sensor_parameters_" + ss.str() + ".txt");

    gener_ = parent_->random()->gener();

    // override default parameters
    std::map<std::string, double> plugin_params;
    plugin_params["senderId"] = parent_->id().id();
    plugin_params["img_width"] = sc::get<int>("img_width", params, 800);
    plugin_params["img_height"] = sc::get<int>("img_height", params, 600);
    plugin_params["max_detect_range"] = sc::get<double>("max_detect_range", params, 1000);
    plugin_params["focal_length"] = sc::get<double>("focal_length", params, 1.0);
    plugin_params["fps"] = sc::get<double>("frames_per_second", params, 10);
    plugin_params["az_thresh"] = sc::Angles::deg2rad(sc::get<double>("azimuth_fov", params, 360));
    plugin_params["el_thresh"] = sc::Angles::deg2rad(sc::get<double>("elevation_fov", params, 360));
    plugin_params["fn_prob"] = sc::get<double>("false_negative_probability", params, 0.1);
    plugin_params["fp_prob"] = sc::get<double>("false_positive_probability", params, 0.1);
    plugin_params["max_false_positives"] = sc::get<int>("max_false_positives_per_frame", params, 10);
    plugin_params["std_dev_w"] = sc::get<int>("std_dev_width", params, 10);
    plugin_params["std_dev_h"] = sc::get<int>("std_dev_height", params, 10);

    set_plugin_params(plugin_params);

    srand(time(NULL));

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

    /**
     * Allow for updating plugin parameters in real-time.
     */
    auto params_cb = [&](sc::MessagePtr<std::map<std::string, double>> msg) {set_plugin_params(msg->data);};
    subscribe<std::map<std::string, double>>("LocalNetwork", "BlobPluginParams", params_cb);

    // Publish the resulting bounding boxes
    pub_ = advertise("LocalNetwork", "ContactBlobCamera");
}

bool ContactBlobCamera::step() {
    if ((time_->t() - last_frame_t_) < 1.0 / fps_) return true;

    sc::State sensor_frame;
    sc::SensorPtr &sensor = parent_->sensors()["ContactBlobCamera0"];
    sc::Quaternion sensor_quat =
      static_cast<sc::Quaternion>(parent_->state()->quat() *
          sensor->transform()->quat());
    Eigen::Vector3d sensor_pos = sensor->transform()->pos() +
      parent_->state()->pos();

    sensor_frame.set_quat(sensor_quat);
    sensor_frame.set_pos(sensor_pos);

    auto msg = std::make_shared<sc::Message<ContactBlobCameraType>>();

    msg->data.frame = cv::Mat::zeros(img_height_, img_width_, CV_8UC3);

    for (auto &kv : *(parent_->contacts())) {
        // Filter out (skip) own contact
        if (kv.second.id().id() == parent_->id().id()) continue;

        // Filter out contacts out of range, should use RTree, but rtree still
        // requires querying of ID from contact lists. (TODO)
        if ((kv.second.state()->pos() - sensor_frame.pos()).norm() >
            max_detect_range_) {
            continue;
        }

        // Don't "detect" current contact relative to false negative probability
        double r = static_cast<double>(rand()) / RAND_MAX;
        if (r <= fn_prob_) continue;

        // Transform contact into "camera" coordinate system
        Eigen::Vector3d rel_pos = sensor_frame.rel_pos_local_frame(kv.second.state()->pos());


        if (!in_field_of_view(rel_pos)) continue;

        // Convert 3D relative position to 2d image plane position
        Eigen::Vector2d raster_center = project_rel_3d_to_2d(rel_pos);

        double object_radius = kv.second.radius();

        // Get angle between (vector between camera and object center) and
        // (vector between object center and line tangent to object circle)
        double beta = acos(object_radius / rel_pos.norm());

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
        Eigen::Vector3d p1(rel_pos(0) + v_r_1(0),
                           rel_pos(1) + v_r_1(1),
                           rel_pos(2));

        Eigen::Vector3d p2(rel_pos(0) + v_r_2(0),
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

        // Add bounding box to frame around object
        cv::Rect rect(std::floor(raster_center(0))-std::floor(object_img_radius),
                      std::floor(raster_center(1))-std::floor(object_img_radius),
                      std::floor(object_img_radius*2) + 1,
                      std::floor(object_img_radius*2) + 1);

        std::vector<cv::Rect> bounding_boxes;
        bounding_boxes.push_back(rect);

        if (object_img_radius > 0) {
            draw_object_with_bounding_box(msg->data.frame, rect, raster_center, object_img_radius);
        }

        // Add false positives
        for (int i = 0; i <= max_false_positives_; i++) {
            double r = static_cast<double>(rand()) / RAND_MAX;
            if (r > fp_prob_) continue;

            // TODO: use the input standard deviation to add tracks in a smarter manner
            // Generate random position in frame
            int h = rand() % img_height_;
            int w = rand() % img_width_;

            rect = cv::Rect(h - object_img_radius, w - object_img_radius,
                            std::floor(object_img_radius*2) + 1,
                            std::floor(object_img_radius*2) + 1);

            if (object_img_radius > 0) {
                draw_object_with_bounding_box(msg->data.frame, rect, Eigen::Vector2d(h, w), object_img_radius);
            }

            bounding_boxes.push_back(rect);
        }

        // Collect all bounding boxes for current detected object
        msg->data.bounding_boxes[kv.second.id().id()] = bounding_boxes;
    }

    frame_ = msg->data.frame;
    last_frame_t_ = time_->t();

    pub_->publish(msg);
    return true;
}

Eigen::Vector2d ContactBlobCamera::project_rel_3d_to_2d(Eigen::Vector3d rel_pos) {
    // 3D to 2D transforms are described here:
    // http://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points

    // Convert relative position of contact into computer vision coordinates
    // for relative coordinates:
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

bool ContactBlobCamera::in_field_of_view(Eigen::Vector3d rel_pos) {
    double az = atan2(rel_pos(1), rel_pos(0));
    double norm_xy = sqrt(pow(rel_pos(0), 2) + pow(rel_pos(1), 2));
    double el = atan2(rel_pos(2), norm_xy);
    if (std::abs(az) > (az_thresh_ / 2) ||
        std::abs(el) > (el_thresh_ / 2)) {
        return false;
    }
    return true;
}

void ContactBlobCamera::draw_object_with_bounding_box(cv::Mat frame, cv::Rect rect,
                                                      Eigen::Vector2d center,
                                                      double radius) {
    Eigen::Vector2i c(std::floor(center(0)),
                      std::floor(center(1)));

    cv::circle(frame, cv::Point(c(0), c(1)),
               std::floor(radius), cv::Scalar(255, 255, 255),
               -1, 8, 0);

    cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 1, 8, 0);
}

void ContactBlobCamera::set_plugin_params(std::map<std::string, double> params) {
    if (params["senderId"] != parent_->id().id()) { return; }

    img_width_ = static_cast<int>(params["img_width"]);
    img_height_ = static_cast<int>(params["img_height"]);

    max_detect_range_ = params["max_detect_range"];
    focal_length_ = params["focal_length"];
    fps_ = params["fps"];
    az_thresh_ = params["az_thresh"];
    el_thresh_ = params["el_thresh"];

    fn_prob_ = params["fn_prob"];
    fp_prob_ = params["fp_prob"];
    max_false_positives_ = params["max_false_positives"];
    std_dev_w_ = params["std_dev_w"];
    std_dev_h_ = params["std_dev_h"];

    if (std_dev_w_ > img_width_) { std_dev_w_ = 10; } // TODO: use standard deviation values to inform the false positives
    if (std_dev_h_ > img_height_) { std_dev_h_ = 10; }

    canvas_width_ = 2 * focal_length_ * tan(az_thresh_ / 2.0);
    canvas_height_ = 2 * focal_length_ * tan(el_thresh_ / 2.0);

    // keep track of values of the parameters
    char buf[100];
    snprintf(buf, sizeof(buf), "\n===================================\n");
    parameters_file_ << buf << std::endl;
    for (const auto &kv : params) {
        snprintf(buf, sizeof(buf), "%s: %f", kv.first.c_str(), kv.second);
        parameters_file_ << buf << std::endl;
    }
}

} // namespace sensor
} // namespace scrimmage
