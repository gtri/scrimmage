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

#include <GeographicLib/LocalCartesian.hpp>

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

    window_name_ = sc::get<std::string>("window_name", params, window_name_);
    ignore_real_entities_ = sc::get<bool>("ignore_real_entities", params, ignore_real_entities_);
    show_image_ = sc::get<bool>("show_image", params, show_image_);
    show_frustum_ = sc::get<bool>("show_frustum", params, show_frustum_);
    log_detections_ = sc::get<bool>("log_detections", params, log_detections_);
    show_sim_contacts_ = sc::get<bool>("show_sim_contacts", params, true);

    // Parse the simulated detections
    std::string sim_det_str = sc::get<std::string>("simulated_detections", params, "");
    std::vector<std::vector<std::string>> vecs;
    if (get_vec_of_vecs(sim_det_str, vecs, ", ")) {
        int i = 0;
        for (auto &vec : vecs) {
            // First, assume the description is in XYZ
            int id = std::stoi(vec[1]);
            double radius = std::stod(vec[2]);
            double x = std::stod(vec[3]);
            double y = std::stod(vec[4]);
            double z = std::stod(vec[5]);

            if (vec[0] == "GPS") {
                // If the description is in GPS, convert to local cartesian
                parent_->projection()->Forward(x, y, z, x, y, z);
            }

            sc::Quaternion quat(std::stod(vec[6]), std::stod(vec[7]),
                                std::stod(vec[8]));

            sc::StatePtr cnt_state = std::make_shared<sc::State>(
                Eigen::Vector3d(x, y, z), Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(0, 0, 0), quat);

            sc::Contact cnt(sc::ID(id, 0, 0), cnt_state);
            cnt.set_type(sc::Contact::Type::MESH);
            cnt.set_radius(radius);
            cnt.set_active(true);

            // Allow multiple simulated contacts to have the same ID
            sim_contacts_[i++] = cnt;
        }
    }

    // override default parameters
    plugin_params_["senderId"] = parent_->id().id();
    plugin_params_["camera_id"] = sc::get<int>("camera_id", params, camera_id_);
    plugin_params_["img_width"] = sc::get<int>("img_width", params, 800);
    plugin_params_["img_height"] = sc::get<int>("img_height", params, 600);
    plugin_params_["max_detect_range"] = sc::get<double>("max_detect_range", params, 1000);
    plugin_params_["focal_length"] = sc::get<double>("focal_length", params, 1.0);
    plugin_params_["fps"] = sc::get<double>("frames_per_second", params, 10);
    plugin_params_["az_thresh"] = sc::Angles::deg2rad(sc::get<double>("azimuth_fov", params, 360));
    plugin_params_["el_thresh"] = sc::Angles::deg2rad(sc::get<double>("elevation_fov", params, 360));
    plugin_params_["fn_prob"] = sc::get<double>("false_negative_probability", params, 0.1);
    plugin_params_["fp_prob"] = sc::get<double>("false_positive_probability", params, 0.1);
    plugin_params_["max_false_positives"] = sc::get<int>("max_false_positives_per_frame", params, 10);
    plugin_params_["std_dev_w"] = sc::get<int>("std_dev_width", params, 10);
    plugin_params_["std_dev_h"] = sc::get<int>("std_dev_height", params, 10);

    set_plugin_params(plugin_params_);

    if (log_detections_) {
        detections_file_.open(parent_->mp()->log_dir() + "/blob_sensor_detections_" + std::to_string(camera_id_) + ".txt");
    }

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

    // Initialize frustum shapes
    frustum_shapes_.resize(8);
    for (auto&& shape : frustum_shapes_) {
        shape = std::make_shared<scrimmage_proto::Shape>();
        sc::set(shape->mutable_color(), 0, 255, 0);
        shape->set_opacity(1.0);
        shape->set_persistent(false);
        shape->set_persist_duration(0.0);
    }
}

void ContactBlobCamera::draw_frustum(const std::vector<scrimmage_proto::ShapePtr>& frustum_shapes, double x_rot, double y_rot, double z_rot) {
    if (frustum_shapes.size() != 8) {
        std::cerr << "ContactBlobCamera::draw_frustum: ERROR: input shape vector must be size 8" << std::endl;
        return;
    }
    double sensor_footprint_height = max_detect_range_ * tan(el_thresh_ / 2) * 2;
    double sensor_footprint_width = max_detect_range_ * tan(az_thresh_ / 2) * 2;
    Eigen::Vector3d sensor_UL(max_detect_range_,  sensor_footprint_width / 2, -sensor_footprint_height / 2);
    Eigen::Vector3d sensor_UR(max_detect_range_,  sensor_footprint_width / 2,  sensor_footprint_height / 2);
    Eigen::Vector3d sensor_LR(max_detect_range_, -sensor_footprint_width / 2,  sensor_footprint_height / 2);
    Eigen::Vector3d sensor_LL(max_detect_range_, -sensor_footprint_width / 2, -sensor_footprint_height / 2);
    std::vector<Eigen::Vector3d> sensor_scene_bb = {sensor_UL, sensor_UR, sensor_LR, sensor_LL, sensor_UL};

    // Rotate into sensor frame
    auto x_rotation = Eigen::AngleAxisd(x_rot, Eigen::Vector3d::UnitX());
    auto y_rotation = Eigen::AngleAxisd(y_rot, Eigen::Vector3d::UnitY());
    auto z_rotation = Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ());
    auto A = Eigen::Translation3d(parent_->state_truth()->pos()) * z_rotation * y_rotation * x_rotation;
    for (size_t i = 0; i < sensor_scene_bb.size(); i++) {
        sensor_scene_bb[i] = A * sensor_scene_bb[i];
    }

    // Draw scene box
    auto scene_bb_line = std::make_shared<sp::Shape>();
    scene_bb_line->set_opacity(1.0);
    scene_bb_line->set_persistent(false);
    scene_bb_line->set_persist_duration(0.0);
    sc::set(scene_bb_line->mutable_color(), 0, 255, 0);

    sc::path_to_lines(sensor_scene_bb, scene_bb_line,
                      std::static_pointer_cast<EntityPlugin>(shared_from_this()));
    // draw 8 lines for the 4 points
    const Eigen::Vector3d fov_edge_start = parent_->state_truth()->pos();
    for (int idx_bb = 0; idx_bb != 4; ++idx_bb) {
        // box edges are idx 0-3 in frustum, fov edges are 4-7
        const size_t idx_fov_edge = idx_bb + 4;

        // set box line
        Eigen::Vector3d box_edge_start;
        const Eigen::Vector3d box_edge_end = sensor_scene_bb.at(idx_bb);
        if (idx_bb == 0) {
            box_edge_start = sensor_scene_bb.back();
        } else {
            box_edge_start = sensor_scene_bb.at(idx_bb-1);
        }
        scrimmage::set(frustum_shapes.at(idx_bb)->mutable_line()->mutable_start(), box_edge_start);
        scrimmage::set(frustum_shapes.at(idx_bb)->mutable_line()->mutable_end(), box_edge_end);

        // set fov line
        scrimmage::set(frustum_shapes.at(idx_fov_edge)->mutable_line()->mutable_start(), fov_edge_start);
        scrimmage::set(frustum_shapes.at(idx_fov_edge)->mutable_line()->mutable_end(), box_edge_end);
    }
    // draw all lines
    for (auto&& shape : frustum_shapes) {
        draw_shape(shape);
    }
}

void ContactBlobCamera::contacts_to_bounding_boxes(
    const scrimmage::State &sensor_frame,
    scrimmage::ContactMap &contacts,
    std::shared_ptr<sc::Message<ContactBlobCameraType>> &msg) {

    for (auto &kv : contacts) {
        // Filter out (skip) own contact
        if (kv.second.id().id() == parent_->id().id()) continue;

        // Filter out contacts out of range, should use RTree, but rtree still
        // requires querying of ID from contact lists. (TODO)
        if ((kv.second.state()->pos() - sensor_frame.pos()).norm() >
            max_detect_range_) {
            continue;
        }

        // Don't "detect" current contact relative to false negative probability
        double r = parent_->random()->rng_uniform(0.0, 1.0);
        if (r < fn_prob_) continue;

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

        if (object_img_radius > 0) {
            draw_object_with_bounding_box(msg->data.frame, kv.second.id().id(),
                                          rect, raster_center, object_img_radius);
        }

        // Collect all bounding boxes for current detected object
        msg->data.bounding_boxes[kv.second.id().id()].push_back(rect);
    }
}

void ContactBlobCamera::add_false_positives(
    std::shared_ptr<scrimmage::Message<ContactBlobCameraType>> &msg) {

    for (int i = 0; i < max_false_positives_; i++) {
        double r = parent_->random()->rng_uniform(0.0, 1.0);
        if (r > fp_prob_) continue;

        // Generate random center position in frame
        Eigen::Vector2d raster_center(
            parent_->random()->rng_uniform_int(0, img_width_),
            parent_->random()->rng_uniform_int(0, img_height_));

        double object_img_radius = parent_->random()->rng_uniform(0.1, 20.0);

        // Add bounding box to frame around object
        cv::Rect rect(std::floor(raster_center(0))-std::floor(object_img_radius),
                      std::floor(raster_center(1))-std::floor(object_img_radius),
                      std::floor(object_img_radius*2) + 1,
                      std::floor(object_img_radius*2) + 1);

        // Generate a random ID
        int id = parent_->random()->rng_uniform_int(1, 100);

        if (object_img_radius > 0) {
            draw_object_with_bounding_box(msg->data.frame, id, rect,
                                          raster_center, object_img_radius);
        }
        msg->data.bounding_boxes[id].push_back(rect);
    }
}

bool ContactBlobCamera::step() {
    if ((time_->t() - last_frame_t_) < 1.0 / fps_) return true;

    sc::State sensor_frame;
    sensor_frame.quat() =
            static_cast<sc::Quaternion>(parent_->state_truth()->quat() *
                                        this->transform()->quat());
    sensor_frame.pos() = this->transform()->pos() + parent_->state_truth()->pos();

    if (show_frustum_) {
        draw_frustum(frustum_shapes_, sensor_frame.quat().roll(), sensor_frame.quat().pitch(), sensor_frame.quat().yaw());
    }

    auto msg = std::make_shared<sc::Message<ContactBlobCameraType>>();

    msg->data.camera_id = camera_id_;
    msg->data.img_width = img_width_;
    msg->data.img_height = img_height_;
    msg->data.max_detect_range = max_detect_range_;
    msg->data.focal_length = focal_length_;
    msg->data.fps = fps_;
    msg->data.az_thresh = az_thresh_;
    msg->data.el_thresh = el_thresh_;
    msg->data.fn_prob = fn_prob_;
    msg->data.fp_prob = fp_prob_;
    msg->data.max_false_positives = max_false_positives_;

    msg->data.frame = cv::Mat::zeros(img_height_, img_width_, CV_8UC3);

    if (not ignore_real_entities_) {
        // Compute bounding boxes for real contacts
        contacts_to_bounding_boxes(sensor_frame, *(parent_->contacts()), msg);
    }

    // Compute bounding boxes for added "simulated" contacts
    contacts_to_bounding_boxes(sensor_frame, sim_contacts_, msg);

    if (show_sim_contacts_ &&
        time_->t() > last_contact_send_time_ + contact_send_dt_) {
        // draw_sim_contacts(sim_contacts_);
        for (auto &kv : sim_contacts_) {
            // Draw a sphere at the expected commanded state
            Eigen::Vector3d sphere_center(kv.second.state()->pos().x(),
                                          kv.second.state()->pos().y(),
                                          kv.second.state()->pos().z());

            sc::set(sim_tgt_sphere_->mutable_sphere()->mutable_center(), sphere_center);
            sc::set(sim_tgt_sphere_->mutable_color(), 0, 255, 255);
            sim_tgt_sphere_->mutable_sphere()->set_radius(kv.second.radius());
            sim_tgt_sphere_->set_opacity(0.7);
            sim_tgt_sphere_->set_persistent(false);
            draw_shape(sim_tgt_sphere_);
        }

        last_contact_send_time_ = time_->t();
    }

    // Add false positives
    add_false_positives(msg);

    frame_ = msg->data.frame;
    last_frame_t_ = time_->t();

    if (show_image_) {
        std::string window_name = window_name_ + "-" +
                std::to_string(parent_->id().id()) + "-" +
                std::to_string(camera_id_);
        cv::imshow(window_name.c_str(), msg->data.frame);
        cv::waitKey(1);
    }

    // Log any detections
    if (log_detections_) {
        detections_file_ << time_->t();
        for (auto elem : msg->data.bounding_boxes) {
            detections_file_ << ", " << elem.first;
        }
        detections_file_ << std::endl;
    }

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

void ContactBlobCamera::draw_object_with_bounding_box(
    cv::Mat &frame, const int &id, const cv::Rect &rect,
    const Eigen::Vector2d &center, const double &radius) {

    Eigen::Vector2i c(std::floor(center(0)),
                      std::floor(center(1)));

    cv::circle(frame, cv::Point(c(0), c(1)),
               std::floor(radius), cv::Scalar(255, 255, 255),
               -1, 8, 0);

    cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 1, 8, 0);

    cv::Point top_left(rect.x + rect.width, rect.y + rect.height);
    cv::putText(frame, std::to_string(id), top_left,  cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(8, 100, 22), 1, 8, false);
}

void ContactBlobCamera::set_plugin_params(std::map<std::string, double> params) {
    if (params["senderId"] != parent_->id().id()) { return; }

    camera_id_ = static_cast<int>(params["camera_id"]);

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
