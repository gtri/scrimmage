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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_CONTACTBLOBCAMERA_CONTACTBLOBCAMERA_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_CONTACTBLOBCAMERA_CONTACTBLOBCAMERA_H_

#include <Eigen/Dense>

#include <scrimmage/sensor/Sensor.h>

#include <random>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace scrimmage {
namespace sensor {
class ContactBlobCamera : public scrimmage::Sensor {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;

 protected:
    std::shared_ptr<std::default_random_engine> gener_;
    std::vector<std::shared_ptr<std::normal_distribution<double>>> pos_noise_;
    std::vector<std::shared_ptr<std::normal_distribution<double>>> orient_noise_;

    Eigen::Vector2d project_rel_3d_to_2d(Eigen::Vector3d rel_pos);
    bool in_field_of_view(Eigen::Vector3d rel_pos);
    void draw_object_with_bounding_box(cv::Mat frame, cv::Rect rect,
                                       Eigen::Vector2d center, double radius);

    double max_detect_range_;
    double az_thresh_;
    double el_thresh_;

    double fn_prob_;
    double fp_prob_;
    int max_false_positives_;
    int std_dev_w_;
    int std_dev_h_;

    int img_width_;
    int img_height_;

    double focal_length_;
    double canvas_width_;
    double canvas_height_;

    cv::Mat frame_;
    double fps_;
    double last_frame_t_;

    PublisherPtr pub_;
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_CONTACTBLOBCAMERA_CONTACTBLOBCAMERA_H_
