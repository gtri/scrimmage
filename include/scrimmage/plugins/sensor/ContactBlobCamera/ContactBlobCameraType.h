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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_CONTACTBLOBCAMERA_CONTACTBLOBCAMERATYPE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_CONTACTBLOBCAMERA_CONTACTBLOBCAMERATYPE_H_

#include <map>
#include <vector>
#include <opencv2/core/core.hpp>

namespace scrimmage {
namespace sensor {
class ContactBlobCameraType {
 public:
    cv::Mat frame;
    std::map<int, std::vector<cv::Rect>> bounding_boxes;
    int camera_id;
    int img_width;
    int img_height;
    double max_detect_range;
    double focal_length;
    double fps;
    double az_thresh;
    double el_thresh;
    double fn_prob;
    double fp_prob;
    int max_false_positives;
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_CONTACTBLOBCAMERA_CONTACTBLOBCAMERATYPE_H_
