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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_RAYTRACE_RAYTRACE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_RAYTRACE_RAYTRACE_H_

#include <scrimmage/sensor/Sensor.h>

#include <random>
#include <map>
#include <string>
#include <vector>
#include <memory>

namespace scrimmage {
namespace sensor {
class RayTrace : public scrimmage::Sensor {
 public:
    class PCPoint {
     public:
        PCPoint() : point(Eigen::Vector3d(0, 0, 0)), intensity(0), oor(false) {}
        explicit PCPoint(Eigen::Vector3d p) : point(p), intensity(0), oor(false) {}
        PCPoint(Eigen::Vector3d p, double i, bool o) : point(p), intensity(i), oor(o) {}
        Eigen::Vector3d point;
        double intensity;
        bool oor;  // Out of range
    };

    class PointCloud {
     public:
        PointCloud() : max_range(0), min_range(0), num_rays_vert(0),
            num_rays_horiz(0), angle_res_vert(0), angle_res_horiz(0),
            max_sample_rate(0) {}

        std::vector<PCPoint> points;
        double max_range;
        double min_range;
        double num_rays_vert;
        double num_rays_horiz;
        double angle_res_vert;
        double angle_res_horiz;
        double max_sample_rate;
    };

    RayTrace();

    std::string name() override { return "RayTrace"; }
    std::string type() override { return "Ray"; }
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;

    double angle_res_vert() { return angle_res_vert_; }
    double angle_res_horiz() { return angle_res_horiz_; }
    int num_rays_vert() { return num_rays_vert_; }
    int num_rays_horiz() { return num_rays_horiz_; }
    double max_range() { return max_range_; }
    double min_range() { return min_range_; }
    double max_sample_rate() { return max_sample_rate_; }

 protected:
    std::shared_ptr<std::default_random_engine> gener_;
    std::vector<std::shared_ptr<std::normal_distribution<double>>> pos_noise_;

    double angle_res_vert_;
    double angle_res_horiz_;
    int num_rays_vert_;
    int num_rays_horiz_;
    double max_range_;
    double min_range_;
    double max_sample_rate_;

    PublisherPtr pub_;
};
}  // namespace sensor
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_RAYTRACE_RAYTRACE_H_
