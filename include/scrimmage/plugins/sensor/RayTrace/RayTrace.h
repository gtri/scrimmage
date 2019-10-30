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

    /**
     * Defines a single ray for the point cloud in terms of azimuth and elevation angles from center
     */
    class PCRay {
     public:
        PCRay() : azimuth_rad(0.0), elevation_rad(0.0) {}
        PCRay(double in_az_rad, double in_el_rad) : azimuth_rad(in_az_rad), elevation_rad(in_el_rad) {}
        double azimuth_rad;
        double elevation_rad;
    };

    class PointCloud {
     public:
        PointCloud() : max_range(0), min_range(0), max_sample_rate(0) {}

        std::vector<PCRay> get_rays();

        std::vector<PCPoint> points;
        std::vector<PCRay> rays;
        double max_range;
        double min_range;
        double max_sample_rate;
    };

    class PointCloudWithId : public PointCloud {
     public:
        PointCloudWithId() :  PointCloud(), sensor_name("unknown"), entity_id(0), world_frame(false) {}

        std::string sensor_name;
        int entity_id;
        bool world_frame;  // Whether the point cloud is in the world frame (true) or sensor frame (false)
    };

    RayTrace();

    std::string type() override { return "Ray"; }
    void init(std::map<std::string, std::string> &params) override;
    bool step() override;

    /** Whether to have the bullet collision plugin automatically run ray tracing for this sensor and send associated messages */
    bool automatic_ray_tracing() { return auto_ray_tracing_; }
    /** Returns a copy of the rays*/
    std::vector<PCRay> rays();
    /** Sensor ranges and update rates */
    double max_range() { return max_range_; }
    double min_range() { return min_range_; }
    double max_sample_rate() { return max_sample_rate_; }

 protected:
    std::shared_ptr<std::default_random_engine> gener_;
    std::vector<std::shared_ptr<std::normal_distribution<double>>> pos_noise_;

    // Define all rays instead of max and min angle sweeps with resolution
    std::vector<PCRay> rays_;
    uint32_t ray_id_counter_{0};
    // Ranges used to cap the ray search for entities and ground
    double max_range_;
    double min_range_;
    // Sample rate used for update rate in collision models
    double max_sample_rate_;
    // Configuration item of whether to do automatic ray tracing - default true for backwards compatibility
    bool auto_ray_tracing_{true};

    PublisherPtr pub_;
};
}  // namespace sensor
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_SENSOR_RAYTRACE_RAYTRACE_H_
