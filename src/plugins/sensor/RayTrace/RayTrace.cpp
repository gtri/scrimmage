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

#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>

#include <scrimmage/plugins/sensor/RayTrace/RayTrace.h>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::RayTrace, RayTrace_plugin)

namespace scrimmage {
namespace sensor {

std::vector<RayTrace::PCRay> RayTrace::PointCloud::get_rays() {
    std::vector<RayTrace::PCRay> return_rays;
    for (auto &ray : rays) {
        // Make a copy so can be used cleanly
        return_rays.push_back(RayTrace::PCRay(ray.azimuth_rad, ray.elevation_rad));
    }
    return return_rays;
}

RayTrace::RayTrace() : max_range_(0), min_range_(0), max_sample_rate_(0) {}

void RayTrace::init(std::map<std::string, std::string> &params) {
    // Use the same generator as the parent so that the simulation is
    // completely deterministic with respect to the simulation seed.
    gener_ = parent_->random()->gener();

    // Create three independent gaussian noise generators. They will use the
    // same generator seed.
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

    max_range_ = sc::get<double>("max_range", params, 1);
    min_range_ = sc::get<double>("min_range", params, 1);
    max_sample_rate_ = sc::get<double>("max_sample_rate", params, 1);
    auto_ray_tracing_ = sc::get<bool>("auto_ray_tracing", params, true);

    // Create the ray definitions
    double angle_res_vert = sc::Angles::deg2rad(sc::get<double>("angle_res_vert", params, 1));
    double angle_res_horiz = sc::Angles::deg2rad(sc::get<double>("angle_res_horiz", params, 1));
    int num_rays_vert = sc::get<int>("num_rays_vert", params, 1);
    int num_rays_horiz = sc::get<int>("num_rays_horiz", params, 1);
    rays_.clear();

    double fov_horiz = angle_res_horiz * (num_rays_horiz - 1);
    double fov_vert = angle_res_vert * (num_rays_vert - 1);
    double start_angle_horiz = -fov_horiz / 2.0;
    double start_angle_vert = -fov_vert / 2.0;

    double angle_vert = start_angle_vert;
    for (int v = 0; v < num_rays_vert; v++) {
        double angle_horiz = start_angle_horiz;
        for (int h = 0; h < num_rays_horiz; h++) {
            rays_.push_back(PCRay(angle_horiz, angle_vert));

            angle_horiz += angle_res_horiz;
        }
        angle_vert += angle_res_vert;
    }

    // Create the publisher
    pub_ = advertise("LocalNetwork", "RayTrace");
}

bool RayTrace::step() {
    // pub_->publish(std::make_shared<MessageBase>());
    return true;
}

std::vector<RayTrace::PCRay> RayTrace::rays() {
    std::vector<RayTrace::PCRay> return_rays;
    for (auto &ray : rays_) {
        // Make a copy so can be used cleanly
        return_rays.push_back(RayTrace::PCRay(ray.azimuth_rad, ray.elevation_rad));
    }
    return return_rays;
}

}  // namespace sensor
}  // namespace scrimmage
