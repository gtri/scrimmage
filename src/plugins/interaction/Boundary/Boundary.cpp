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
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>
#include <scrimmage/plugins/interaction/Boundary/Sphere.h>
#include <scrimmage/plugins/interaction/Boundary/Plane.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::Boundary,
                Boundary_plugin)

namespace scrimmage {
namespace interaction {

Boundary::Boundary() :
    boundary_shape_(std::make_shared<scrimmage_proto::Shape>()) {
}

bool Boundary::init(std::map<std::string, std::string> &mission_params,
                    std::map<std::string, std::string> &plugin_params) {
    // Parse common boundary properties

    // Parse boundary name
    boundary_shape_->set_name(sc::get<std::string>("name", plugin_params, "no_name"));

    // Parse boundary ID
    boundary_shape_->mutable_id()->set_id(sc::get<int>("id", plugin_params, 0));
    boundary_shape_->mutable_id()->set_team_id(sc::get<int>("team_id", plugin_params, 0));

    std::string network_name = sc::get("network_name", plugin_params, "GlobalNetwork");
    pub_boundary_ = advertise(network_name, "Boundary");

    // Parse boundary type
    std::string type = sc::get<std::string>("type",
                                            plugin_params,
                                            "cuboid");

    if (type == "cuboid") {
        std::vector<double> center;
        if (!sc::get_vec<double>("center", plugin_params, " ,", center, 3)) {
            std::cout << "Failed to parse 'center'" << endl;
            return false;
        }
        sc::set(boundary_shape_->mutable_cuboid()->mutable_center(), sc::vec2eigen(center));

        std::vector<double> lengths;
        if (!sc::get_vec<double>("lengths", plugin_params, " ,", lengths, 3)) {
            std::cout << "Failed to parse 'lengths'" << endl;
            return false;
        }
        boundary_shape_->mutable_cuboid()->set_x_length(lengths[0]);
        boundary_shape_->mutable_cuboid()->set_y_length(lengths[1]);
        boundary_shape_->mutable_cuboid()->set_z_length(lengths[2]);

        std::vector<double> rpy;
        if (!sc::get_vec<double>("rpy", plugin_params, " ,", rpy, 3)) {
            std::cout << "Failed to parse 'rpy'" << endl;
            return false;
        }
        sc::Quaternion quat(rpy[0], rpy[1], rpy[2]);
        sc::set(boundary_shape_->mutable_cuboid()->mutable_quat(), quat);

    } else if (type == "polyhedron") {
        std::string polyhedron_points = sc::get<std::string>("polyhedron_points",
                                                             plugin_params, "");

        std::vector<std::vector<std::string>> vecs;
        if (!sc::get_vec_of_vecs(polyhedron_points, vecs)) {
            cout << "Failed to parse polyhedron_points." << endl;
            return false;
        }

        // Convert string representation of points into Eigen::Vector3d
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 3) {
                cout << "Invalid vector size in: " << polyhedron_points << endl;
                return false;
            }
            sp::Vector3d *point = boundary_shape_->mutable_polyhedron()->add_point();
            point->set_x(std::stod(vec[0]));
            point->set_y(std::stod(vec[1]));
            point->set_z(std::stod(vec[2]));
        }
    } else if (type == "sphere") {
        boundary_shape_->mutable_sphere()->set_radius(sc::get<double>("radius", plugin_params, 10));
        std::vector<double> center;
        if (!sc::get_vec<double>("center", plugin_params, " ,", center, 3)) {
            std::cout << "Failed to parse 'center'" << endl;
            return false;
        }
        sc::set(boundary_shape_->mutable_sphere()->mutable_center(), sc::vec2eigen(center));
    } else if (type == "plane") {
        std::vector<double> center;
        if (!sc::get_vec<double>("center", plugin_params, " ,", center, 3)) {
            std::cout << "Failed to parse 'center'" << endl;
            return false;
        }
        sc::set(boundary_shape_->mutable_plane()->mutable_center(), sc::vec2eigen(center)); // TODO

        std::vector<double> rpy;
        if (!sc::get_vec<double>("rpy", plugin_params, " ,", rpy, 3)) {
            std::cout << "Failed to parse 'rpy'" << endl;
            return false;
        }
        sc::Quaternion quat(rpy[0], rpy[1], rpy[2]);
        sc::set(boundary_shape_->mutable_plane()->mutable_quat(), quat);

        std::vector<double> lengths;
        if (!sc::get_vec<double>("lengths", plugin_params, " ,", lengths, 2)) {
            std::cout << "Failed to parse 'lengths'" << endl;
            return false;
        }
        boundary_shape_->mutable_plane()->set_x_length(lengths[0]);
        boundary_shape_->mutable_plane()->set_y_length(lengths[1]);

        std::string texture;
        texture = sc::get<std::string>("texture", plugin_params, "");
        boundary_shape_->mutable_plane()->set_texture(texture);

        bool diffuse;
        diffuse = sc::get<bool>("diffuse_lighting", plugin_params, false);
        boundary_shape_->mutable_plane()->set_diffuse_lighting(diffuse);

    } else {
        cout << "Invalid type: " << type << endl;
        return false;
    }

    if (sc::get<bool>("show_boundary", plugin_params, false)) {
        double opacity = sc::get<double>("opacity", plugin_params,
                                         1.0);
        std::vector<int> color;
        if (!sc::get_vec("color", plugin_params, " ", color, 3)) {
            cout << "Failed to parse color" << endl;
            color.clear();
            color = {255, 0, 0};
        }

        sc::set(boundary_shape_->mutable_color(), color[0], color[1], color[2]);
        boundary_shape_->set_opacity(opacity);
        boundary_shape_->set_persistent(true);
        draw_shape(boundary_shape_);
    }


    return true;
}

bool Boundary::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (!boundary_published_) {
        boundary_published_ = true;
        auto msg = std::make_shared<sc::Message<sp::Shape>>(*boundary_shape_);
        msg->data = *boundary_shape_;
        pub_boundary_->publish(msg);
    }
    return true;
}

std::shared_ptr<BoundaryBase> Boundary::make_boundary(const scrimmage_proto::Shape &shape) {
    std::shared_ptr<BoundaryBase> boundary = nullptr;
    switch (shape.oneof_type_case()) {
    case sp::Shape::kTriangle:
        break;
    case sp::Shape::kPlane:
        boundary = std::make_shared<Plane>(shape);
        break;
    case sp::Shape::kArrow:
        break;
    case sp::Shape::kCone:
        break;
    case sp::Shape::kLine:
        break;
    case sp::Shape::kPolygon:
        break;
    case sp::Shape::kPolydata:
        break;
    case sp::Shape::kCuboid:
        boundary = std::make_shared<Cuboid>(shape);
        break;
    case sp::Shape::kPointcloud:
        break;
    case sp::Shape::kCircle:
        break;
    case sp::Shape::kSphere:
        boundary = std::make_shared<Sphere>(shape);
        break;
    case sp::Shape::kText:
        break;
    default:
        break;
    }

    if (boundary != nullptr) {
        sc::ShapePtr shape_ptr = std::make_shared<scrimmage_proto::Shape>(shape);
        boundary->set_shape(shape_ptr);
    }

    return boundary;
}


} // namespace interaction
} // namespace scrimmage
