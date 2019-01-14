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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_PLANE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_PLANE_H_

#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <limits>
#include <tuple>
#include <string>

using std::cout;
using std::endl;

namespace sc = scrimmage;

namespace scrimmage {
namespace interaction {

class Plane : public BoundaryBase {
 public:
    Plane() {
    }

    Plane(Eigen::Vector3d center, double x_length, double y_width,
           scrimmage::Quaternion quat, std::string texture,
           bool diffuse_lighting) {
        double x = x_length / 2.0;
        double y = y_width / 2.0;
        double z = center(2);
        center_ = center;

        normZ = quat * -Eigen::Vector3d::UnitZ();
        normY = quat * Eigen::Vector3d::UnitY();
        normX = quat * Eigen::Vector3d::UnitX();

        std::vector<Eigen::Vector3d> points;
        points.push_back(Eigen::Vector3d(x, y, z));
        points.push_back(Eigen::Vector3d(-x, y, z));
        points.push_back(Eigen::Vector3d(-x, -y, z));
        points.push_back(Eigen::Vector3d(x, -y, z));

        // TODO: Handle rotation

        for (Eigen::Vector3d &p : points) {
            p += center;
        }
        set_points(points);
    }

    explicit Plane(const scrimmage_proto::Shape &shape) :
    Plane(proto_2_vector3d(shape.plane().center()),
           shape.plane().x_length(), shape.plane().y_length(),
            proto_2_quat(shape.plane().quat()), shape.plane().texture(),
            shape.plane().diffuse_lighting()) {
        set_visual(shape.color().r(), shape.color().g(), shape.color().b(),
                   shape.opacity());
    }

    void compute_dots() {
        u = normZ;
        v = -1*normX;
        w = normX;
        s = normY;
        r = -1*normY;

        // up or down
        u_dot_P0 = u.dot(center_ - points_[0]);

        // inside bounds of plane
        w_dot_P0 = w.dot(center_ - points_[0]);
        s_dot_P1 = s.dot(center_ - points_[1]);
        v_dot_P2 = v.dot(center_ - points_[2]);
        r_dot_P3 = r.dot(center_ - points_[3]);
    }

    bool contains(Eigen::Vector3d p) override {
        double u_dot_p = u.dot(p - center_);
        double w_dot_p = w.dot(p - center_);
        double v_dot_p = v.dot(p - center_);
        double r_dot_p = r.dot(p - center_);
        double s_dot_p = s.dot(p - center_);

        if ((u_dot_p > u_dot_P0) && (w_dot_p > w_dot_P0) &&
            (v_dot_p > v_dot_P2) && (s_dot_p > s_dot_P1) &&
            (r_dot_p > r_dot_P3)) {
            return true;
        }
        return false;
    }

    const std::vector<Eigen::Vector3d> & points() {
        return points_;
    }

    void set_points(std::vector<Eigen::Vector3d> &points) {
        points_ = points;
        compute_dots();

        Eigen::Vector3d xy_center = (points[0] + points[2]) / 2;
        double alt_center = (points[0](2) + points[3](2)) / 2;
        center_ << xy_center(0), xy_center(1), alt_center;

        // Compute min / max values for x, y, z
        Eigen::Vector3d mins(std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity());
        Eigen::Vector3d maxs(-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity());
        for (Eigen::Vector3d p : points) {
            for (int i = 0; i < 3; i++) {
                if (p(i) < mins(i)) mins(i) = p(i);
                if (p(i) > maxs(i)) maxs(i) = p(i);
            }
        }
        extents_.clear();
        extents_.push_back(std::tuple<double, double>(mins(0), maxs(0))); // x bounds
        extents_.push_back(std::tuple<double, double>(mins(1), maxs(1))); // y bounds
        extents_.push_back(std::tuple<double, double>(mins(2), maxs(2))); // z bounds
    }

    void set_visual(int R, int G, int B, double opacity) override {
        // Generate the shape
        if (points_.size() != 4) {
            cout << "Invalid number of cube points: " << points_.size() << endl;
            return;
        }

        const int num_faces = 2;
        const int vert_per_face = 4;
        int vert_lookup[num_faces][vert_per_face] = {
            {0, 1, 2, 3},
            {0, 1, 5, 4},
        };

        for (int f = 0; f < num_faces; f++) {
            sc::ShapePtr polygon(new sp::Shape);
            polygon->set_opacity(opacity);
            polygon->set_persistent(true);
            sc::set(polygon->mutable_color(), R, G, B);

            for (int r = 0; r < vert_per_face; r++) {
                sp::Vector3d * p = polygon->mutable_polygon()->add_point();
                sc::set(p, points_[vert_lookup[f][r]]);
            }
            // shapes_.push_back(polygon);
        }
    }

 protected:
    std::vector<Eigen::Vector3d> points_;
    Eigen::Vector3d u;
    Eigen::Vector3d w;
    Eigen::Vector3d v;
    Eigen::Vector3d s;
    Eigen::Vector3d r;
    Eigen::Vector3d normZ;
    Eigen::Vector3d normY;
    Eigen::Vector3d normX;
    double u_dot_P0 = 0;
    double w_dot_P0 = 0;
    double r_dot_P3 = 0;
    double v_dot_P2 = 0;
    double s_dot_P1 = 0;
};

} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_BOUNDARY_PLANE_H_
