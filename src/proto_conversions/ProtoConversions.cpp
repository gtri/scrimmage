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

#include <scrimmage/common/ID.h>
#include <scrimmage/common/ColorMaps.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/log/Frame.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/Frame.pb.h>

#include <cmath>

namespace scrimmage {

void set(scrimmage_proto::Vector3d *dst, Eigen::Vector3d src) {
    dst->set_x(src(0));
    dst->set_y(src(1));
    dst->set_z(src(2));
}

void set(scrimmage_proto::Vector3d *dst, double x, double y, double z) {
    dst->set_x(x);
    dst->set_y(y);
    dst->set_z(z);
}

void set(scrimmage_proto::Color *dst, const std::vector<int> &src) {
    dst->set_r(src[0]);
    dst->set_g(src[1]);
    dst->set_b(src[2]);
}

void set(Eigen::Vector3d &dst, scrimmage_proto::Vector3d *src) {
    dst(0) = src->x();
    dst(1) = src->y();
    dst(2) = src->z();
}

void set(scrimmage_proto::Color &dst, const scrimmage_proto::Color &src) {
    dst.set_r(src.r());
    dst.set_g(src.g());
    dst.set_b(src.b());
}

void set(scrimmage_proto::Color &dst, scrimmage_proto::Color &src) {
    dst.set_r(src.r());
    dst.set_g(src.g());
    dst.set_b(src.b());
}

void set(scrimmage_proto::Color &dst, int r, int g, int b) {
    scrimmage_proto::Color src;
    src.set_r(r);
    src.set_g(g);
    src.set_b(b);
    set(dst, src);
}

void set(scrimmage_proto::Color *dst, scrimmage_proto::Color *src) {
    *dst = *src;
}

void set(scrimmage_proto::Color *dst, scrimmage_proto::Color src) {
    dst->set_r(src.r());
    dst->set_g(src.g());
    dst->set_b(src.b());
}

void set(scrimmage_proto::Color *dst, const scrimmage::Color_t &src) {
    dst->set_r(src.r);
    dst->set_g(src.g);
    dst->set_b(src.b);
}

void set(scrimmage_proto::Color *color, int r, int g, int b) {
    scrimmage::Color_t c;
    c.r = r;
    c.g = g;
    c.b = b;

    set(color, c);
}

void set(scrimmage_proto::Color *color, int grayscale) {
    Color_t c = GetColor_matlab(grayscale, 0, 255);
    set(color, c);
}

void set(scrimmage_proto::Quaternion *dst, Quaternion &src) {
    dst->set_x(src.x());
    dst->set_y(src.y());
    dst->set_z(src.z());
    dst->set_w(src.w());
}

void set(scrimmage_proto::Quaternion *dst, const double &w, const double &x,
         const double &y, const double &z) {
    dst->set_x(x);
    dst->set_y(y);
    dst->set_z(z);
    dst->set_w(w);
}

void set(scrimmage_proto::State *dst, const scrimmage::StatePtr &state) {
    set(dst->mutable_position(), state->pos());
    set(dst->mutable_linear_velocity(), state->vel());
    set(dst->mutable_angular_velocity(), state->ang_vel());
    set(dst->mutable_orientation(), state->quat());
}

void set(scrimmage::Quaternion &dst, const scrimmage_proto::Quaternion &quat) {
    dst.set(quat.w(), quat.x(), quat.y(), quat.z());
}

void set(scrimmage::State &dst, const scrimmage_proto::State &state) {
    set(dst.pos(), state.position());
    set(dst.vel(), state.linear_velocity());
    set(dst.ang_vel(), state.angular_velocity());
    set(dst.quat(), state.orientation());
}

void set(scrimmage_proto::State *dst, scrimmage::State &state) {
    set(dst->mutable_position(), state.pos());
    set(dst->mutable_linear_velocity(), state.vel());
    set(dst->mutable_angular_velocity(), state.ang_vel());
    set(dst->mutable_orientation(), state.quat());
}

Eigen::Vector3d eigen(const scrimmage_proto::Vector3d &src) {
    Eigen::Vector3d dst;
    dst(0) = src.x();
    dst(1) = src.y();
    dst(2) = src.z();
    return dst;
}

void add_point_color(std::shared_ptr<scrimmage_proto::PointCloud> s,
                     const scrimmage::Color_t &c) {
    scrimmage_proto::Color *color = s->add_color();
    set(color, c);
}

void add_point_color(std::shared_ptr<scrimmage_proto::PointCloud> s,
                     const int &r, const int &g, const int &b) {
    scrimmage_proto::Color *color = s->add_color();
    set(color, r, g, b);
}

void add_point_color(std::shared_ptr<scrimmage_proto::PointCloud> s,
                     const int &grayscale) {
    scrimmage_proto::Color *color = s->add_color();
    set(color, grayscale);
}

void set(scrimmage::ID &id, const scrimmage_proto::ID &proto_id) {
    id.set_id(proto_id.id());
    id.set_sub_swarm_id(proto_id.sub_swarm_id());
    id.set_team_id(proto_id.team_id());
}

Eigen::Vector3d proto_2_vector3d(const scrimmage_proto::Vector3d &proto_vector3d) {
    return Eigen::Vector3d(proto_vector3d.x(), proto_vector3d.y(), proto_vector3d.z());
}

Quaternion proto_2_quat(const scrimmage_proto::Quaternion &proto_quat) {
    const Quaternion quat(proto_quat.w(), proto_quat.x(), proto_quat.y(), proto_quat.z());
    return quat;
}

void set(Eigen::Vector3d &dst, const scrimmage_proto::Vector3d &proto_vector3d) {
    dst << proto_vector3d.x(), proto_vector3d.y(), proto_vector3d.z();
}

StatePtr proto_2_state(const scrimmage_proto::State &proto_state) {
    State state;
    set(state.pos(), proto_state.position());
    set(state.vel(), proto_state.linear_velocity());
    set(state.ang_vel(), proto_state.angular_velocity());
    state.set_quat(proto_2_quat(proto_state.orientation()));
    return std::make_shared<State>(state);
}

void path_to_lines(std::vector<Eigen::Vector3d> &path,
                   std::shared_ptr<scrimmage_proto::Shape> sample_line,
                   std::shared_ptr<Plugin> p) {
    for (size_t i = 0; i < path.size() - 1; i++) {
        auto ln = std::make_shared<scrimmage_proto::Shape>();
        ln->CopyFrom(*sample_line);
        scrimmage::set(ln->mutable_line()->mutable_start(), path[i]);
        scrimmage::set(ln->mutable_line()->mutable_end(), path[i + 1]);
        p->draw_shape(ln);
    }
}

Frame proto_2_frame(const scrimmage_proto::Frame &proto_frame) {
    Frame frame;

    frame.contacts_ = std::make_shared<ContactMap>();
    frame.time_ = proto_frame.time();
    ContactMap &contacts = *(frame.contacts_);

    for (int i = 0; i < proto_frame.contact_size(); i++) {
        contacts[proto_frame.contact(i).id().id()] =
                proto_2_contact(proto_frame.contact(i));
    }
    return frame;
}

std::shared_ptr<scrimmage_proto::Frame> create_frame(double time, std::shared_ptr<ContactMap> &contacts) {
    std::shared_ptr<scrimmage_proto::Frame> frame(new scrimmage_proto::Frame());
    frame->set_time(time);

    for (auto &kv : *contacts) {
        StatePtr &state = kv.second.state();
        Contact::Type type = kv.second.type();
        const ID &id = kv.second.id();

        scrimmage_proto::Contact *contact = frame->add_contact();
        scrimmage_proto::ID *sp_id = contact->mutable_id();

        set(contact->mutable_state(), state);

        switch (type) {
        case Contact::Type::AIRCRAFT:
            contact->set_type(scrimmage_proto::AIRCRAFT);
            break;
        case Contact::Type::QUADROTOR:
            contact->set_type(scrimmage_proto::QUADROTOR);
            break;
        case Contact::Type::SPHERE:
            contact->set_type(scrimmage_proto::SPHERE);
            break;
        case Contact::Type::MESH:
            contact->set_type(scrimmage_proto::MESH);
            break;
        default:
            contact->set_type(scrimmage_proto::UNKNOWN);
            break;
        }

        contact->set_active(kv.second.active());

        sp_id->set_id(id.id());
        sp_id->set_sub_swarm_id(id.sub_swarm_id());
        sp_id->set_team_id(id.team_id());
    }
    return frame;
}

Contact proto_2_contact(const scrimmage_proto::Contact &proto_contact) {
    Contact contact;
    set(contact.id(), proto_contact.id());

    if (proto_contact.type() == scrimmage_proto::AIRCRAFT) {
        contact.set_type(Contact::Type::AIRCRAFT);
    } else if (proto_contact.type() == scrimmage_proto::QUADROTOR) {
        contact.set_type(Contact::Type::QUADROTOR);
    } else if (proto_contact.type() == scrimmage_proto::SPHERE) {
        contact.set_type(Contact::Type::SPHERE);
    } else if (proto_contact.type() == scrimmage_proto::MESH) {
        contact.set_type(Contact::Type::MESH);
    } else {
        contact.set_type(Contact::Type::UNKNOWN);
    }

    StatePtr state = proto_2_state(proto_contact.state());

    contact.set_state(state);

    return contact;
}

} // namespace scrimmage
