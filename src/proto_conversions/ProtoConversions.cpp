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
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/Frame.pb.h>

#include <cmath>

namespace scrimmage {

void set(scrimmage_proto::Vector3d *dst, Eigen::Vector3d &src) {
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

Eigen::Vector3d eigen(const scrimmage_proto::Vector3d &src) {
    Eigen::Vector3d dst;
    dst(0) = src.x();
    dst(1) = src.y();
    dst(2) = src.z();
    return dst;
}

void add_point(std::shared_ptr<scrimmage_proto::Shape> s, Eigen::Vector3d src) {
    scrimmage_proto::Vector3d *point = s->add_point();
    set(point, src);
}

void add_point_color(std::shared_ptr<scrimmage_proto::Shape> s, const scrimmage::Color_t &c) {
    scrimmage_proto::Color *color = s->add_point_color();
    set(color, c);
}

void add_point_color(scrimmage_proto::ShapePtr s, int r, int g, int b) {
    scrimmage_proto::Color *color = s->add_point_color();
    set(color, r, g, b);
}

void add_point_color(scrimmage_proto::ShapePtr s, int grayscale) {
    scrimmage_proto::Color *color = s->add_point_color();
    set(color, grayscale);
}

ID proto_2_id(const scrimmage_proto::ID &proto_id) {
    ID id;
    id.set_id(proto_id.id());
    id.set_sub_swarm_id(proto_id.sub_swarm_id());
    id.set_team_id(proto_id.team_id());
    return id;
}

Quaternion proto_2_quat(scrimmage_proto::Quaternion proto_quat) {
    const Quaternion quat(proto_quat.w(), proto_quat.x(), proto_quat.y(), proto_quat.z());
    return quat;
}

Eigen::Vector3d proto_2_vector3d(scrimmage_proto::Vector3d proto_vector3d) {
    return Eigen::Vector3d(proto_vector3d.x(), proto_vector3d.y(), proto_vector3d.z());
}

StatePtr proto_2_state(const scrimmage_proto::State &proto_state) {
    State state;
    state.set_pos(proto_2_vector3d(proto_state.position()));
    state.set_vel(proto_2_vector3d(proto_state.velocity()));
    state.set_quat(proto_2_quat(proto_state.orientation()));
    return std::make_shared<State>(state);
}

void path_to_lines(std::vector<Eigen::Vector3d> &path, scrimmage_proto::Shape &sample_line, std::list<scrimmage_proto::ShapePtr> &shapes) {
    for (size_t i = 0; i < path.size() - 1; i++) {
        scrimmage_proto::ShapePtr ln(new scrimmage_proto::Shape());
        ln->CopyFrom(sample_line);
        add_point(ln, path[i]);
        add_point(ln, path[i + 1]);
        shapes.push_back(ln);
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
        Eigen::Vector3d &pos = state->pos();
        Eigen::Vector3d &vel = state->vel();
        Quaternion &quat = state->quat();
        Contact::Type type = kv.second.type();
        const ID &id = kv.second.id();

        scrimmage_proto::Contact *contact = frame->add_contact();
        scrimmage_proto::State *sp_state = contact->mutable_state();
        scrimmage_proto::Vector3d *sp_pos = sp_state->mutable_position();
        scrimmage_proto::Vector3d *sp_vel = sp_state->mutable_velocity();
        scrimmage_proto::Quaternion *sp_quat = sp_state->mutable_orientation();
        scrimmage_proto::ID *sp_id = contact->mutable_id();

        sp_pos->set_x(pos(0));
        sp_pos->set_y(pos(1));
        sp_pos->set_z(pos(2));

        sp_vel->set_x(vel(0));
        sp_vel->set_y(vel(1));
        sp_vel->set_z(vel(2));

        sp_quat->set_x(quat.x());
        sp_quat->set_y(quat.y());
        sp_quat->set_z(quat.z());
        sp_quat->set_w(quat.w());

        switch (type) {
        case Contact::Type::AIRCRAFT: contact->set_type(scrimmage_proto::AIRCRAFT); break;
        case Contact::Type::QUADROTOR: contact->set_type(scrimmage_proto::QUADROTOR); break;
        case Contact::Type::SPHERE: contact->set_type(scrimmage_proto::SPHERE); break;
        case Contact::Type::MESH: contact->set_type(scrimmage_proto::MESH); break;
        default: contact->set_type(scrimmage_proto::UNKNOWN);
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
    contact.set_id(proto_2_id(proto_contact.id()));

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
