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
 *   FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @section DESCRIPTION
 *
 * See Trace.h. Record schema (shared with scrimmage-rs):
 *   {"kind":"delivery","t":..,"network":..,"topic":..,
 *    "from":[entity_id|null,"Plugin"],"to":[entity_id|null,"Plugin"],"ids":[..]}
 *   {"kind":"scheduled_delivery", ...same fields..., "deliver_at":..}
 *   {"kind":"belief","t":..,"id":..,"p":[x,y,z],"v":[..],"q":[w,x,y,z],"w":[..]}
 *   {"kind":"output","t":..,"id":..,"plugin":..,"port":..,"value":..}
 *   {"kind":"publication","t":..,"topic":..,"from":[..],
 *    "states":[{"id":contact_id|null,"p":..,"v":..,"q":..,"w":..,"cov":[row-major]}]}
 * "ids" is present only for lifecycle and collision topics.
 */

#include "scrimmage/common/Trace.h"

#include "scrimmage/autonomy/Autonomy.h"
#include "scrimmage/common/VariableIO.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/entity/EntityPlugin.h"
#include "scrimmage/math/Quaternion.h"
#include "scrimmage/math/State.h"
#include "scrimmage/math/StateWithCovariance.h"
#include "scrimmage/entity/Contact.h"
#include "scrimmage/motion/Controller.h"
#include "scrimmage/msgs/Collision.pb.h"
#include "scrimmage/msgs/Event.pb.h"
#include "scrimmage/pubsub/Message.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace sm = scrimmage_msgs;

namespace scrimmage {
namespace trace {
namespace {

std::ofstream* stream() {
    // Opened once, on first use; nullptr when tracing is disabled. A static
    // object (not a leaked pointer) so the final records flush at exit.
    static const char* path = std::getenv("SCRIMMAGE_TRACE");
    static const bool enabled = path != nullptr && *path != '\0';
    static std::ofstream file = enabled ? std::ofstream(path, std::ios::out | std::ios::trunc)
                                        : std::ofstream();
    return enabled ? &file : nullptr;
}

std::string text(const std::string& value) {
    std::string out = "\"";
    for (char c : value) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

// JSON has no NaN or infinity; both traces write them as strings.
std::string number(double value) {
    if (std::isnan(value)) return "\"NaN\"";
    if (std::isinf(value)) return value > 0 ? "\"Infinity\"" : "\"-Infinity\"";
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return out.str();
}

// The comparator maps parent IDs of non-entity plugins to the Rust model.
std::string endpoint(const EntityPluginPtr& plugin) {
    // SimControl publishes through an unnamed EntityPlugin, whose default name is "Plugin".
    std::string name = plugin->name() == "Plugin" ? "SimControl" : plugin->name();
    auto parent = plugin->parent();
    std::string id = parent == nullptr ? "null" : std::to_string(parent->id().id());
    return "[" + id + "," + text(name) + "]";
}

// Entity IDs carried by lifecycle and collision messages; empty otherwise.
std::vector<int> payload_ids(const std::string& topic, const MessageBasePtr& msg) {
    if (topic == "EntityGenerated") {
        if (auto m = std::dynamic_pointer_cast<Message<sm::EntityGenerated>>(msg)) {
            return {m->data.entity_id()};
        }
    } else if (topic == "EntityRemoved") {
        if (auto m = std::dynamic_pointer_cast<Message<sm::EntityRemoved>>(msg)) {
            return {m->data.entity_id()};
        }
    } else if (topic == "EntityPresentAtEnd") {
        if (auto m = std::dynamic_pointer_cast<Message<sm::EntityPresentAtEnd>>(msg)) {
            return {m->data.entity_id()};
        }
    } else if (topic == "GroundCollision") {
        if (auto m = std::dynamic_pointer_cast<Message<sm::GroundCollision>>(msg)) {
            return {m->data.entity_id()};
        }
    } else if (topic == "TeamCollision") {
        if (auto m = std::dynamic_pointer_cast<Message<sm::TeamCollision>>(msg)) {
            return {m->data.entity_id_1(), m->data.entity_id_2()};
        }
    } else if (topic == "NonTeamCollision") {
        if (auto m = std::dynamic_pointer_cast<Message<sm::NonTeamCollision>>(msg)) {
            return {m->data.entity_id_1(), m->data.entity_id_2()};
        }
    }
    return {};
}

void write_delivery(
    const char* kind,
    double t,
    const std::string& network,
    const std::string& topic,
    const EntityPluginPtr& from,
    const EntityPluginPtr& to,
    const MessageBasePtr& msg,
    bool scheduled) {
    std::ofstream* out = stream();
    if (out == nullptr) return;
    *out << "{\"kind\":\"" << kind << "\",\"t\":" << number(t)
         << ",\"network\":" << text(network) << ",\"topic\":" << text(topic)
         << ",\"from\":" << endpoint(from) << ",\"to\":" << endpoint(to);
    if (scheduled) {
        *out << ",\"deliver_at\":" << number(msg->time);
    }
    auto ids = payload_ids(topic, msg);
    if (!ids.empty()) {
        *out << ",\"ids\":[";
        for (size_t i = 0; i < ids.size(); ++i) {
            *out << (i ? "," : "") << ids[i];
        }
        *out << "]";
    }
    *out << "}\n";
    out->flush();
}

std::string vector3(const Eigen::Vector3d& v) {
    return "[" + number(v(0)) + "," + number(v(1)) + "," + number(v(2)) + "]";
}

double current_time = 0;

std::string state_json(const std::string& id, StateWithCovariance& state) {
    const Quaternion& q = state.quat();
    std::string out = "{\"id\":" + id + ",\"p\":" + vector3(state.pos())
                      + ",\"v\":" + vector3(state.vel()) + ",\"q\":[" + number(q.w()) + ","
                      + number(q.x()) + "," + number(q.y()) + "," + number(q.z())
                      + "],\"w\":" + vector3(state.ang_vel()) + ",\"cov\":[";
    const Eigen::MatrixXd& cov = state.covariance();
    for (int row = 0; row < cov.rows(); ++row) {
        for (int col = 0; col < cov.cols(); ++col) {
            out += (row || col ? "," : "") + number(cov(row, col));
        }
    }
    return out + "]}";
}

void write_outputs(double t, int id, const EntityPluginPtr& plugin) {
    std::ofstream* out = stream();
    VariableIO& vars = plugin->vars();
    for (auto& kv : vars.output_variable_index()) {
        *out << "{\"kind\":\"output\",\"t\":" << number(t) << ",\"id\":" << id
             << ",\"plugin\":" << text(plugin->name()) << ",\"port\":" << text(kv.first)
             << ",\"value\":" << number(vars.output(kv.second)) << "}\n";
    }
}

}  // namespace

bool enabled() {
    return stream() != nullptr;
}

void set_time(double t) {
    current_time = t;
}

void publication(const std::string& topic, const EntityPluginPtr& from, const MessageBasePtr& msg) {
    std::ofstream* out = stream();
    if (out == nullptr) return;
    std::vector<std::string> states;
    if (auto m = std::dynamic_pointer_cast<Message<StateWithCovariance>>(msg)) {
        states.push_back(state_json("null", m->data));
    } else if (auto m = std::dynamic_pointer_cast<Message<ContactMap>>(msg)) {
        std::map<int, std::string> sorted;  // unordered_map order is not meaningful
        for (auto& kv : m->data) {
            auto state = std::dynamic_pointer_cast<StateWithCovariance>(kv.second.state());
            if (state == nullptr) return;
            sorted[kv.first] = state_json(std::to_string(kv.first), *state);
        }
        for (auto& kv : sorted) states.push_back(kv.second);
    } else {
        return;
    }
    *out << "{\"kind\":\"publication\",\"t\":" << number(current_time)
         << ",\"topic\":" << text(topic) << ",\"from\":" << endpoint(from) << ",\"states\":[";
    for (size_t i = 0; i < states.size(); ++i) *out << (i ? "," : "") << states[i];
    *out << "]}\n";
    out->flush();
}

void delivery(
    double t,
    const std::string& network,
    const std::string& topic,
    const EntityPluginPtr& from,
    const EntityPluginPtr& to,
    const MessageBasePtr& msg) {
    write_delivery("delivery", t, network, topic, from, to, msg, false);
}

void scheduled_delivery(
    double t,
    const std::string& network,
    const std::string& topic,
    const EntityPluginPtr& from,
    const EntityPluginPtr& to,
    const MessageBasePtr& msg) {
    write_delivery("scheduled_delivery", t, network, topic, from, to, msg, true);
}

void beliefs(double t, const std::list<EntityPtr>& ents) {
    std::ofstream* out = stream();
    if (out == nullptr) return;
    for (const EntityPtr& ent : ents) {
        auto state = ent->state_belief();
        const Quaternion& q = state->quat();
        *out << "{\"kind\":\"belief\",\"t\":" << number(t) << ",\"id\":" << ent->id().id()
             << ",\"p\":" << vector3(state->pos()) << ",\"v\":" << vector3(state->vel())
             << ",\"q\":[" << number(q.w()) << "," << number(q.x()) << "," << number(q.y())
             << "," << number(q.z()) << "],\"w\":" << vector3(state->ang_vel()) << "}\n";
    }
    out->flush();
}

void outputs(double t, const std::list<EntityPtr>& ents) {
    if (stream() == nullptr) return;
    for (const EntityPtr& ent : ents) {
        for (auto& autonomy : ent->autonomies()) {
            write_outputs(t, ent->id().id(), autonomy);
        }
        for (auto& controller : ent->controllers()) {
            write_outputs(t, ent->id().id(), controller);
        }
    }
    stream()->flush();
}

}  // namespace trace
}  // namespace scrimmage
