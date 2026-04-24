/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2024 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it
 *   under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or (at
 *   your option) any later version.
 *
 * @brief SCRIMMAGE-Unity ZeroMQ bridge implementation.
 */

#include <scrimmage/unity_bridge/UnityBridge.h>
#include <scrimmage/log/Logger.h>

#include <chrono>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

using json = nlohmann::json;
using namespace scrimmage::unity_bridge;

static constexpr int ZMQ_HWM = 6;  // High-water mark for both sockets

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

UnityBridge::UnityBridge()
    : ctx_(std::make_unique<zmq::context_t>(1)) {}

UnityBridge::~UnityBridge() {
    disconnect();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool UnityBridge::bind() {
    pub_sock_ = std::make_unique<zmq::socket_t>(*ctx_, ZMQ_PUB);
    pub_sock_->set(zmq::sockopt::sndhwm, ZMQ_HWM);
    try {
        pub_sock_->bind(pub_addr_);
    } catch (const zmq::error_t& e) {
        LOG_ERROR("UnityBridge: cannot bind PUB to " << pub_addr_
                  << ": " << e.what());
        return false;
    }

    sub_sock_ = std::make_unique<zmq::socket_t>(*ctx_, ZMQ_SUB);
    sub_sock_->set(zmq::sockopt::rcvhwm, ZMQ_HWM);
    sub_sock_->set(zmq::sockopt::subscribe, "");
    try {
        sub_sock_->bind(sub_addr_);
    } catch (const zmq::error_t& e) {
        LOG_ERROR("UnityBridge: cannot bind SUB to " << sub_addr_
                  << ": " << e.what());
        return false;
    }

    recv_running_ = true;
    recv_thread_ = std::thread(&UnityBridge::recv_loop, this);
    return true;
}

bool UnityBridge::send_handshake(double sim_time) {
    json j;
    j["msg_type"]         = "handshake";
    j["sim_time"]         = sim_time;
    j["protocol_version"] = PROTOCOL_VERSION;
    j["sim_dt"]           = sim_dt_;
    j["origin_lat"]       = origin_lat_;
    j["origin_lon"]       = origin_lon_;
    j["origin_alt"]       = origin_alt_;

    if (!zmq_send("handshake", j.dump())) {
        return false;
    }

    LOG_INFO("UnityBridge: handshake sent, waiting for ack ("
             << connection_timeout_s_ << "s timeout)");

    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::duration<double>(connection_timeout_s_);

    std::unique_lock<std::mutex> lock(ack_mutex_);
    bool got_ack = ack_cv_.wait_until(lock, deadline,
                                      [this] { return ack_received_; });

    if (!got_ack) {
        LOG_ERROR("UnityBridge: timed out waiting for handshake_ack");
        return false;
    }

    if (ack_status_ != "ok") {
        LOG_ERROR("UnityBridge: handshake_ack status=" << ack_status_
                  << " message=" << ack_message_);
        return false;
    }

    connected_ = true;
    LOG_INFO("UnityBridge: connected to Unity (protocol " << ack_status_ << ")");
    return true;
}

void UnityBridge::disconnect() {
    recv_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    if (sub_sock_) { sub_sock_->close(); sub_sock_.reset(); }
    if (pub_sock_) { pub_sock_->close(); pub_sock_.reset(); }
    connected_ = false;
}

// ---------------------------------------------------------------------------
// Entity management
// ---------------------------------------------------------------------------

bool UnityBridge::send_entity_create(double sim_time, const EntityConfig& cfg) {
    json j;
    j["msg_type"]      = "entity_create";
    j["sim_time"]      = sim_time;
    j["id"]            = cfg.id;
    j["sub_swarm_id"]  = cfg.sub_swarm_id;
    j["team_id"]       = cfg.team_id;
    j["name"]          = cfg.name;
    j["prefab_id"]     = cfg.prefab_id;
    j["contact_type"]  = cfg.contact_type;
    j["scale"]         = cfg.scale;
    j["base_roll_deg"]  = cfg.base_roll_deg;
    j["base_pitch_deg"] = cfg.base_pitch_deg;
    j["base_yaw_deg"]   = cfg.base_yaw_deg;
    j["color"]         = {cfg.color_r, cfg.color_g, cfg.color_b};
    j["opacity"]       = cfg.opacity;
    return zmq_send("entity_create", j.dump());
}

bool UnityBridge::send_entity_destroy(double sim_time, int entity_id) {
    json j;
    j["msg_type"] = "entity_destroy";
    j["sim_time"] = sim_time;
    j["id"]       = entity_id;
    return zmq_send("entity_destroy", j.dump());
}

// ---------------------------------------------------------------------------
// Per-step state update
// ---------------------------------------------------------------------------

bool UnityBridge::send_state_update(double sim_time,
                                    uint64_t frame_id,
                                    const std::vector<EntityState>& states) {
    json j;
    j["msg_type"] = "state_update";
    j["sim_time"] = sim_time;
    j["frame_id"] = frame_id;

    json ents = json::array();
    for (const auto& s : states) {
        json e;
        e["id"]     = s.id;
        e["active"] = s.active;
        e["position"]    = {s.pos_x, s.pos_y, s.pos_z};
        e["orientation"] = {s.qw, s.qx, s.qy, s.qz};
        e["linear_velocity"]  = {s.vel_x, s.vel_y, s.vel_z};
        e["angular_velocity"] = {s.ang_vel_x, s.ang_vel_y, s.ang_vel_z};
        ents.push_back(e);
    }
    j["entities"] = ents;

    return zmq_send("state_update", j.dump());
}

// ---------------------------------------------------------------------------
// ZMQ send helper
// ---------------------------------------------------------------------------

bool UnityBridge::zmq_send(const std::string& msg_type,
                            const std::string& json_str) {
    if (!pub_sock_) return false;
    try {
        zmq::message_t type_frame(msg_type.data(), msg_type.size());
        zmq::message_t json_frame(json_str.data(), json_str.size());
        auto r = pub_sock_->send(type_frame,
                                 zmq::send_flags::sndmore |
                                 zmq::send_flags::dontwait);
        if (!r) {
            ++dropped_publish_count_;
            return false;
        }
        pub_sock_->send(json_frame, zmq::send_flags::dontwait);
        return true;
    } catch (const zmq::error_t& e) {
        if (e.num() == EAGAIN) {
            ++dropped_publish_count_;
        } else {
            LOG_ERROR("UnityBridge: zmq_send error: " << e.what());
        }
        return false;
    }
}

// ---------------------------------------------------------------------------
// Receive thread — only processes handshake_ack in this revision
// ---------------------------------------------------------------------------

void UnityBridge::recv_loop() {
    while (recv_running_) {
        zmq::pollitem_t items[] = {
            { static_cast<void*>(*sub_sock_), 0, ZMQ_POLLIN, 0 }
        };
        zmq::poll(items, 1, std::chrono::milliseconds(50));
        if (!(items[0].revents & ZMQ_POLLIN)) continue;

        // Collect both frames
        std::string type_str, json_str;
        bool first = true, more = true;
        while (more) {
            zmq::message_t frame;
            auto r = sub_sock_->recv(frame, zmq::recv_flags::dontwait);
            if (!r) break;
            more = frame.more();
            if (first) {
                type_str.assign(static_cast<const char*>(frame.data()),
                                frame.size());
                first = false;
            } else {
                json_str.assign(static_cast<const char*>(frame.data()),
                                frame.size());
            }
        }

        if (type_str == "handshake_ack" && !json_str.empty()) {
            parse_handshake_ack(json_str);
        }
        // Additional message types (sensor_response, gui_command, etc.)
        // will be dispatched here in future revisions.
    }
}

bool UnityBridge::parse_handshake_ack(const std::string& json_str) {
    try {
        auto j = json::parse(json_str);
        std::lock_guard<std::mutex> lock(ack_mutex_);
        ack_status_  = j.value("status", std::string("error"));
        ack_message_ = j.value("message", std::string{});
        ack_received_ = true;
        ack_cv_.notify_all();
        return true;
    } catch (const json::exception& e) {
        LOG_ERROR("UnityBridge: failed to parse handshake_ack: " << e.what());
        return false;
    }
}

// ---------------------------------------------------------------------------
// Helper: build EntityConfig from SCRIMMAGE's ContactVisual
// ---------------------------------------------------------------------------

EntityConfig scrimmage::unity_bridge::entity_config_from_contact_visual(
    const scrimmage_proto::ContactVisual& cv,
    int team_id,
    int sub_swarm_id,
    const std::string& contact_type_str,
    const std::string& prefab_id_override) {

    EntityConfig cfg;
    cfg.id           = cv.id();
    cfg.team_id      = team_id;
    cfg.sub_swarm_id = sub_swarm_id;
    cfg.name         = cv.name();
    cfg.contact_type = contact_type_str;
    cfg.scale        = cv.scale() > 0.0 ? cv.scale() : 1.0;
    cfg.opacity      = cv.opacity() > 0.0 ? cv.opacity() : 1.0;

    if (cv.has_color()) {
        cfg.color_r = cv.color().r();
        cfg.color_g = cv.color().g();
        cfg.color_b = cv.color().b();
    }

    // ContactVisual.rotate is a packed repeated double [roll, pitch, yaw]
    if (cv.rotate_size() >= 3) {
        cfg.base_roll_deg  = cv.rotate(0);
        cfg.base_pitch_deg = cv.rotate(1);
        cfg.base_yaw_deg   = cv.rotate(2);
    }

    // prefab_id_override comes from <unity_visual prefab_id="..."/> in XML.
    // Fall back to contact_type-derived default if not set.
    if (!prefab_id_override.empty()) {
        cfg.prefab_id = prefab_id_override;
    } else if (contact_type_str == "aircraft") {
        cfg.prefab_id = "fw_generic";
    } else if (contact_type_str == "quadrotor") {
        cfg.prefab_id = "rw_quadrotor";
    } else if (contact_type_str == "sphere") {
        cfg.prefab_id = "sphere";
    } else {
        cfg.prefab_id = "unknown";
    }

    return cfg;
}
