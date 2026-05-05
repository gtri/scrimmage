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

#include "scrimmage/unity_bridge/UnityBridge.h"
#include "scrimmage/log/Logger.h"

#include <chrono>

#include <zmq.hpp>

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
    scrimmage_unity::Handshake msg;
    msg.set_protocol_version(PROTOCOL_VERSION);
    msg.set_sim_time(sim_time);
    msg.set_sim_dt(sim_dt_);
    msg.set_origin_lat(origin_lat_);
    msg.set_origin_lon(origin_lon_);
    msg.set_origin_alt(origin_alt_);

    std::string proto_payload;
    if (!msg.SerializeToString(&proto_payload)) {
        LOG_ERROR("UnityBridge: failed to serialize handshake");
        return false;
    }

    LOG_INFO("UnityBridge: sending handshake with periodic retry ("
             << connection_timeout_s_ << "s timeout)");

    using clock = std::chrono::steady_clock;
    using time_point = clock::time_point;

    time_point deadline = clock::now()
                        + std::chrono::duration_cast<clock::duration>(
                            std::chrono::duration<double>(connection_timeout_s_));

    std::unique_lock<std::mutex> lock(ack_mutex_);

    // Send handshake periodically until ack received or timeout
    // This mitigates ZMQ slow joiner problem
    int attempt = 0;
    while (true) {
        // Send handshake
        if (zmq_send("handshake", proto_payload)) {
            attempt++;
            if (attempt == 1) {
                LOG_INFO("UnityBridge: handshake sent (will retry every 500ms until ack)");
            }
        } else if (attempt == 0) {
            LOG_ERROR("UnityBridge: Failed to send handshake!");
            return false;
        }

        // Calculate next retry time (500ms from now)
        time_point next_retry = clock::now() + std::chrono::milliseconds(500);
        time_point wait_until_time = (next_retry < deadline) ? next_retry : deadline;

        // Wait for ack with 500ms timeout per iteration
        bool got_ack = ack_cv_.wait_until(
            lock,
            wait_until_time,
            [this] { return ack_received_; });

        if (got_ack) {
            // Ack received, check status
            if (ack_status_ != scrimmage_unity::HANDSHAKE_OK) {
                LOG_ERROR("UnityBridge: handshake_ack status=" << ack_status_
                          << " message=" << ack_message_);
                return false;
            }

            connected_ = true;
            LOG_INFO("UnityBridge: connected to Unity after " << attempt
                     << " attempt(s) (protocol " << PROTOCOL_VERSION << ")");
            return true;
        }

        // Check if overall timeout reached
        if (clock::now() >= deadline) {
            LOG_ERROR("UnityBridge: timed out waiting for handshake_ack after "
                     << attempt << " attempts");
            return false;
        }

        // Continue loop to send handshake again
    }
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

    LOG_INFO("Sending Entity Create Msg to Unity!");
    scrimmage_unity::EntityCreate msg;
    msg.set_sim_time(sim_time);
    msg.set_id(cfg.id);
    msg.set_sub_swarm_id(cfg.sub_swarm_id);
    msg.set_team_id(cfg.team_id);
    msg.set_name(cfg.name);
    msg.set_prefab_id(cfg.prefab_id);
    msg.set_contact_type(cfg.contact_type);
    msg.set_scale(cfg.scale);
    msg.set_base_roll_deg(cfg.base_roll_deg);
    msg.set_base_pitch_deg(cfg.base_pitch_deg);
    msg.set_base_yaw_deg(cfg.base_yaw_deg);
    msg.set_color_r(cfg.color_r);
    msg.set_color_g(cfg.color_g);
    msg.set_color_b(cfg.color_b);
    msg.set_opacity(cfg.opacity);

    std::string proto_payload;
    if (!msg.SerializeToString(&proto_payload)) {
        LOG_ERROR("UnityBridge: failed to serialize entity_create");
        return false;
    }
    return zmq_send("entity_create", proto_payload);
}

bool UnityBridge::send_entity_destroy(double sim_time, int entity_id) {
    scrimmage_unity::EntityDestroy msg;
    msg.set_sim_time(sim_time);
    msg.set_id(entity_id);

    std::string proto_payload;
    if (!msg.SerializeToString(&proto_payload)) {
        LOG_ERROR("UnityBridge: failed to serialize entity_destroy");
        return false;
    }
    return zmq_send("entity_destroy", proto_payload);
}

// ---------------------------------------------------------------------------
// Per-step state update
// ---------------------------------------------------------------------------

bool UnityBridge::send_state_update(double sim_time,
                                    uint64_t frame_id,
                                    const std::vector<EntityState>& states) {
    scrimmage_unity::StateUpdate msg;
    msg.set_sim_time(sim_time);
    msg.set_frame_id(frame_id);

    for (const auto& s : states) {
        auto* ent = msg.add_entities();
        ent->set_id(s.id);
        ent->set_active(s.active);
        ent->set_pos_x(s.pos_x);
        ent->set_pos_y(s.pos_y);
        ent->set_pos_z(s.pos_z);
        ent->set_qw(s.qw);
        ent->set_qx(s.qx);
        ent->set_qy(s.qy);
        ent->set_qz(s.qz);
        ent->set_vel_x(s.vel_x);
        ent->set_vel_y(s.vel_y);
        ent->set_vel_z(s.vel_z);
        ent->set_ang_vel_x(s.ang_vel_x);
        ent->set_ang_vel_y(s.ang_vel_y);
        ent->set_ang_vel_z(s.ang_vel_z);
    }

    std::string proto_payload;
    if (!msg.SerializeToString(&proto_payload)) {
        LOG_ERROR("UnityBridge: failed to serialize state_update");
        return false;
    }
    return zmq_send("state_update", proto_payload);
}

// ---------------------------------------------------------------------------
// ZMQ send helper
// ---------------------------------------------------------------------------

bool UnityBridge::zmq_send(const std::string& msg_type,
                            const std::string& proto_binary) {
    if (!pub_sock_) return false;
    try {
        zmq::message_t type_frame(msg_type.data(), msg_type.size());
        zmq::message_t proto_frame(proto_binary.data(), proto_binary.size());
        auto r = pub_sock_->send(type_frame,
                                 zmq::send_flags::sndmore |
                                 zmq::send_flags::dontwait);
        if (!r) {
            ++dropped_publish_count_;
            return false;
        }
        pub_sock_->send(proto_frame, zmq::send_flags::dontwait);
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

bool UnityBridge::parse_handshake_ack(const std::string& proto_binary) {
    scrimmage_unity::HandshakeAck ack;
    if (!ack.ParseFromString(proto_binary)) {
        LOG_ERROR("UnityBridge: failed to parse handshake_ack protobuf");
        return false;
    }

    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_status_ = ack.status();
    ack_message_ = ack.message();
    ack_received_ = true;
    ack_cv_.notify_all();
    return true;
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
