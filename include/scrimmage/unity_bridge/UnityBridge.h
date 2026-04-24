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
 * @brief ZeroMQ bridge between SCRIMMAGE and a Unity visualization client.
 *
 * @section DESCRIPTION
 *
 * Implements the SCRIMMAGE-Unity ICD revision 0.3 (initial testing).
 *
 * Five message types are exchanged:
 *   handshake      SCRIMMAGE → Unity  (once, at sim start)
 *   handshake_ack  Unity → SCRIMMAGE  (once, in reply)
 *   entity_create  SCRIMMAGE → Unity  (once per entity)
 *   entity_destroy SCRIMMAGE → Unity  (when entity is removed)
 *   state_update   SCRIMMAGE → Unity  (every simulation step)
 *
 * ZMQ topology:
 *   SCRIMMAGE binds PUB on pub_addr_ (default tcp://ip:10250)
 *   SCRIMMAGE binds SUB on sub_addr_ (default tcp://ip:10251)
 *   Unity connects SUB to port 10250 and connects PUB to port 10251.
 *
 * Wire format: two-frame ZMQ multi-part message.
 *   Frame 0: message type string (UTF-8, used as ZMQ subscription filter)
 *   Frame 1: JSON payload (UTF-8, nlohmann/json)
 *
 * See: docs/unity_bridge/ICD.md
 */

#ifndef INCLUDE_SCRIMMAGE_UNITY_BRIDGE_UNITYBRIDGE_H_
#define INCLUDE_SCRIMMAGE_UNITY_BRIDGE_UNITYBRIDGE_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <scrimmage/entity/Contact.h>
#include <scrimmage/proto/Visual.pb.h>

// Forward-declare ZMQ types to avoid including zmq.hpp in user headers.
namespace zmq {

class context_t;
class socket_t;

}  // namespace zmq

namespace scrimmage {
namespace unity_bridge {

// Current ICD protocol version sent in the handshake message.
static constexpr const char* PROTOCOL_VERSION = "1.0";

// ---------------------------------------------------------------------------
// EntityConfig — data needed to spawn one entity in Unity
//
// Populated from the SCRIMMAGE Contact, ContactVisual, and mission XML
// <unity_visual> element. Passed to send_entity_create().
// ---------------------------------------------------------------------------

struct EntityConfig {
    // Identity — mirrors Contact.id (ID proto)
    int id = 0;
    int sub_swarm_id = 0;
    int team_id = 0;
    std::string name;

    // Visual — prefab_id is the Unity prefab selector (see ICD Section 3.3)
    std::string prefab_id;     // e.g. "fw_zephyr", "rw_quadrotor"
    std::string contact_type;  // lowercase ContactType fallback: "aircraft", etc.

    // Scale and orientation correction, from ContactVisual.scale / .rotate
    double scale = 1.0;
    double base_roll_deg = 0.0;
    double base_pitch_deg = 0.0;
    double base_yaw_deg = 0.0;

    // Color, from ContactVisual.color / .opacity
    int color_r = 128;
    int color_g = 128;
    int color_b = 128;
    double opacity = 1.0;
};

// ---------------------------------------------------------------------------
// EntityState — per-entity state published every simulation step
//
// Populated from Contact.state (State proto) at each sim step.
// ---------------------------------------------------------------------------

struct EntityState {
    int id = 0;
    bool active = true;

    // Position in ENU local frame (meters)
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;

    // Orientation: unit quaternion [qw, qx, qy, qz], ENU world→body
    // Matches SCRIMMAGE's Quaternion proto field order (.w, .x, .y, .z)
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

    // Linear velocity, ENU frame (m/s)
    double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;

    // Angular velocity, body frame (rad/s)
    double ang_vel_x = 0.0, ang_vel_y = 0.0, ang_vel_z = 0.0;
};

// ---------------------------------------------------------------------------
// UnityBridge
// ---------------------------------------------------------------------------

class UnityBridge {
 public:
    UnityBridge();
    ~UnityBridge();

    // -----------------------------------------------------------------------
    // Configuration — set before calling connect()
    // -----------------------------------------------------------------------

    void set_pub_address(const std::string& addr) { pub_addr_ = addr; }
    void set_sub_address(const std::string& addr) { sub_addr_ = addr; }
    void set_connection_timeout_s(double t) { connection_timeout_s_ = t; }

    // Geographic origin — sent in the handshake message
    void set_origin(double lat, double lon, double alt) {
        origin_lat_ = lat;
        origin_lon_ = lon;
        origin_alt_ = alt;
    }

    // Simulation time step — sent in the handshake message
    void set_sim_dt(double dt) { sim_dt_ = dt; }

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /// Bind ZMQ sockets. Returns immediately — does not wait for Unity.
    /// Call send_handshake() after this to begin the connection sequence.
    bool bind();

    /// Send the handshake message and block until handshake_ack is received
    /// (or connection_timeout_s_ elapses). Returns true on successful ack.
    bool send_handshake(double sim_time = 0.0);

    /// Close sockets and join the receive thread. Safe to call at any time.
    void disconnect();

    bool is_connected() const { return connected_; }

    // -----------------------------------------------------------------------
    // Entity management
    // -----------------------------------------------------------------------

    /// Send an entity_create message to Unity. Call after send_handshake()
    /// returns true, once per entity at spawn time.
    bool send_entity_create(double sim_time, const EntityConfig& cfg);

    /// Send an entity_destroy message to Unity. Call when an entity is
    /// permanently removed from the simulation.
    bool send_entity_destroy(double sim_time, int entity_id);

    // -----------------------------------------------------------------------
    // Per-step state update
    // -----------------------------------------------------------------------

    /// Publish a state_update message containing the current pose and
    /// velocity of all changed entities. Non-blocking; messages may be
    /// dropped at the ZMQ HWM if Unity is not consuming fast enough.
    ///
    /// @param sim_time   Current simulation time (seconds from start).
    /// @param frame_id   Monotonically increasing step counter.
    /// @param states     States of all entities to include in this update.
    bool send_state_update(
        double sim_time,
        uint64_t frame_id,
        const std::vector<EntityState>& states);

    // -----------------------------------------------------------------------
    // Statistics
    // -----------------------------------------------------------------------

    uint64_t dropped_publish_count() const { return dropped_publish_count_; }

 private:
    // -----------------------------------------------------------------------
    // ZMQ
    // -----------------------------------------------------------------------

    std::unique_ptr<zmq::context_t> ctx_;
    std::unique_ptr<zmq::socket_t> pub_sock_;
    std::unique_ptr<zmq::socket_t> sub_sock_;

    std::string pub_addr_ = "tcp://*:10250";
    std::string sub_addr_ = "tcp://*:10251";

    /// Send a two-frame ZMQ message: [type_string][json_string].
    /// Returns false if the send fails (e.g. HWM reached, socket closed).
    bool zmq_send(const std::string& msg_type, const std::string& json);

    // -----------------------------------------------------------------------
    // Receive thread — listens for handshake_ack
    // -----------------------------------------------------------------------

    std::thread recv_thread_;
    std::atomic<bool> recv_running_{false};
    void recv_loop();

    bool parse_handshake_ack(const std::string& json);

    // Signalled by recv_loop when a valid handshake_ack arrives.
    std::mutex ack_mutex_;
    std::condition_variable ack_cv_;
    bool ack_received_ = false;
    std::string ack_status_;  // "ok" | "version_mismatch" | "error"
    std::string ack_message_;

    // -----------------------------------------------------------------------
    // Configuration state
    // -----------------------------------------------------------------------

    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_alt_ = 0.0;
    double sim_dt_ = 0.1;
    double connection_timeout_s_ = 10.0;

    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> dropped_publish_count_{0};
};

using UnityBridgePtr = std::shared_ptr<UnityBridge>;

// ---------------------------------------------------------------------------
// Helper: build EntityConfig from SCRIMMAGE's existing message types
// ---------------------------------------------------------------------------

/// Populate an EntityConfig from a SCRIMMAGE ContactVisual message plus
/// the entity's team/swarm IDs (from Contact.id) and contact type string.
/// Call this when building the entity_create payload.
EntityConfig entity_config_from_contact_visual(
    const scrimmage_proto::ContactVisual& cv,
    int team_id,
    int sub_swarm_id,
    const std::string& contact_type_str,
    const std::string& prefab_id_override = "");

}  // namespace unity_bridge
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_UNITY_BRIDGE_UNITYBRIDGE_H_
