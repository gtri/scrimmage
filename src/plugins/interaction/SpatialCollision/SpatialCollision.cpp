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
 * @author Ethan M Boos <ethan.m.boos@gtri.gatech.edu>
 * @date 01 May 2026
 * @version 0.1.0
 * @brief Drop-in O(n) SimpleCollision replacement.
 * @section DESCRIPTION
 * Uses a sparse spatial hash broad phase instead of SimpleCollision's
 * O(n^2) all-pairs scan, giving near-linear behavior for fixed collision
 * radius and bounded local density.
 *
 */

#include "scrimmage/plugins/interaction/SpatialCollision/SpatialCollision.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "scrimmage/autonomy/Autonomy.h"
#include "scrimmage/common/RTree.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/math/State.h"
#include "scrimmage/msgs/Collision.pb.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"
#include "scrimmage/pubsub/Message.h"
#include "scrimmage/pubsub/Publisher.h"

namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(
    scrimmage::EntityInteraction,
    scrimmage::interaction::SpatialCollision,
    SpatialCollision_plugin)

namespace {

// This is a sparse spatial hash, not a preallocated world grid. We only create
// entries for occupied collision cells, so mission size is bounded by numeric
// range and memory, not by a fixed map extent.
//
// Each axis index is floor(position / collision_range_) stored in int64_t.
// That gives a theoretical per-axis bound of about:
//   max_abs_position ~= collision_range_ * 9.22e18
// before the cell index would overflow.
//
// Example: with a 2 m collision range, that is about 1.8e19 m per axis.
// In practice, entity positions are doubles, so precision becomes the earlier
// limit for extremely large worlds, but any realistic SCRIMMAGE mission is far
// below that range.
struct CellIndex {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const CellIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex& cell) const {
        std::size_t seed = std::hash<std::int64_t>{}(cell.x);
        seed ^= std::hash<std::int64_t>{}(cell.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<std::int64_t>{}(cell.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

CellIndex to_cell_index(const Eigen::Vector3d& pos, double cell_size) {
    return CellIndex {
        static_cast<std::int64_t>(std::floor(pos(0) / cell_size)),
        static_cast<std::int64_t>(std::floor(pos(1) / cell_size)),
        static_cast<std::int64_t>(std::floor(pos(2) / cell_size))};
}

}  // namespace

namespace scrimmage {
namespace interaction {

bool SpatialCollision::init(
    std::map<std::string, std::string>& mission_params,
    std::map<std::string, std::string>& plugin_params) {
    collision_range_ = get("collision_range", plugin_params, 0.0);
    startup_collision_range_ = get("startup_collision_range", plugin_params, collision_range_);
    startup_collisions_only_ = get("startup_collisions_only", plugin_params, false);
    enable_team_collisions_ = get<bool>("enable_team_collisions", plugin_params, true);
    enable_non_team_collisions_ = get<bool>("enable_non_team_collisions", plugin_params, true);
    init_alt_deconflict_ = get<bool>("init_alt_deconflict", plugin_params, false);

    team_collision_pub_ = advertise("GlobalNetwork", "TeamCollision");
    non_team_collision_pub_ = advertise("GlobalNetwork", "NonTeamCollision");

    return true;
}

bool SpatialCollision::step_entity_interaction(std::list<EntityPtr>& ents, double t, double dt) {
    if (startup_collisions_only_ || collision_range_ <= 0.0) {
        return true;
    }

    // Reserve around one bucket per live entity. The map stays sparse even for
    // huge worlds because only occupied cells are stored.
    std::unordered_map<CellIndex, std::vector<EntityPtr>, CellIndexHash> spatial_hash;
    spatial_hash.reserve(ents.size());

    for (EntityPtr ent : ents) {
        if (!ent->is_alive()) {
            continue;
        }

        const Eigen::Vector3d pos = ent->state_truth()->pos();
        const CellIndex cell = to_cell_index(pos, collision_range_);

        for (int dx = -1; dx <= 1 && ent->is_alive(); ++dx) {
            for (int dy = -1; dy <= 1 && ent->is_alive(); ++dy) {
                for (int dz = -1; dz <= 1 && ent->is_alive(); ++dz) {
                    auto it = spatial_hash.find(CellIndex {cell.x + dx, cell.y + dy, cell.z + dz});
                    if (it == spatial_hash.end()) {
                        continue;
                    }

                    for (EntityPtr candidate : it->second) {
                        if (!ent->is_alive()) {
                            break;
                        }
                        if (!candidate->is_alive()) {
                            continue;
                        }

                        const double dist = (pos - candidate->state_truth()->pos()).norm();
                        if (dist < collision_range_) {
                            handle_collision(ent, candidate);
                        }
                    }
                }
            }
        }

        if (ent->is_alive()) {
            spatial_hash[cell].push_back(ent);
        }
    }

    return true;
}

bool SpatialCollision::collision_exists(std::list<EntityPtr>& ents, Eigen::Vector3d& p) {
    if (ents.empty()) {
        return false;
    }

    if (init_alt_deconflict_) {
        for (EntityPtr ent : ents) {
            if (std::abs(p(2) - ent->state_truth()->pos()(2)) <= startup_collision_range_) {
                return true;
            }
        }
        return false;
    }

    if (ents.front()->autonomies().empty()) {
        return false;
    }

    std::vector<ID> neighbors;
    RTreePtr rtree = ents.front()->autonomies().front()->rtree();
    rtree->neighbors_in_range(p, neighbors, startup_collision_range_);
    return !neighbors.empty();
}

bool SpatialCollision::handle_collision(EntityPtr& ent1, EntityPtr& ent2) {
    if (ent1->id().team_id() == ent2->id().team_id()) {
        if (!enable_team_collisions_) {
            return false;
        }

        ent1->collision();
        ent2->collision();

        auto msg = std::make_shared<Message<sm::TeamCollision>>();
        msg->data.set_entity_id_1(ent1->id().id());
        msg->data.set_entity_id_2(ent2->id().id());
        team_collision_pub_->publish(msg);
        return true;
    }

    if (!enable_non_team_collisions_) {
        return false;
    }

    ent1->collision();
    ent2->collision();

    auto msg = std::make_shared<Message<sm::NonTeamCollision>>();
    msg->data.set_entity_id_1(ent1->id().id());
    msg->data.set_entity_id_2(ent2->id().id());
    non_team_collision_pub_->publish(msg);
    return true;
}

}  // namespace interaction
}  // namespace scrimmage