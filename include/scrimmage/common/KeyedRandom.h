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
 * @brief Keyed random numbers, for the scrimmage-rs (Ripple) comparison.
 * @section DESCRIPTION
 * With SCRIMMAGE_KEYED_RANDOM=1, entity generation, NoisyState,
 * NoisyContacts, and SphereNetwork draw keyed random numbers, as Ripple does
 * (crates/core/src/common/random.rs): each number is a function of a label
 * saying what it is for, never of how many numbers were drawn before it.
 * Without the option every draw is unchanged.
 *
 * A label is a list of 64-bit parts, absorbed in order with SplitMix64's
 * mixer. The label's hash starts a SplitMix64 sequence: a uniform number is
 * its first output, a normal number its first two (Box-Muller). A name enters
 * a label as its 64-bit FNV-1a hash, a signed integer as its two's-complement
 * bits.
 */

#ifndef INCLUDE_SCRIMMAGE_COMMON_KEYEDRANDOM_H_
#define INCLUDE_SCRIMMAGE_COMMON_KEYEDRANDOM_H_

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <string>

namespace scrimmage {
namespace keyed_random {

/// True when SCRIMMAGE_KEYED_RANDOM=1 (read once).
inline bool enabled() {
    static const bool on = [] {
        const char* value = std::getenv("SCRIMMAGE_KEYED_RANDOM");
        return value != nullptr && std::strcmp(value, "1") == 0;
    }();
    return on;
}

/// SplitMix64's increment, the golden ratio in 64 bits.
constexpr uint64_t kGamma = 0x9e3779b97f4a7c15ULL;

/// SplitMix64's output function.
inline uint64_t mix(uint64_t z) {
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

/// Adds one part to a label's hash.
inline uint64_t absorb(uint64_t hash, uint64_t part) {
    return mix((hash + kGamma) ^ part);
}

/// The hash of a whole label, parts in order.
inline uint64_t label(std::initializer_list<uint64_t> parts) {
    uint64_t hash = 0;
    for (uint64_t part : parts) {
        hash = absorb(hash, part);
    }
    return hash;
}

/// A signed integer as a label part.
inline uint64_t part(int64_t value) {
    return static_cast<uint64_t>(value);
}

/// A name as a label part: its bytes hashed with 64-bit FNV-1a.
inline uint64_t name(const std::string& text) {
    uint64_t hash = 0xcbf29ce484222325ULL;
    for (unsigned char byte : text) {
        hash = (hash ^ byte) * 0x00000100000001b3ULL;
    }
    return hash;
}

/// The nth output (from 1) of the sequence a label starts, strictly inside (0, 1).
inline double output(uint64_t label, uint64_t n) {
    const uint64_t bits = mix(label + kGamma * n);
    return (static_cast<double>(bits >> 12) + 0.5) / 4503599627370496.0;
}

/// The label's uniform number, strictly inside (0, 1).
inline double uniform(uint64_t label) {
    return output(label, 1);
}

/// The label's normal number (Box-Muller, cosine branch).
inline double normal(uint64_t label, double mean, double stddev) {
    const double radius = std::sqrt(-2.0 * std::log(output(label, 1)));
    const double angle_rad = 6.283185307179586 * output(label, 2);
    return mean + stddev * radius * std::cos(angle_rad);
}

/// Where a plugin instance's keys start at a step: the seed, entity ID (0 for
/// a network), the instance's identity, and the step (-1 for entities created
/// before the start time).
inline uint64_t plugin_label(uint32_t seed, int entity_id, const std::string& identity, int64_t step) {
    return absorb(label({seed, part(entity_id), name(identity)}), part(step));
}

/// A plugin's key parts absorbed after `plugin_label`.
inline uint64_t key(uint64_t at_step, std::initializer_list<uint64_t> parts) {
    for (uint64_t p : parts) {
        at_step = absorb(at_step, p);
    }
    return at_step;
}

}  // namespace keyed_random
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_COMMON_KEYEDRANDOM_H_
