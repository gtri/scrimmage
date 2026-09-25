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

#include "scrimmage/common/Random.h"

#include <chrono>  // NOLINT
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace scrimmage {

Random::Random()
    : seed_(std::chrono::system_clock::now().time_since_epoch().count()),
      gener_(std::make_shared<std::default_random_engine>()),
      rng_normal_(0, 1),
      rng_uniform_(-1, 1) {}

uint32_t Random::get_seed() {
    return seed_;
}

void Random::seed() {
    seed(std::chrono::system_clock::now().time_since_epoch().count());
}

void Random::seed(uint32_t _seed) {
    seed_ = _seed;
    gener_->seed(seed_);
    spawn_gener_.seed(seed_);
}

double Random::rng_uniform() {
    return rng_uniform_(*gener_);
}

double Random::rng_uniform(double low, double high) {
    double pct = (rng_uniform_(*gener_) + 1) / 2;
    return low + (high - low) * pct;
}

double Random::rng_normal() {
    return rng_normal_(*gener_);
}

double Random::rng_normal(double mean, double sigma) {
    return std::normal_distribution<double>(mean, sigma)(*gener_);
}

int Random::rng_uniform_int(int low, int high) {
    return std::uniform_int_distribution<int>(low, high)(*gener_);
}

int Random::rng_discrete_int(std::vector<double>& weights) {
    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    return dist(*gener_);
}

std::shared_ptr<std::normal_distribution<double>> Random::make_rng_normal(
    double mean,
    double sigma) {
    return std::make_shared<std::normal_distribution<double>>(mean, sigma);
}

bool libcxx_spawn_random() {
    static const bool enabled = [] {
        const char* value = std::getenv("SCRIMMAGE_LIBCXX_SPAWN_RANDOM");
        return value != nullptr && std::strcmp(value, "1") == 0;
    }();
    return enabled;
}

SpawnNormal::SpawnNormal(double mean, double sigma)
    : standard_(mean, sigma), mean_(mean), sigma_(sigma) {}

double SpawnNormal::operator()(Random& random) {
    if (!libcxx_spawn_random()) {
        return standard_(*random.gener());
    }
    // libc++ generate_canonical<double, 53> for minstd_rand: two draws.
    auto canonical = [&random]() {
        const double r = 2147483646.0;
        double lo = static_cast<double>(random.spawn_gener()() - 1);
        double hi = static_cast<double>(random.spawn_gener()() - 1);
        return (lo + hi * r) / (r * r);
    };
    double value;
    if (has_cached_) {
        has_cached_ = false;
        value = cached_;
    } else {
        double u, v, s;
        do {
            u = 2.0 * canonical() - 1.0;
            v = 2.0 * canonical() - 1.0;
            s = u * u + v * v;
        } while (s > 1.0 || s == 0.0);
        double f = std::sqrt(-2.0 * std::log(s) / s);
        cached_ = v * f;
        has_cached_ = true;
        value = u * f;
    }
    return value * sigma_ + mean_;
}

}  // namespace scrimmage
