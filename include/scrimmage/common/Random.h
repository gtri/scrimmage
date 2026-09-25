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

#ifndef INCLUDE_SCRIMMAGE_COMMON_RANDOM_H_
#define INCLUDE_SCRIMMAGE_COMMON_RANDOM_H_

#include <memory>
#include <random>
#include <vector>

namespace scrimmage {

class Random {
 public:
    Random();

    uint32_t get_seed();
    void seed();

    void seed(uint32_t _seed);

    double rng_uniform();
    double rng_uniform(double low, double high);
    double rng_normal();
    double rng_normal(double mean, double sigma);
    int rng_uniform_int(int low, int high);  // inclusive of low and high

    std::shared_ptr<std::normal_distribution<double>> make_rng_normal(double mean, double sigma);

    int rng_discrete_int(std::vector<double>& weights);

    std::shared_ptr<std::default_random_engine> gener() { return gener_; }

    /// scrimmage-rs comparison option (branch scrimmage-rs-instrumentation):
    /// when SCRIMMAGE_LIBCXX_SPAWN_RANDOM=1, spawn draws use this dedicated
    /// std::minstd_rand stream, which the C++ standard fully specifies.
    std::minstd_rand& spawn_gener() { return spawn_gener_; }

 protected:
    uint32_t seed_;
    std::shared_ptr<std::default_random_engine> gener_;
    std::minstd_rand spawn_gener_;
    std::normal_distribution<double> rng_normal_;
    std::uniform_real_distribution<double> rng_uniform_;
};

typedef std::shared_ptr<Random> RandomPtr;

/// True when SCRIMMAGE_LIBCXX_SPAWN_RANDOM=1 (read once).
bool libcxx_spawn_random();

/// Normal draws for entity generation. By default this is exactly
/// std::normal_distribution on Random::gener(), unchanged. With
/// SCRIMMAGE_LIBCXX_SPAWN_RANDOM=1 it instead reproduces libc++ (Apple)
/// normal_distribution on Random::spawn_gener(): the Marsaglia polar method
/// returning the first variate and caching the second, which libstdc++ reverses.
/// Only spawn draws move to the dedicated stream, so this matches unmodified
/// macOS C++ only when spawning is the sole consumer of the shared generator.
class SpawnNormal {
 public:
    SpawnNormal(double mean, double sigma);
    double operator()(Random& random);

 private:
    std::normal_distribution<double> standard_;
    double mean_;
    double sigma_;
    bool has_cached_ = false;
    double cached_ = 0;
};

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_COMMON_RANDOM_H_
