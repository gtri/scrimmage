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
 */

#include <gtest/gtest.h>

#include <memory>
#include <unordered_set>
#include <vector>

#include "scrimmage/common/ID.h"
#include "scrimmage/common/Random.h"
#include "scrimmage/common/Shape.h"
#include "scrimmage/common/Time.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/entity/EntityPlugin.h"

namespace sc = scrimmage;

namespace {

std::shared_ptr<sc::EntityPlugin> make_draw_plugin(
    const sc::EntityPtr& entity,
    const std::shared_ptr<const sc::Time>& time) {
    auto plugin = std::make_shared<sc::EntityPlugin>();
    plugin->set_parent(entity);
    plugin->set_time(time);
    plugin->set_name("DrawShapeTest");
    return plugin;
}

std::vector<double> draw_uniform_sequence(const sc::RandomPtr& random, int count) {
    std::vector<double> values;
    values.reserve(count);
    for (int i = 0; i < count; ++i) {
        values.push_back(random->rng_uniform());
    }
    return values;
}

}  // namespace

TEST(test_entity_plugin_draw_shape, draw_shape_does_not_consume_master_rng) {
    constexpr uint32_t seed = 649;
    constexpr int warmup_draws = 4;
    constexpr int shape_draws = 12;
    constexpr int post_draws = 8;

    auto random_with_shapes = std::make_shared<sc::Random>();
    random_with_shapes->seed(seed);
    auto random_baseline = std::make_shared<sc::Random>();
    random_baseline->seed(seed);

    const std::vector<double> warmup_with =
        draw_uniform_sequence(random_with_shapes, warmup_draws);
    const std::vector<double> warmup_baseline =
        draw_uniform_sequence(random_baseline, warmup_draws);
    ASSERT_EQ(warmup_with, warmup_baseline);

    auto entity = std::make_shared<sc::Entity>();
    sc::ID id;
    id.set_id(42);
    entity->set_id(id);
    entity->set_random(random_with_shapes);

    auto time = std::make_shared<sc::Time>();
    time->set_t(3.5);

    auto plugin = make_draw_plugin(entity, time);
    std::unordered_set<std::size_t> shape_hashes;
    for (int i = 0; i < shape_draws; ++i) {
        auto shape = sc::shape::make_sphere();
        plugin->draw_shape(shape);
        ASSERT_TRUE(shape->hash_set());
        shape_hashes.insert(shape->hash());
    }
    EXPECT_EQ(shape_hashes.size(), static_cast<std::size_t>(shape_draws));

    const std::vector<double> post_with =
        draw_uniform_sequence(random_with_shapes, post_draws);
    const std::vector<double> post_baseline =
        draw_uniform_sequence(random_baseline, post_draws);

    EXPECT_EQ(post_with, post_baseline);
}
