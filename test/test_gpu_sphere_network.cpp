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
 * @author Wesley Ford <wford32@gatech.edu>
 * @date 23 April 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/gpu/GPUMapBuffer.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/plugins/network/GPUSphereNetwork/GPUSphereNetworkUtils.h>
#include <numeric>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <set>

#include <CL/opencl.hpp>

namespace sc = scrimmage;
namespace sn = sc::network;
namespace fs = std::filesystem;
using namespace testing;

constexpr double tau = 2*M_PI;

class GPUTestFixture : public Test {
 protected:
    void SetUp() {
        using GPUBuildParamMap = std::map<std::string, sc::GPUPluginBuildParams>;

        fs::path src_file{__FILE__};
        fs::path mission_path = src_file.parent_path() / mission_path_;

        sc::MissionParsePtr mp = std::make_shared<sc::MissionParse>();
        mp->parse(mission_path.string());
        gpu_ = std::make_shared<sc::GPUController>();
        gpu_->init(mp);
        const GPUBuildParamMap& params = gpu_->get_plugin_params();
        EXPECT_EQ(params.count("GPUSphereNetwork"), 1);
        utils_ = std::make_shared<sn::GPUSphereNetworkUtils>(range_, params.at("GPUSphereNetwork"));
    }

 public:
    std::string mission_path_ = "./test_missions/gpu_sphere_network.xml";
    sc::GPUControllerPtr gpu_;
    const double range_ = 10.0;
    std::shared_ptr<sn::GPUSphereNetworkUtils> utils_;
};

TEST_F(GPUTestFixture, TestAllInRange) {
    using EntityIdPair = std::pair<int, int>;
    std::vector<int> num_ents(254, 0);
    std::iota(num_ents.begin(), num_ents.end(), 2);

    //std::vector<int> num_ents{23};

    for (int num_ent : num_ents) {
        std::map<int, sc::StatePtr> states;
        //std::cout << "Testing " << num_ent << " number of entities" << std::endl;

        std::shared_ptr<sn::GPUSphereNetworkUtils> utils = this->utils_;
        double theta = 0;
        double dTheta = tau / static_cast<double>(num_ent);
        double radius = 4.5;
        for (int i = 0; i < num_ent; ++i) {
            sc::StatePtr state = std::make_shared<sc::State>();
            Eigen::Vector3d pos{std::cos(theta), std::sin(theta), 0};
            state->set_pos(radius * pos);
            states[i] = state;
            theta += dTheta;
        }
        std::set<EntityIdPair> within_range = utils->proximity_pairs(states);

        std::size_t expected_size = num_ent * (num_ent - 1) / 2;
        ASSERT_EQ(within_range.size(), expected_size);

        for (int first_ind = 0; first_ind < num_ent - 1; ++first_ind) {
            for (int second_ind = first_ind + 1; second_ind < num_ent; ++second_ind) {
                EntityIdPair expected_id_pair = std::make_pair(first_ind, second_ind);
                ASSERT_TRUE(within_range.count(expected_id_pair) == 1);
            }
        }
    }
}

TEST_F(GPUTestFixture, TestNoneInRange) {
    using EntityIdPair = std::pair<int, int>;
    std::vector<int> num_ents(254, 0);
    std::iota(num_ents.begin(), num_ents.end(), 2);

    for (int num_ent : num_ents) {
        std::map<int, sc::StatePtr> states;
        //std::cout << "Testing " << num_ent << " number of entities" << std::endl;

        std::shared_ptr<sn::GPUSphereNetworkUtils> utils = this->utils_;
        double theta = 0;
        double dTheta = M_PI_2 / static_cast<double>(num_ent);
        double radius = std::numeric_limits<float>::max();
        for (int i = 0; i < num_ent; ++i) {
            sc::StatePtr state = std::make_shared<sc::State>();
            Eigen::Vector3d pos{std::cos(theta), std::sin(theta), 0};
            state->set_pos(radius * pos);
            states[i] = state;
            theta += dTheta;
        }
        std::set<EntityIdPair> within_range = utils->proximity_pairs(states);

        ASSERT_EQ(within_range.size(), 0);
    }
}

TEST_F(GPUTestFixture, TestOneOutOfRange) {
    using EntityIdPair = std::pair<int, int>;
    std::vector<int> num_ents{5, 6, 100, 101, 256};


    for (int num_ent : num_ents) {
        for (int out_of_range_ent = 0; out_of_range_ent < num_ent; ++out_of_range_ent) {
            std::map<int, sc::StatePtr> states;

            std::shared_ptr<sn::GPUSphereNetworkUtils> utils = this->utils_;
            double theta = 0;
            double dTheta = M_PI_2 / static_cast<double>(num_ent);
            double in_range_radius = 4.5;
            double not_in_range_radius = std::numeric_limits<float>::max();
            for (int i = 0; i < num_ent; ++i) {
                sc::StatePtr state = std::make_shared<sc::State>();
                Eigen::Vector3d pos{std::cos(theta), std::sin(theta), 0};
                if (i == out_of_range_ent) {
                    state->set_pos(not_in_range_radius * pos);
                } else {
                    state->set_pos(in_range_radius * pos);
                }
                states[i] = state;
                theta += dTheta;
            }
            std::set<EntityIdPair> within_range_pairs = utils->proximity_pairs(states);

            std::size_t expected_size = ((num_ent - 1) * (num_ent - 2)) / 2;
            ASSERT_EQ(within_range_pairs.size(), expected_size);

            for(EntityIdPair within_range_pair: within_range_pairs) {
                ASSERT_NE(within_range_pair.first, out_of_range_ent);
                ASSERT_NE(within_range_pair.second, out_of_range_ent);
            }
        }
    }
}
