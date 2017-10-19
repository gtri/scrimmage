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
#include <gtest/gtest.h>
#include <scrimmage/plugins/sensor/SimpleCamera/SimpleCamera.h>
#include <scrimmage/plugins/sensor/NoisyState/NoisyState.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/Hash.h>

#include <boost/optional.hpp>

namespace sc = scrimmage;

class SensorTest : public ::testing::Test {
 protected:
    virtual void SetUp() {
        sc::ContactMapPtr contacts(new sc::ContactMap());
        (*contacts)[1].set_id(sc::ID(1, 1, 1));
        (*contacts)[1].state()->pos() = Eigen::Vector3d::Zero();
        (*contacts)[1].state()->pos() = Eigen::Vector3d::Zero();
        (*contacts)[1].state()->quat().set(0, 0, 0);

        (*contacts)[2].set_id(sc::ID(2, 1, 1));
        (*contacts)[2].state()->pos() = Eigen::Vector3d(10, 0, 0);
        (*contacts)[2].state()->vel() = Eigen::Vector3d::Zero();
        (*contacts)[2].state()->quat().set(0, 0, 0);

        parent_ = std::make_shared<sc::Entity>();

        parent_->rtree() = std::make_shared<scrimmage::RTree>();
        parent_->rtree()->init(2);
        parent_->rtree()->add(contacts->at(1).state()->pos(), contacts->at(1).id());
        parent_->rtree()->add(contacts->at(2).state()->pos(), contacts->at(2).id());

        parent_->id() = sc::ID(1, 1, 1);
        parent_->contacts() = contacts;
    }

    sc::EntityPtr parent_;
};

TEST_F(SensorTest, simple_camera) {

    std::shared_ptr<SimpleCamera> simple_camera(new SimpleCamera());
    std::map<std::string, std::string> params
        {{"range", "100"}, {"fov_az", "90"},
        {"fov_el", "90"}, {"draw_cone", "false"}};
    simple_camera->init(params);

    simple_camera->set_parent(parent_);
    simple_camera->parent()->contacts() = parent_->contacts();
    auto msg = simple_camera->sense<std::unordered_set<sc::ID>>(0);
    std::unordered_set<sc::ID> &msg_data = (*msg)->data;

    EXPECT_TRUE(msg_data.size() == 1);

    auto &contacts = *parent_->contacts();
    EXPECT_TRUE(msg_data.count(contacts[2].id()) == 1);
    EXPECT_TRUE(msg_data.count(contacts[1].id()) == 0);
}
