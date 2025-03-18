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
#include <scrimmage/common/CSV.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/fwd_decl.h>
#include <scrimmage/math/State.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <boost/optional.hpp>
#include <list>

namespace sc = scrimmage;

class ScrimmageEntityStateTest : public testing::Test {
 protected:
    void SetUp() override {
        if (!simcontrol.init(mission, false)) {
            std::cout << "Failed to initialize SimControl." << std::endl;
            FAIL();
        }
        simcontrol.pause(false);
        simcontrol.start();

        test = 1;
    }

    const std::string mission = "straight";
    sc::SimControl simcontrol;
    int test = 0;
};

/*
 * This works because member pointers are bound to actual type of the object the address is taken
 * of, not the named type. So taking the address of the ControllerExposer's state_ name gives us
 * back the address to the Controller state_ name, which we can then dereference and use to access
 * the state_ value of the controller. See
 * https://stackoverflow.com/questions/75538/hidden-features-of-c/1065606#1065606 for an in-depth
 * explantation
 */
struct ControllerExposer : public scrimmage::Controller {
    static sc::StatePtr expose_state(scrimmage::Controller& controller) {
        return controller.*(&ControllerExposer::state_);
    }
};

/*
 * Tests to make sure that when Entities are initialized, both the state_truth_ and state_belief_
 * pointers are coupled (i.e. point to the same State data). Also checks that the controllers and
 * motion model of the entity are using the same data.
 */
TEST_F(ScrimmageEntityStateTest, test_coupled_state) {
    const std::list<sc::EntityPtr>& ents = this->simcontrol.ents();
    ASSERT_NE(ents.size(), 0);

    for (sc::EntityPtr ent : ents) {
        sc::StatePtr state_truth = ent->state_truth();
        const std::shared_ptr<const sc::State> state_belief = ent->state_belief();
        ASSERT_EQ(state_truth, state_belief);

        for (sc::ControllerPtr controller : ent->controllers()) {
            ASSERT_EQ(ControllerExposer::expose_state(*controller), state_belief);
        }

        sc::MotionModelPtr motion_model = ent->motion();
        ASSERT_EQ(motion_model->state(), state_truth);
    }
}

/*
 * Tests to make sure that when Entities are initialized, and then the state_truth_ pointer is
 * decoupled from the state_belief_ pointer, those pointers point to different State Objects. Also
 * checks that the controllers are using the new state_belief_ pointer and the motion model of the
 * entity is using the state_truth_ pointer.
 */
TEST_F(ScrimmageEntityStateTest, test_decoupled_state) {
    const std::list<sc::EntityPtr>& ents = simcontrol.ents();
    ASSERT_NE(ents.size(), 0);

    for (sc::EntityPtr ent : ents) {
        sc::StatePtr state_truth = ent->state_truth();

        // This function should decouple state_truth_ and state_belief_.
        ent->set_state_belief(std::make_shared<sc::State>());

        const std::shared_ptr<const sc::State> state_belief = ent->state_belief();
        ASSERT_NE(state_truth, state_belief);

        for (sc::ControllerPtr controller : ent->controllers()) {
            ASSERT_EQ(ControllerExposer::expose_state(*controller), state_belief);
        }

        sc::MotionModelPtr motion_model = ent->motion();
        ASSERT_EQ(motion_model->state(), state_truth);
    }
}
