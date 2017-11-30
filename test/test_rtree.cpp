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

#include <iostream>

#include <list>
#include <limits.h>

#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/common/ID.h>
#include <scrimmage/math/State.h>

#include <gtest/gtest.h>

using std::cout;
using std::endl;
namespace sc = scrimmage;

bool is_less_than_dist(std::pair<int,double> i, std::pair<int,double> j)
{
     return (i.second < j.second);
}

bool is_less_than_id(sc::ID i, sc::ID j)
{
     return (i.id() < j.id());
}

void populate_tree_randomly(int num_contacts, double range, bool two_dims,
                            std::list<sc::Contact> &contacts,
                            sc::RTree &rtree,
                            sc::Contact &own) {
     // Setup random number generator
     sc::Random rand;
     rand.seed();

     // Randomly generate an ownship position
     auto rnd = [&]() {return rand.rng_uniform() * range;};
     own.state()->pos()(0) = rnd();
     own.state()->pos()(1) = rnd();
     own.state()->pos()(2) = two_dims ? 0 : rnd();

     contacts.clear();

     Eigen::Vector3d vel(0, 0, 0);
     Eigen::Vector3d ang_vel(0, 0, 0);
     sc::Quaternion quat(0, 0, 0);

     for (int i = 0; i < num_contacts; i++) {
          sc::Contact c;
          c.set_id({i, 0, 0});
          double x = rnd();
          double y = rnd();
          double z = two_dims ? 0 : rnd();
          sc::StatePtr state =
              std::make_shared<sc::State>(Eigen::Vector3d(x, y, z), vel,
                                          ang_vel, quat);
          c.set_state(state);
          c.set_type(sc::Contact::Type::AIRCRAFT);
          contacts.push_back(c);
     }

     // Populate the rtree
     rtree.init(contacts.size());
     for(std::list<sc::Contact>::iterator it = contacts.begin();
         it != contacts.end(); it++) {
          rtree.add(it->state()->pos(), it->id());
     }
}

TEST(rtree_test, nearest_dist) {
    int num_contacts = 10000;
    double range = 10000;
    double circ_range = 1000;

    sc::Contact own;
    sc::RTree rtree;
    std::list<sc::Contact> contacts;
    std::vector<sc::ID> rtree_neighbors;

    populate_tree_randomly(num_contacts, range, true, contacts, rtree, own);
    rtree.neighbors_in_range(own.state()->pos_const(), rtree_neighbors, circ_range);

    auto beg_it = rtree_neighbors.begin();
    auto end_it = rtree_neighbors.end();

    for (sc::Contact &c : contacts) {
        double dist = (c.state()->pos() - own.state()->pos()).norm();
        bool found = std::find(beg_it, end_it, c.id()) != end_it;

        if (dist < circ_range) {
            ASSERT_TRUE(found);
        } else {
            ASSERT_FALSE(found);
        }
    }
}

TEST(rtree_test, nearest_n_neighbors)
{
    // Find nearest neighbor of ownship
    unsigned int num_neighbors = 100;
    int num_contacts = 10000;
    double range = 10000;

    sc::Contact own;
    sc::RTree rtree;
    std::list<sc::Contact> contacts;
    std::vector<sc::ID> rtree_neighbors;

    populate_tree_randomly(num_contacts, range, false, contacts, rtree, own);
    rtree.nearest_n_neighbors(own.state()->pos_const(), rtree_neighbors, num_neighbors);

    ASSERT_EQ(rtree_neighbors.size(), (unsigned int)num_neighbors);

    // Find the nearest neighbors the slow, but sure way:
    // 1. Sort the distance vector
    std::vector<std::pair<int,double> > id_dists;
    for (sc::Contact &c : contacts) {
         double dist = (own.state()->pos()-c.state()->pos()).norm();
         id_dists.push_back(std::make_pair(c.id().id(),dist));
    }
    std::sort(id_dists.begin(), id_dists.end(), is_less_than_dist);

    // 2. Get the first n distances only
    unsigned int n = 0;
    std::vector<sc::ID> slow_neighbors;
    for (std::vector<std::pair<int,double> >::iterator it = id_dists.begin();
         it != id_dists.end() && n < num_neighbors; it++, n++) {
         sc::ID id;
         id.set_id(it->first);
         slow_neighbors.push_back(id);
    }

    // Make sure the two ID lists are the same size
    ASSERT_EQ(rtree_neighbors.size(), slow_neighbors.size());

    // Make sure the two ID lists match:
    // Sort the two ID lists based on their ID
    std::sort(rtree_neighbors.begin(), rtree_neighbors.end(), is_less_than_id);
    std::sort(slow_neighbors.begin(), slow_neighbors.end(), is_less_than_id);

    std::vector<sc::ID>::iterator it1 = rtree_neighbors.begin();
    std::vector<sc::ID>::iterator it2 = slow_neighbors.begin();
    for (; it1 != rtree_neighbors.end() && it2 != slow_neighbors.end();
         it1++, it2++) {
         //cout << it1->id() << " , " << it2->id() << endl;
         ASSERT_EQ(it1->id(), it2->id());
    }
}

TEST(rtree_test, nearest_n_neighbors_minus_self)
{
    // Find nearest neighbor of ownship
    unsigned int num_neighbors = 1;
    int num_contacts = 10;
    double range = 1000;

    sc::Contact own;
    sc::RTree rtree;
    std::list<sc::Contact> contacts;
    std::vector<sc::ID> rtree_neighbors;

    populate_tree_randomly(num_contacts, range, false, contacts, rtree, own);

    sc::Contact &c = contacts.front();

    rtree.nearest_n_neighbors(c.state()->pos_const(), rtree_neighbors, num_neighbors, c.id().id());
    ASSERT_EQ(rtree_neighbors.size(), num_neighbors);

    rtree.nearest_n_neighbors(c.state()->pos_const(), rtree_neighbors, num_neighbors);
    ASSERT_EQ(rtree_neighbors.size(), num_neighbors);
}
