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

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Network.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Message.h>

#include <scrimmage/plugins/autonomy/AuctionAssign/AuctionAssign.h>
#include <scrimmage/msgs/AuctionMsgs.pb.h>

#include <iostream>
#include <limits>
#include <memory>

namespace sc = scrimmage;

using BidMsg = sc::Message<auction::BidAuction>;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::AuctionAssign, AuctionAssign_plugin)

namespace scrimmage {
namespace autonomy {

void AuctionAssign::init(std::map<std::string, std::string> &params) {
    id_ = parent_->id().id();

    desired_state_->vel() << 0, 0, 0;
    desired_state_->quat().set(0, 0, state_->quat().yaw());
    desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    create_publisher("StartAuction");
    create_publisher("BidAuction");

    create_subscriber("StartAuction");
    create_subscriber("BidAuction");

    auction_started_ = false;
    auction_in_prog_ = false;
    auction_max_time_ = 5;
    max_bid_ = -std::numeric_limits<double>::max();
}

bool AuctionAssign::step_autonomy(double t, double dt) {
    // Read the Start Auction inbox
    for (auto &msg : subs_["StartAuction"]->msgs<sc::MessageBase>(true, false)) {
        std::cout << "StartAuction: " << id_
          << " received message from " << msg->sender << std::endl;

        auto msg_bid = std::make_shared<sc::Message<auction::BidAuction>>();
        msg_bid->sender = id_;
        const double bid = parent_->random()->rng_uniform() * 10.0;
        msg_bid->data.set_bid(bid);
        std::cout << " sending back bid of " << bid << std::endl;
        pubs_["BidAuction"]->publish(msg_bid, t, false);
    }

    if (auction_started_) {
        // Read the Bid Auction inbox
        for (auto msg : subs_["BidAuction"]->msgs<auction::BidAuction>(true, false)) {

            std::cout << "BidAuction: " << id_ << " received message from "
                << msg->sender << " bid: " << msg->data.bid() << std::endl;

            if (msg->data.bid() > max_bid_) {
                max_bid_ = msg->data.bid();
                max_bid_champ_ = msg->sender;
            }
        }
    }

    if (!auction_started_ && id_ == 1) {
        sc::MessageBasePtr msg(new sc::MessageBase());
        msg->sender = id_;

        std::cout << "origin: " << msg->sender << std::endl;
        pubs_["StartAuction"]->publish(msg, t, false);

        auction_started_ = true;
        auction_in_prog_ = true;
        auction_start_time_ = t;
        std::cout << "Starting!" << std::endl;
    }

    if (auction_in_prog_ && t > auction_start_time_ + auction_max_time_
        && id_ == 1) {
        std::cout << "======================================" << std::endl;
        std::cout << "Auction Complete" << std::endl;
        std::cout << "Max Bidder: " << max_bid_champ_ << " - Bid=" << max_bid_ << std::endl;
        std::cout << "======================================" << std::endl;
        auction_in_prog_ = false;
    }

    return true;
}
} // namespace autonomy
} // namespace scrimmage
