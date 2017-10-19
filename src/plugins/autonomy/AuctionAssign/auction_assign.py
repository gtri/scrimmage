#
# @file
#
# @section LICENSE
#
# Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
#
# This file is part of SCRIMMAGE.
#
#   SCRIMMAGE is free software: you can redistribute it and/or modify it under
#   the terms of the GNU Lesser General Public License as published by the
#   Free Software Foundation, either version 3 of the License, or (at your
#   option) any later version.
#
#   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
#   License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
#
# @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
# @author Eric Squires <eric.squires@gtri.gatech.edu>
# @date 31 July 2017
# @version 0.1.0
# @brief Brief file description.
# @section DESCRIPTION
# A Long description goes here.
#
#

from __future__ import print_function
import scrimmage as sc
import numpy as np
from AuctionMsgs_pb2 import BidAuction, StartAuction


class AuctionAssign(sc.Autonomy):

    def init(self, params):
        self.create_publisher("StartAuction")
        self.create_publisher("BidAuction")
        self.create_subscriber("StartAuction")
        self.create_subscriber("BidAuction")

        self.auction_started = False
        self.auction_in_prog = False
        self.auction_max_time = 5
        self.auction_start_time = -np.inf

        self.max_bid = -np.inf
        self.max_bid_champ = -1

    def step_autonomy(self, t, dt):
        self.action = sc.Action()
        self.action.action_type = sc.Action.ActionType.None
        self.action.target_id = 0

        # if someone started an auction then send in a bid
        for msg_in in self.subs["StartAuction"].msg_list:
            print("{} received StartAuction from {}" \
                  .format(self.id.id, msg_in.sender))
            data = BidAuction(originator_id=self.id.id, bid=np.random.rand())
            msg_out = sc.Message(self.id.id, data.SerializeToString(), data)
            self.pubs["BidAuction"].publish(msg_out)

        if self.auction_started:
            for msg_in in self.subs["BidAuction"].msg_list:

                data = msg_in.data

                print("{} received BidAuction from {}: bid = {:.2f}" \
                      .format(self.id.id, msg_in.sender, data.bid))
                if data.bid > self.max_bid:
                    self.max_bid = data.bid
                    self.max_bid_champ = msg_in.sender

        if not self.auction_started and self.id.id == 1:
            print("{} is sending StartAuction msg".format(self.id.id))
            msg_out = sc.Message(self.id.id)
            self.pubs["StartAuction"].publish(msg_out)
            self.auction_started = True
            self.auction_in_prog = True
            self.auction_start_time = t

        if (self.auction_in_prog and
            t > self.auction_start_time + self.auction_max_time and
            self.id.id == 1):

            self.auction_in_prog = False
            print("="*60)
            print("Auction Complete")
            print("Max Bidder: {} - Bid: {:.2f}" \
                  .format(self.max_bid_champ, self.max_bid))

        return True
