# ---------------------------------------------------------------------------
# @section LICENSE
#
# Copyright (c) 2016 Georgia Tech Research Institute (GTRI)
#               All Rights Reserved
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
# ---------------------------------------------------------------------------
# @file filename.ext
# @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
# @author Eric Squires <eric.squires@gtri.gatech.edu>
# @version 1.0
# ---------------------------------------------------------------------------
# @brief A brief description.
#
# @section DESCRIPTION
# A long description.
# ---------------------------------------------------------------------------
from __future__ import print_function
import scrimmage as sc
import numpy as np
from AuctionMsgs_pb2 import BidAuction, StartAuction
import lvdb

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
