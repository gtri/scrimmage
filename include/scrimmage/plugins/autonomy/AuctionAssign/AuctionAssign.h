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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AUCTIONASSIGN_AUCTIONASSIGN_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AUCTIONASSIGN_AUCTIONASSIGN_H_

#include <scrimmage/autonomy/Autonomy.h>

#include <map>
#include <string>
#include <limits>

namespace scrimmage {
namespace autonomy {
class AuctionAssign : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    PublisherPtr start_auction_pub_;
    PublisherPtr bid_auction_pub_;
    PublisherPtr result_auction_pub_;

    bool auction_started_ = false;
    bool auction_in_prog_ = false;
    double auction_start_time_ = -std::numeric_limits<double>::infinity();
    double auction_max_time_ = 5;
    double max_bid_ = -std::numeric_limits<double>::max();
    int max_bid_champ_ = -1;

    int id_ = -1;
    bool auctioneer_;

    int output_vel_x_idx_ = 0;
    int output_vel_y_idx_ = 0;
    int output_vel_z_idx_ = 0;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_AUCTIONASSIGN_AUCTIONASSIGN_H_
