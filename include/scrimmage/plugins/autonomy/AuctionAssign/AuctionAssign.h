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

#ifndef AuctionAssign_H_
#define AuctionAssign_H_
#include <scrimmage/autonomy/Autonomy.h>

class AuctionAssign : public scrimmage::Autonomy {
public:
    AuctionAssign();
    virtual void init(std::map<std::string,std::string> &params);
    virtual bool step_autonomy(double t, double dt);
protected:

    scrimmage::PublisherPtr pub_start_auction_;
    scrimmage::PublisherPtr pub_bid_auction_;
    scrimmage::SubscriberPtr sub_start_auction_;
    scrimmage::SubscriberPtr sub_bid_auction_;

    bool auction_started_;    
    bool auction_in_prog_;
    double auction_start_time_;
    double auction_max_time_;
    double max_bid_;
    int max_bid_champ_;
    int id_;
private:     
};

#endif
