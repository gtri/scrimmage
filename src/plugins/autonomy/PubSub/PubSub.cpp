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
 * @author Christopher Richardson <christopher.richardson@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/autonomy/PubSub/PubSub.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <iostream>
#include <limits>
#include <memory>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::PubSub,
                PubSub_plugin)

namespace scrimmage {
namespace autonomy {

void PubSub::init(std::map<std::string, std::string> &params) {
    pub_logged_ = advertise("GlobalNetwork", "LoggedTopic", 1);

    // sub
    subscribe<std::vector<scrimmage::State>>(
        "GlobalNetwork", "LoggedTopic",
        std::bind(&PubSub::callback_logged, this, std::placeholders::_1));

    ofs_.open("test_boost_serial.xml");
}

bool PubSub::step_autonomy(double t, double dt) {

    // pub
    std::vector<scrimmage::State> vec;
    scrimmage::State st = *state_;
    for (int i=0; i<3; ++i) {
        Eigen::Vector3d curpos = st.pos();
        st.pos() = curpos + Eigen::Vector3d(100, 0, 0);
        vec.push_back(st);
    }
    auto msg = std::make_shared<sc::Message<std::vector<scrimmage::State>>>();
    msg->data = vec;
    pub_logged_->publish(msg);

    return true;
}

void PubSub::callback_logged(scrimmage::MessagePtr<std::vector<scrimmage::State>> msg) {
    std::cout << "PubSub: received state vector. Serializing to xml..." << std::endl;
    boost::archive::xml_oarchive oa(ofs_);
    oa << boost::serialization::make_nvp("state_vec", msg->data);
}
} // namespace autonomy
} // namespace scrimmage
