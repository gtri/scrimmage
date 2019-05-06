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

#ifndef INCLUDE_SCRIMMAGE_PUBSUB_SERIALIZATION_H_
#define INCLUDE_SCRIMMAGE_PUBSUB_SERIALIZATION_H_

#include <scrimmage/math/State.h>
#include <scrimmage/math/Quaternion.h>

#include <string>
#include <memory>

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, Eigen::Vector3d & vec, const unsigned int version)
{
    ar & boost::serialization::make_nvp("x", vec[0]);
    ar & boost::serialization::make_nvp("y", vec[1]);
    ar & boost::serialization::make_nvp("z", vec[2]);
}

template<class Archive>
void serialize(Archive & ar, scrimmage::Quaternion & quat, const unsigned int version)
{
    ar & boost::serialization::make_nvp("w", quat.w());
    ar & boost::serialization::make_nvp("x", quat.vec()[0]);
    ar & boost::serialization::make_nvp("y", quat.vec()[1]);
    ar & boost::serialization::make_nvp("z", quat.vec()[2]);
}

template<class Archive>
void serialize(Archive & ar, scrimmage::State & state, const unsigned int version)
{
    ar & boost::serialization::make_nvp("pos", state.pos());
    ar & boost::serialization::make_nvp("vel", state.vel());
    ar & boost::serialization::make_nvp("quat", state.quat());
    ar & boost::serialization::make_nvp("ang_vel", state.ang_vel());
}

} // namespace serialization
} // namespace boost

#endif // INCLUDE_SCRIMMAGE_PUBSUB_SERIALIZATION_H_
