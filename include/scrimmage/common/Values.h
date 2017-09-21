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

#ifndef INCLUDE_SCRIMMAGE_COMMON_VALUES_H_
#define INCLUDE_SCRIMMAGE_COMMON_VALUES_H_

#include <algorithm>

namespace scrimmage {

/*! \brief extracts values from a map
 *
 * inspired by
 * https://stackoverflow.com/a/35262398
 */
template <class MapType>
class ValuesIterator : public MapType::iterator {
 public:
    explicit ValuesIterator(typename MapType::iterator b, typename MapType::iterator e) :
        MapType::iterator(b), begin_(b), end_(e) {}

    typename MapType::key_type *operator-> () {
        return (typename MapType::key_type* const)&(MapType::iterator::operator->()->second);
    }

    typename MapType::key_type operator*( ) {
        return MapType::iterator::operator*().second;
    }

    ValuesIterator<MapType> begin() {
        return ValuesIterator<MapType>(begin_, end_);
    }

    ValuesIterator<MapType> end() {
        return ValuesIterator<MapType>(end_, end_);
    }

    template <class ContainerType>
    ContainerType as() {
        ContainerType out;
        std::transform(begin_, end_, std::inserter(out, std::end(out)),
            [&](auto &kv) {return kv.second;});

        return out;
    }

 protected:
    typename MapType::iterator begin_;
    typename MapType::iterator end_;
};

template <class MapType>
ValuesIterator<MapType> values(MapType &map) {
    return ValuesIterator<MapType>(map.begin(), map.end());
}
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_VALUES_H_
