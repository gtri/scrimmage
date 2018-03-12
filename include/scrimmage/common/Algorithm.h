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

#ifndef INCLUDE_SCRIMMAGE_COMMON_ALGORITHM_H_
#define INCLUDE_SCRIMMAGE_COMMON_ALGORITHM_H_

#include <iterator>
#include <algorithm>
#include <unordered_set>

namespace scrimmage {
/*! \brief std::remove_if does not work with associative containers.
 *
 * This implementation assumes the first argument has an erase method.
 */
template <class ContainerType, class Predicate>
void remove_if(ContainerType &container, Predicate pred) {
    auto it = container.begin();
    while (it != container.end()) {
        it = pred(*it) ? container.erase(it) : std::next(it);
    }
}

/*! \brief Returns container1 minus container2
 *
 * std::set_difference does not work on unordered_sets
 */
template <class T = std::unordered_set<int>>
std::unordered_set<T>
set_difference(const std::unordered_set<T> &container1, const std::unordered_set<T> &container2) {
    std::unordered_set<T> out;
    std::copy_if(container1.begin(), container1.end(), std::inserter(out, out.end()),
        [&](const T &val) {return container2.count(val) == 0;});
    return out;
}

template <class T = std::unordered_set<int>>
std::unordered_set<T>
set_union(const std::unordered_set<T> &container1, const std::unordered_set<T> &container2) {
    std::unordered_set<T> out = container1;
    out.insert(container2.begin(), container2.end());
    return out;
}

template <class T = std::unordered_set<int>>
std::unordered_set<T>
set_intersection(const std::unordered_set<T> &container1, const std::unordered_set<T> &container2) {
    std::unordered_set<T> out;
    std::copy_if(container1.begin(), container1.end(), std::inserter(out, out.end()),
        [&](const T &val) {return container2.count(val) != 0;});
    return out;
}
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_COMMON_ALGORITHM_H_
