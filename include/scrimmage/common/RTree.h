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

#ifndef INCLUDE_SCRIMMAGE_COMMON_RTREE_H_
#define INCLUDE_SCRIMMAGE_COMMON_RTREE_H_

#include <scrimmage/common/ID.h>

#include <Eigen/Dense>

#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <utility>

#include <boost/tuple/tuple.hpp>
#include <boost/geometry/index/detail/exception.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/index/parameters.hpp> // for dynamic_rstar definition
#include <boost/geometry/geometries/point.hpp> // for model::point
#include <boost/geometry/index/indexable.hpp>

namespace boost { namespace geometry { namespace index {
// boost/geometry/index/rtree.hpp
template <typename T1, typename T2, typename T3, typename T4, typename T5> class rtree;
}}}

namespace scrimmage {

typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef std::pair<point, ID> point_id_t;
typedef boost::geometry::index::rtree<
    point_id_t,
    boost::geometry::index::dynamic_rstar,
    boost::geometry::index::indexable<point_id_t>,
    std::equal_to<point_id_t>,
    std::allocator<point_id_t>> rtree_t;

typedef std::shared_ptr<rtree_t> rtreePtr;

class RTree {
 public:
    void init(int size);
    void clear();

    void add(Eigen::Vector3d &pos, const ID &id);
    void nearest_n_neighbors(const Eigen::Vector3d &pos,
                             std::vector<ID> &neighbors, unsigned int n,
                             int self_id = -1, int team_id = -1) const;
    void neighbors_in_range(const Eigen::Vector3d &pos,
                            std::vector<ID> &neighbors, double dist,
                            int self_id = -1, int team_id = -1) const;
 protected:
    rtreePtr rtree_;
    std::map<int, rtreePtr> rtree_team_;
    int size_ = 0;
};

typedef std::shared_ptr<RTree> RTreePtr;
}  // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_RTREE_H_
