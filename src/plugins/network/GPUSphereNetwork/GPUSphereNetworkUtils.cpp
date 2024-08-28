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

// #include <scrimmage/plugins/network/GPUSphereNetwork/GPUSphereNetwork.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugins/network/GPUSphereNetwork/GPUSphereNetworkUtils.h>

namespace sc = scrimmage;

namespace scrimmage {
namespace network {  // namespace network

GPUSphereNetworkUtils::GPUSphereNetworkUtils(double range,
                                             const GPUPluginBuildParams& network_kernel_params)
    : range_{range},
      queue_{network_kernel_params.queue},
      kernel_{network_kernel_params.kernel} {
    assert(range > 0);

    cl::Device device = queue_.getInfo<CL_QUEUE_DEVICE>();
    prefered_workgroup_size_multiple_ =
        OpenCLUtils::prefered_workgroup_size_multiples(kernel_, device);
    max_workgroup_size_ = OpenCLUtils::max_workgroup_size(device);
}

std::set<std::pair<int, int>> GPUSphereNetworkUtils::proximity_pairs(
    std::map<int, StatePtr> states) {
    using EntityPair = std::pair<int, int>;
    std::set<EntityPair> proximity_pairs;

    cl_int err;
    // Used to associate an index in our state buffer to a specific entity
    std::map<int, std::size_t> id_map;

    std::size_t num_entities = states.size();
    GPUMapBuffer<double> positions{queue_, 3 * num_entities, CL_MEM_READ_WRITE};
    GPUMapBuffer<bool> reachable_map{
        queue_, num_entities * (num_entities - 1) / 2, CL_MEM_READ_WRITE};
    GPUMapBuffer<double> distances{
        queue_, num_entities * (num_entities - 1) / 2, CL_MEM_READ_WRITE};

    positions.map(CL_MAP_WRITE_INVALIDATE_REGION);
    std::size_t i = 0;
    for (const auto& kv : states) {
        const auto& position = kv.second->pos();
        positions.at(3 * i) = position(0);
        positions.at(3 * i + 1) = position(1);
        positions.at(3 * i + 2) = position(2);
        id_map[kv.first] = i++;
    }
    positions.unmap();

    bool num_entities_even = num_entities % 2 == 0;
    int rows = (num_entities_even) ? num_entities - 1 : num_entities;
    int cols = (num_entities_even) ? num_entities / 2 : (num_entities - 1) / 2;

    reachable_map.map(CL_MAP_WRITE);
    reachable_map.fill(false);
    reachable_map.unmap();

    distances.map(CL_MAP_WRITE);
    distances.fill(-1.0);
    distances.unmap();

    err = kernel_.setArg(0, positions.device_buffer());
    CL_CHECK_ERROR(err, "Error setting GPU Sphere Network Position Inputs");

    err = kernel_.setArg(1, reachable_map.device_buffer());
    CL_CHECK_ERROR(err, "Error setting GPU Sphere Network Reachable Map");

    err = kernel_.setArg(2, distances.device_buffer());
    CL_CHECK_ERROR(err, "Error setting GPU Sphere Network Distances");

    err = kernel_.setArg(3, range_);
    CL_CHECK_ERROR(err, "Error setting GPU Sphere Network Range");

    err = kernel_.setArg(4, rows);
    CL_CHECK_ERROR(err, "Error setting GPU Sphere Network Rows");

    err = kernel_.setArg(5, cols);
    CL_CHECK_ERROR(err, "Error setting GPU Sphere Network Cols");

    std::ldiv_t row_div = std::ldiv(rows, prefered_workgroup_size_multiple_);
    std::ldiv_t col_div = std::ldiv(cols, prefered_workgroup_size_multiple_);

    std::size_t upper_bound_row_size =
        (row_div.quot + ((row_div.rem > 0) ? 1 : 0)) * prefered_workgroup_size_multiple_;
    std::size_t upper_bound_col_size =
        (col_div.quot + ((col_div.rem > 0) ? 1 : 0)) * prefered_workgroup_size_multiple_;

    cl::NDRange global_work_size{upper_bound_row_size, upper_bound_col_size};

    std::size_t max_dim_size = std::sqrt(max_workgroup_size_);
    cl::NDRange local_work_size{std::min(max_dim_size, upper_bound_row_size),
                                std::min(max_dim_size, upper_bound_col_size)};

    err = queue_.enqueueNDRangeKernel(kernel_, cl::NullRange, global_work_size, local_work_size);
    CL_CHECK_ERROR(err, "Error Executing Kernels");

    queue_.finish();

    reachable_map.map(CL_MAP_READ);
    std::vector<bool> values(reachable_map.cbegin(), reachable_map.cend());
    reachable_map.unmap();

    distances.map(CL_MAP_READ);
    std::vector<double> ent_distances(distances.cbegin(), distances.cend());
    distances.unmap();

    std::size_t n = num_entities - 1;
    for (std::size_t i = 0; i < values.size(); i++) {
        bool within_range = values[i];

        std::ldiv_t pos = std::ldiv(i, rows);
        std::size_t row = pos.rem;
        std::size_t col = pos.quot;

        std::size_t first_ind = col;
        std::size_t second_ind = row;

        bool on_diagonal = (row == col);
        bool above_diagonal = (row < col);
        bool odd_num_ents = num_entities % 2 == 1;
        bool reverse_indexing = (odd_num_ents && on_diagonal) || above_diagonal;

        if (reverse_indexing) {
            if (odd_num_ents) {
                first_ind = n - 1 - col;
            } else {
                first_ind = n - col;
            }
            second_ind = n - row;
        } else if (!odd_num_ents) {
            second_ind += 1;
        }

        if (within_range) {
            proximity_pairs.emplace(first_ind, second_ind);
        }
    }

    return proximity_pairs;
}

}  // namespace network
}  // namespace scrimmage
