/*!
 * @file
 *
 * @section LICENSE
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

#include <scrimmage/gpu/GPUMotionModel.h>
#include <scrimmage/gpu/GPUMotionModelImplementation.h>

#include <memory>

namespace scrimmage {
GPUMotionModelPtr GPUMotionModel::build_motion_model(const GPUPluginBuildParams& build_params) {
  const auto& kernel = build_params.kernel;
  const auto& queue = build_params.queue;
  if (build_params.single_precision) {
    return std::make_shared<GPUMotionModelImplementation<float>>(kernel, queue);
  } else {
    return std::make_shared<GPUMotionModelImplementation<double>>(kernel, queue);
  }
}

std::map<std::string, GPUMotionModelPtr> GPUMotionModel::build_motion_models(
    const std::map<std::string, GPUPluginBuildParams>& build_params) {
  std::map<std::string, GPUMotionModelPtr> motion_models;
  for (const auto& kv : build_params) {
    motion_models.insert({kv.first, build_motion_model(kv.second)});
  }
  return motion_models;
}
}  // namespace scrimmage
