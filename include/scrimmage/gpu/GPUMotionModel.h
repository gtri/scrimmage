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

#ifndef INCLUDE_SCRIMMAGE_GPU_GPUMOTIONMODEL_H_
#define INCLUDE_SCRIMMAGE_GPU_GPUMOTIONMODEL_H_

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif


namespace scrimmage {
  class GPUPluginBuildParams;

  class GPUMotionModel {
    public:
      virtual void add_entity(EntityPtr entity) = 0; 
      virtual VariableIO& get_entity_input(EntityPtr entity) = 0; 
      virtual void init_new_entities(double time) = 0;
      virtual bool step(double time, double dt, std::size_t iterations) = 0; // Enqueue and execute kernel

#if ENABLE_GPU_ACCELERATION == 1
      static GPUMotionModelPtr build_motion_model(const GPUPluginBuildParams& build_params);

      static std::map<std::string, GPUMotionModelPtr> build_motion_models(
          const std::map<std::string, GPUPluginBuildParams>& build_params);
#endif
  };
} // namespace motion
#endif // INCLUDE_SCRIMMAGE_GPU_GPUMOTIONMODEL_H_
