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

#ifndef INCLUDE_SCRIMMAGE_MOTION_GPUMOTIONMODEL_H_
#define INCLUDE_SCRIMMAGE_MOTION_GPUMOTIONMODEL_H_

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/gpu/GPUMapBuffer.h>
#include <scrimmage/math/State.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <memory>
#include <optional>

namespace scrimmage {

  class GPUMotionModel : public Plugin {
    public: 
        GPUMotionModel(GPUControllerPtr gpu, const std::string& kernel_name);
        void init_new_entities(double time);
        bool step(double time, double dt, std::size_t iterations); // Enque and exeucte kernel
        void add_entity(EntityPtr entity);
        VariableIO& get_entity_input(EntityPtr entity);

    protected:
      void collect_states(std::vector<EntityPtr>& entities, GPUMapBuffer<float>& states, GPUMapBuffer<float>& inputs);
      bool propagate(GPUMapBuffer<float>& states, GPUMapBuffer<float>& inputs, std::size_t num_entities, double time, double dt, std::size_t iterations);
      void distribute_states(std::vector<EntityPtr>& entities, GPUMapBuffer<float>& states);
      void remove_inactive();
//double state_info(std::size_t entity_index, std::size_t state_index);

      std::vector<EntityPtr> to_init_; // Buffer for initalization. Holds entities 
                                       // generated on the previous timestep to initalize
      std::vector<EntityPtr> entities_; // List of entities that use this motion model.
      std::map<EntityPtr, VariableIO> vars_; // Maps entities to their motion model inputs

      GPUControllerPtr gpu_;
      std::string kernel_name_;

      GPUMapBuffer<float> states_, inputs_;

      std::size_t input_num_items_;
  };

  using GPUMotionModelPtr = std::shared_ptr<GPUMotionModel>;
} // namespace motion
#endif // INCLUDE_SCRIMMAGE_MOTION_MOTIONMODEL_H_
