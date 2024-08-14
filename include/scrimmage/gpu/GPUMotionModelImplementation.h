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

#ifndef INCLUDE_SCRIMMAGE_MOTION_GPUMOTIONMODELIMPLEMENTATION_H_
#define INCLUDE_SCRIMMAGE_MOTION_GPUMOTIONMODELIMPLEMENTATION_H_

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/gpu/GPUMapBuffer.h>
#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/math/State.h>
#include <scrimmage/gpu/GPUMotionModel.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <memory>

namespace scrimmage {

  template<typename T>
    class GPUMotionModelImplementation : public GPUMotionModel {

      enum StateParams {
        X = 0,
        Y,
        Z,
        X_VEL,
        Y_VEL,
        Z_VEL,
        X_ANG_VEL,
        Y_ANG_VEL,
        Z_ANG_VEL,
        QUAT_W,
        QUAT_X,
        QUAT_Y,
        QUAT_Z,
        STATE_NUM_ITEMS
      };

      public: 
      GPUMotionModelImplementation(cl::Kernel kernel, cl::CommandQueue queue):
        queue_{queue},
        kernel_{kernel},
        preferred_work_group_size_multiple_{0},
        max_work_group_size_{0},
        input_num_items_{0},
        states_{queue, CL_MEM_READ_WRITE},
        inputs_{queue, CL_MEM_READ_WRITE}
      {
        cl::Device device = queue.getInfo<CL_QUEUE_DEVICE>();
        kernel_.getWorkGroupInfo(device, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, &preferred_work_group_size_multiple_);
        max_work_group_size_ = device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>();

        /// Make sure these values are not 0.
        if(preferred_work_group_size_multiple_ == 0) {
          preferred_work_group_size_multiple_ = 64;
        }

        if(max_work_group_size_ == 0) {
          max_work_group_size_ = 256;
        }

        std::string attribute = kernel_.getInfo<CL_KERNEL_ATTRIBUTES>();
      }

      void add_entity(EntityPtr entity) {
        entity_mutex_.lock();
          to_init_.push_back(entity);
          vars_.try_emplace(entity);
        entity_mutex_.unlock();
      }

      VariableIO& get_entity_input(EntityPtr entity) {
        if(vars_.count(entity) == 0) {
          // Apparently this entity was not added. Add it now.
          add_entity(entity);
        }
        return vars_[entity];
      }

      void init_new_entities(double time) {
        // "Propagate" the motion model with dt=0. This should not update the motion,
        // but should ensure that all states are properly initalized.
        if (to_init_.size() > 0) {
          if(input_num_items_ == 0) {
            input_num_items_ = vars_[to_init_[0]].input()->size();
          }
          GPUMapBuffer<T> states{queue_, CL_MEM_READ_WRITE};
          GPUMapBuffer<T> inputs{queue_, CL_MEM_WRITE_ONLY};
          collect_states(to_init_, states, inputs); 
          propagate(states, inputs, to_init_.size(), time, 0, 1);
          distribute_states(to_init_, states);
          entities_.insert(entities_.end(), to_init_.begin(), to_init_.end()); 
          to_init_.clear();
        }
      }

      bool step(double time, double dt, std::size_t iterations) {
        remove_inactive();
        bool success = true;
        if(entities_.size() != 0) {
          collect_states(entities_, states_, inputs_);
          success = propagate(states_, inputs_, entities_.size(), time, dt, iterations);
          distribute_states(entities_, states_);
        }
        return success;
      }

      protected:
      void collect_states(std::vector<EntityPtr>& entities, GPUMapBuffer<T>& states, GPUMapBuffer<T>& inputs) { 
        states.resize(STATE_NUM_ITEMS * entities.size());
        inputs.resize(input_num_items_ * entities.size());

        // Map our device buffers into host memory to write to them;
        // We dont care about any possible data in our buffers rn. Invalidate
        // the region we are writing to.
        states.map(CL_MAP_WRITE_INVALIDATE_REGION);
        inputs.map(CL_MAP_WRITE_INVALIDATE_REGION);
        for(auto entityptr_it = entities.cbegin();
            entityptr_it != entities.cend(); 
            ++entityptr_it) {

          // For now, assume that controllers have already been run. 
          // Eventually I think we want to include the controller with the kernel.
          // This also assumes that all entites are homogenous in their state
          // and controller output
          EntityPtr entity = *entityptr_it;
          VariableIO& model_input = vars_[entity];

          // We only need to copy state information when a new entity is added.
          // Otherwise the state information of the device is consistent 
          // with the host, and we just copy that data back to the host after 
          // every step.
          StatePtr state = entity->state_truth();
          Eigen::Vector3d& pos = state->pos();
          Eigen::Vector3d& vel = state->vel();
          Eigen::Vector3d& ang_vel = state->ang_vel();
          Quaternion& quat = state->quat();
          states.insert(states.cend(),
              {
              static_cast<T>(pos(0)),
              static_cast<T>(pos(1)),
              static_cast<T>(pos(2)),
              static_cast<T>(vel(0)),
              static_cast<T>(vel(1)),
              static_cast<T>(vel(2)),
              static_cast<T>(ang_vel(0)),
              static_cast<T>(ang_vel(1)),
              static_cast<T>(ang_vel(2)),
              static_cast<T>(quat.w()),
              static_cast<T>(quat.x()),
              static_cast<T>(quat.y()),
              static_cast<T>(quat.z()),
              });
          // We always need to copy motion inputs as they change on the host.
          for(int i = 0; i < model_input.input()->size(); ++i) {
            inputs.push_back(static_cast<T>(model_input.input(i)));
          }
        }

        // Unmap buffers to commit changes to device.
        states.unmap();
        inputs.unmap();
      }

      bool propagate(
          GPUMapBuffer<T>& states, GPUMapBuffer<T>& inputs,
          std::size_t num_entities, double time, double dt, std::size_t iterations) {
#if ENABLE_GPU_ACCELERATION == 1
        // Copy motion_inputs to mem objects and execute kenrnels
        cl_int err;

        iterations = std::max(1ul, iterations);
        dt /= iterations;

        err = kernel_.setArg(0, states.device_buffer()); 
        CL_CHECK_ERROR(err, "Error setting Motion Model State Input Argument");

        err = kernel_.setArg(1, inputs.device_buffer()); 
        CL_CHECK_ERROR(err, "Error setting Motion Model Controller Inputer Argument");

        err = kernel_.setArg(2, static_cast<T>(time)); 
        CL_CHECK_ERROR(err, "Error setting time for kernel motion model");

        err = kernel_.setArg(3, static_cast<T>(dt));
        CL_CHECK_ERROR(err, "Error setting DT for kernel motion model");

        err = kernel_.setArg(4, static_cast<int>(num_entities));
        CL_CHECK_ERROR(err, "Error setting number of entities for kernel motion model");

        err = kernel_.setArg(5, static_cast<int>(iterations));
        CL_CHECK_ERROR(err, "Error setting number of iterations for kernel motion model");


        err = queue_.enqueueNDRangeKernel(kernel_,
            cl::NullRange, 
            cl::NDRange{num_entities},
            cl::NDRange{num_entities});
        CL_CHECK_ERROR(err, "Error Executing Kernel");
        
        queue_.finish();
#endif
        return true;
      }

      void distribute_states(std::vector<EntityPtr>& entities, GPUMapBuffer<T>& states)  {
        // Map state information back to host memory to copy to entites
        auto state_info = [&](std::size_t entity_index, std::size_t state_index) {
          return states.at(STATE_NUM_ITEMS*entity_index + state_index);
        };

        states.map(CL_MAP_READ);

        std::size_t entity_ind;
        for(auto entity_ptr_it = entities.begin();
            entity_ptr_it != entities.end();
            ++entity_ptr_it) {
          entity_ind = std::distance(entities.begin(), entity_ptr_it);
          StatePtr state = (*entity_ptr_it)->state_truth();
          state->pos() << state_info(entity_ind, X), 
            state_info(entity_ind, Y), 
            state_info(entity_ind, Z);
          state->vel() << state_info(entity_ind, X_VEL),
            state_info(entity_ind, Y_VEL),
            state_info(entity_ind, Z_VEL);
          state->ang_vel() << state_info(entity_ind, X_ANG_VEL),
            state_info(entity_ind, Y_ANG_VEL),
            state_info(entity_ind, Z_ANG_VEL);
          state->quat().w() = state_info(entity_ind, QUAT_W);
          state->quat().x() = state_info(entity_ind, QUAT_X);
          state->quat().y() = state_info(entity_ind, QUAT_Y);
          state->quat().z() = state_info(entity_ind, QUAT_Z);
        }

        states.unmap();
      }

      void remove_inactive() {
        for(auto it = entities_.begin(); it != entities_.end();) {
          if (!(*it)->active()) {
            it = entities_.erase(it); 
          } else {
            ++it;
          }
        }
      }

      cl::CommandQueue queue_;
      cl::Kernel kernel_;
      std::size_t preferred_work_group_size_multiple_;
      std::size_t max_work_group_size_;
      std::size_t input_num_items_;

      std::vector<EntityPtr> to_init_; // Buffer for initalization. Holds entities 
                                       // generated on the previous timestep to initalize
      std::vector<EntityPtr> entities_; // List of entities that use this motion model.
      std::map<EntityPtr, VariableIO> vars_; // Maps entities to their motion model inputs
      GPUMapBuffer<T> states_, inputs_;

      std::mutex entity_mutex_;
      std::mutex state_mutex_;
    };


} // namespace motion
#endif // INCLUDE_SCRIMMAGE_MOTION_MOTIONMODEL_H_
