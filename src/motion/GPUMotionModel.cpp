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


#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/motion/Controller.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/math/State.h>
#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/motion/GPUMotionModel.h>
#include <scrimmage/math/Quaternion.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <map>
#include <optional>
#include <vector>

namespace scrimmage {
  // Full state
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

  enum InputParams  {
    THRUST = 0,
    TURN_RATE,
    PITCH_RATE,
    INPUT_NUM_ITEMS
 };

  GPUMotionModel::GPUMotionModel(GPUControllerPtr gpu, 
      const std::string& kernel_name) : 
    gpu_{gpu},
    kernel_name_{kernel_name},
    states_{gpu, CL_MEM_READ_WRITE},
    inputs_{gpu, CL_MEM_WRITE_ONLY} // Inputs only need to be write only from our perspective
  {}

  void GPUMotionModel::add_entity(EntityPtr entity) {
    entities_.push_back(entity);
    vars_.try_emplace(entity);
    entity_added_ = true;
  }

  VariableIO& GPUMotionModel::get_entity_input(EntityPtr entity) {
    if(vars_.count(entity) == 0) {
      // Apparently this entity was not added. Add it now to avoid 
      // errors.
      add_entity(entity);
    }
    return vars_[entity];
  }

  /* (0): Right now, state/input data is stored on the heap.
  *          We will eventually have to iterate over every entity to pull this information out.
  *          This might be able to be avoided by transfering ownership of state info from individual entities to something higher up in
  *          In the higherarchy, but that would break a lot of things.
  *  (1): Map state & input buffers from device to a region in our Address Space. (Align to multiple of 64 bytes)
  *  (2): Copy State and Control Information from around heap into these regions of memory.
  *  (3): Unmap these buffers from our address space to ensure they are on the device.
  *  (4): Execute Kernel
  *  (5): Remap State Information into our address space 
  *  (6): Copy State Information back to entities state 
  *  (7): Unmap State buffer (could we avoid this until end of program? Reduce the number of mapping/unmapping operations we have to
  *   do? Maybe premeature optomization)
  */

  void GPUMotionModel::remove_inactive() {
    for(auto it = entities_.begin(); it != entities_.end();) {
      if (!(*it)->active()) {
        it = entities_.erase(it); 
      } else {
        ++it;
      }
    }
  }


  void GPUMotionModel::collect_state() {
    states_.resize(STATE_NUM_ITEMS * entities_.size());
    inputs_.resize(INPUT_NUM_ITEMS * entities_.size());

    // Map our device buffers into host memory to write to them;
    // We dont care about any possible data in our buffers rn. Invalidate
    // the region we are writing to.
    states_.map(CL_MAP_WRITE_INVALIDATE_REGION);
    inputs_.map(CL_MAP_WRITE_INVALIDATE_REGION);

    for(auto entityptr_it = entities_.cbegin();
        entityptr_it != entities_.cend(); 
        ++entityptr_it) {
      // For now, assume that controllers have already been run. 
      // Eventually I think we want to include the controller with the kernel.
      // This also assumes that all entites are homogenous in their state
      // and controller output
      EntityPtr entity = *entityptr_it;
      VariableIO& model_input = vars_[entity];
      
      // We only need to copy state information when a new entity is added.
      // Otherwise the state information of the device is consistant 
      // with the host, and we just copy that data back to the host after 
      // every step.
      StatePtr state = entity->state_truth();
      Eigen::Vector3d& pos = state->pos();
      Eigen::Vector3d& vel = state->vel();
      Eigen::Vector3d& ang_vel = state->ang_vel();
      Quaternion& quat = state->quat();
      states_.insert(states_.cend(),
          {
          (float) pos(0),
          (float) pos(1),
          (float) pos(2),
          (float) vel(0),
          (float) vel(1),
          (float) vel(2),
          (float) ang_vel(0),
          (float) ang_vel(1),
          (float) ang_vel(2),
          (float) quat.w(),
          (float) quat.x(),
          (float) quat.y(),
          (float) quat.z(),
          });
      // We always need to copy motion inputs as they change on the host.
      for(int i = 0; i < model_input.input()->size(); ++i) {
        inputs_.push_back((float) model_input.input(i));
      }
    }
    
    // Unmap buffers to commit changes to device.
    states_.unmap();
    inputs_.unmap();
  }

  bool GPUMotionModel::step(double time, double dt, std::size_t iterations) {
#if ENABLE_GPU_ACCELERATION == 1
    remove_inactive();
    collect_state();
    // Copy motion_inputs to mem objects and execute kenrnels
    cl_int err;

    //std::size_t num_ents = device_states_.size() / STATE_NUM_ITEMS;
    std::size_t num_ents = entities_.size();

    // Only reinitalize 
    std::string kernel_name{"simple_aircraft"};
    
    auto kernel_opt = gpu_->get_kernel(kernel_name);
    cl::Kernel motion_kernel;
    if(kernel_opt) {motion_kernel = kernel_opt.value();}
    else { 
      std::cerr << "Could not find kernel \'" << kernel_name << "\'\n";
      return false; 
    }
  
    const cl::CommandQueue& queue = gpu_->queue();

    err = motion_kernel.setArg(0, states_.device_buffer()); 
    GPUController::check_error(err, "Error setting Kernal Args");
    
    err = motion_kernel.setArg(1, inputs_.device_buffer()); 
    GPUController::check_error(err, "Error setting Kernal Args");
    err = motion_kernel.setArg(2, (float) time); 

    GPUController::check_error(err, "Error setting Kernal Args");

    err = motion_kernel.setArg(3, (float) dt);
    GPUController::check_error(err, "Error setting Kernal Args");

    err = queue.enqueueNDRangeKernel(motion_kernel,
        cl::NullRange, 
        cl::NDRange{num_ents},
        cl::NDRange{num_ents});

    GPUController::check_error(err, "Error Executing Kernel");
  
    gpu_->queue().finish();
    distribute_state();
#endif
    return true;
  }

  void GPUMotionModel::distribute_state()  {
    // Map state information back to host memory to copy to entites
    states_.map(CL_MAP_READ);

    std::size_t entity_ind;
    for(auto entity_ptr_it = entities_.begin();
        entity_ptr_it != entities_.end();
        ++entity_ptr_it) {
      entity_ind = std::distance(entities_.begin(), entity_ptr_it);
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
  }

  double GPUMotionModel::state_info(std::size_t entity_index, std::size_t state_index) {
    return states_.at(STATE_NUM_ITEMS*entity_index + state_index);
  }
} // namespace scrimmage

