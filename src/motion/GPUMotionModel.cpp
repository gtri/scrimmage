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

#include <CL/opencl.hpp>

namespace scrimmage {
  // Full state
  enum ModelParams {
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
    MODEL_NUM_ITEMS
  };

  enum ControlParams  {
    THRUST = 0,
    TURN_RATE,
    PITCH_RATE,
    CONTROL_NUM_ITEMS
  };

  GPUMotionModel::GPUMotionModel(GPUControllerPtr gpu) : gpu_{gpu} {}

  void GPUMotionModel::collect(std::list<EntityPtr> entities) {
    states_.reserve(MODEL_NUM_ITEMS * entities.size());
    states_.clear();

    control_inputs_.reserve(CONTROL_NUM_ITEMS * entities.size());
    control_inputs_.clear();

    for(auto entityptr_it = entities.cbegin();
        entityptr_it != entities.cend(); 
        ++entityptr_it) {
      // For now, assume that controllers have already been run. 
      // Eventually I think we want to include the controller with the kernel.
      // This also assumes that all entites are homogonous in their state
      // and controller output
      EntityPtr entity = *entityptr_it;
      VariableIO autonomy_output = entity->autonomies().back()->vars();
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

      auto output = *autonomy_output.output();
      for(int i = 0; i < output.size(); ++i) {
        control_inputs_.push_back(output(i));
      }
    }
  }

  bool GPUMotionModel::step(double dt, std::size_t iterations) {
    // Copy motion_inputs to mem objects and execute kenrnels
    cl_int err;

    std::size_t num_ents = states_.size() / MODEL_NUM_ITEMS;

    cl::Buffer state_buffer{
      gpu_->context(), 
        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, 
        sizeof(double)*states_.size(),
        states_.data(), &err};

    cl::Buffer control_buffer{
      gpu_->context(), 
        CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 
        sizeof(double)*control_inputs_.size(),
        control_inputs_.data(), &err};

    std::string kernel_name{"static_motion"};
    
    auto kernel_opt = gpu_->get_kernel(kernel_name);
    cl::Kernel motion_kernel;
    if(kernel_opt) {motion_kernel = kernel_opt.value();}
    else { 
      std::cerr << "Could not find kernel \'static_motion\'" << std::endl;
      return false; 
    }
  
    cl::CommandQueue& queue = gpu_->queue();

    motion_kernel.setArg(0, state_buffer); 
    motion_kernel.setArg(1, control_buffer); 
    motion_kernel.setArg(2, MODEL_NUM_ITEMS);
    motion_kernel.setArg(3, CONTROL_NUM_ITEMS);

    std::size_t value_size = sizeof(decltype(states_)::value_type);

    err = queue.enqueueWriteBuffer(state_buffer,
        true, 0, value_size*states_.size(),
        states_.data());

    err = queue.enqueueWriteBuffer(control_buffer,
        true, 0, value_size*control_inputs_.size(),
        control_inputs_.data());

    err = queue.enqueueNDRangeKernel(motion_kernel,
        cl::NullRange, 
        cl::NDRange{num_ents},
        cl::NDRange{num_ents});

    err = queue.enqueueReadBuffer(state_buffer,
        true, 0, value_size*states_.size(),
        states_.data());
  

    return true;
  }

  void GPUMotionModel::reassign(std::list<EntityPtr> entities)  {
    std::size_t offset = 0;
    auto state_info = [&](std::size_t indx) {
      return states_[offset + indx];
    };

    for(auto entity_ptr_it = entities.begin();
        entity_ptr_it != entities.end();
        ++entity_ptr_it) {
      offset = std::distance(entity_ptr_it, entities.begin());
      StatePtr state = (*entity_ptr_it)->state_truth();
      state->pos() << state_info(X), state_info(Y), state_info(Z);
      state->vel() << state_info(X_VEL), state_info(Y_VEL), state_info(Z_VEL);
      state->ang_vel() << state_info(X_ANG_VEL), state_info(Y_ANG_VEL), state_info(Z_ANG_VEL);
    }
  }
} // namespace scrimmage

