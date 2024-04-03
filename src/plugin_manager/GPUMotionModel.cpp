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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/motion/GPUMotionModel.h>
#include <scrimmage/gpu/GPUController.h>
#include <CL/cl2.hpp>

namespace scrimmage {
namespace motion {

enum ModelParams {
    X = 0,
    Y,
    Z,
    ROLL,
    PITCH,
    YAW,
    SPEED,
    MODEL_NUM_ITEMS
};

enum ControlParams {
    THRUST = 0,
    TURN_RATE,
    PITCH_RATE,
    CONTROL_NUM_ITEMS
};

std::tuple<int, int, int> GPUMotionModel::version() {
    return std::tuple<int, int, int>(0, 0, 1);
}

bool GPUMotionModel::init(std::map<std::string, std::string> &info,
                          std::map<std::string, std::string> &params) {
    x_.resize(MODEL_NUM_ITEMS);
    Eigen::Vector3d &pos = state_->pos();
    Quaternion &quat = state_->quat();

    min_velocity_ = get("min_velocity", params, 15.0);
    max_velocity_ = get("max_velocity", params, 40.0);
    max_roll_ = Angles::deg2rad(get("max_roll", params, 30.0));
    max_pitch_ = Angles::deg2rad(get("max_pitch", params, 30.0));
    max_pitch_rate_ = Angles::deg2rad(get("max_pitch_rate", params, 57.3));
    max_roll_rate_ = Angles::deg2rad(get("max_roll_rate", params, 57.3));

    state_offset_ = STATES.size();
    control_offset_ = CONTROLS.size();
    STATES.resize(state_offset_ + MODEL_NUM_ITEMS);
    CONTROLS.resize(control_offset_ + CONTROL_NUM_ITEMS);

    // Set initial parameters
    state_info(X) = pos(0);
    state_info(Y) = pos(1);
    state_info(Z) = pos(2);
    state_info(ROLL) = std::clamp(quat.roll(), -max_roll_, max_roll_);
    state_info(PITCH) = std::clamp(quat.pitch(), -max_pitch_, max_pitch_);
    state_info(YAW) = quat.yaw();
    state_info(SPEED) = std::clamp(state_->vel().norm(), min_velocity_, max_velocity_);

    length_ = get("turning_radius", params, 50.0);
    speedTarget_ = get("speed_target", params, 50.0);  // The "0" speed for adjusting the turning radius
    lengthSlopePerSpeed_ = get("radius_slope_per_speed", params, 0.0);  // Enables adjusting the turning radius based on speed

    state_->pos() << state_info(X), state_info(Y), state_info(Z);
    state_->quat().set(-state_info(ROLL), state_info(PITCH), state_info(YAW));
    state_->vel() << state_info(SPEED) * cos(state_info(YAW)), 
      state_info(SPEED) * sin(state_info(YAW)), 0;

    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::In);
    roll_rate_idx_ = vars_.declare(VariableIO::Type::roll_rate, VariableIO::Direction::In);
    pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::In);

    return true;
}

bool GPUMotionModel::step_gpu(GPUControllerPtr gpu, double time, double dt) {
  std::string model_kernel_name = "simple_aircraft_model";
  std::string ode45_kernel_name = "ode45";

  std::optional<cl::Kernel> opt_model_kernel = 
    gpu->get_kernel(model_kernel_name);   

  if (!opt_model_kernel) {
    std::cerr << "Unable to load " << model_kernel_name << std::endl;
    return false;
  }

  std::optional<cl::Kernel> opt_ode45_kernel = 
    gpu->get_kernel(ode45_kernel_name);   

  if (!opt_ode45_kernel) {
    std::cerr << "Unable to load " << ode45_kernel_name << std::endl;
    return false;
  }

  cl::Kernel model_kernel = opt_model_kernel.value();
  cl::Kernel ode45_kernel = opt_ode45_kernel.value();

  std::size_t state_type_size = sizeof(decltype(STATES)::value_type);
  std::size_t control_type_size = sizeof(decltype(CONTROLS)::value_type);

  std::size_t state_size = STATES.size()*state_type_size;
  std::size_t control_size = CONTROLS.size()*control_type_size;

  cl_int err;
  cl::Buffer state_buff = gpu->make_buffer(
      CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, 
      state_size, STATES.data(), &err);

  cl::Buffer derivative_buff = gpu->make_buffer(
      CL_MEM_READ_WRITE, state_size, nullptr, &err);

  cl::Buffer control_buff = gpu->make_buffer(
      CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, 
      control_size, CONTROLS.data(), &err);

  model_kernel.setArg(0, state_buff);
  model_kernel.setArg(1, control_buff);
  model_kernel.setArg(3, derivative_buff);
  model_kernel.setArg(3, MODEL_NUM_ITEMS);
  model_kernel.setArg(4, CONTROL_NUM_ITEMS);

  ode45_kernel.setArg(0, state_buff);
  ode45_kernel.setArg(1, derivative_buff);
  ode45_kernel.setArg(2, MODEL_NUM_ITEMS);
  ode45_kernel.setArg(3, dt);

  void *state_mmap, *derivative_mmap, *control_mmap;

  cl::CommandQueue& queue = gpu->queue();
  state_mmap = queue.enqueueMapBuffer(state_buff,
      true, CL_MAP_WRITE | CL_MAP_READ, 0, state_size, nullptr, nullptr, &err);

  derivative_mmap = queue.enqueueMapBuffer(derivative_buff,
      true, CL_MAP_WRITE | CL_MAP_READ, 0, state_size, nullptr, nullptr, &err);

  control_mmap = queue.enqueueMapBuffer(control_buff, 
      true, CL_MAP_READ, 0, state_size, nullptr, nullptr, &err);

  queue.enqueueMapBuffer(control_buff, true, CL_MAP_READ, 0, state_size, 
      nullptr, nullptr, &err);

  std::size_t num_work_items = STATES.size() / MODEL_NUM_ITEMS;
  std::size_t local_items = 256;

  queue.enqueueNDRangeKernel(model_kernel, cl::NDRange{}, 
      cl::NDRange{num_work_items}, cl::NDRange{local_items});

  queue.enqueueNDRangeKernel(ode45_kernel, cl::NDRange{},
      cl::NDRange{num_work_items}, cl::NDRange{local_items});

  memcpy(STATES.data(), (float*) state_mmap, state_size);
    
  queue.enqueueUnmapMemObject(state_buff, state_mmap);
  queue.enqueueUnmapMemObject(derivative_buff, derivative_mmap);
  queue.enqueueUnmapMemObject(control_buff, control_mmap);

  return true;
}

bool GPUMotionModel::step(double time, double dt) {
    //// Need to saturate state variables before model runs
    //x_[ROLL] = std::clamp(x_[ROLL], -max_roll_, max_roll_);
    //x_[PITCH] = std::clamp(x_[PITCH], -max_pitch_, max_pitch_);
    //x_[SPEED] = std::clamp(x_[SPEED], min_velocity_, max_velocity_);

    //ode_step(dt);

    state_->pos() << state_info(X), state_info(Y), state_info(Z);
    state_->quat().set(-state_info(ROLL), state_info(PITCH), state_info(YAW));
    state_->vel() << state_info(SPEED) * cos(state_info(YAW)) * cos(state_info(PITCH)), 
      state_info(SPEED) * sin(state_info(YAW)) * cos(state_info(PITCH)), 
      state_info(SPEED) * sin(state_info(PITCH));

    return true;
}

void GPUMotionModel::model(const vector_t &x , vector_t &dxdt , double t) {
    /// 0 : x-position
    /// 1 : y-position
    /// 2 : z-position
    /// 3 : roll
    /// 4 : pitch
    /// 5 : yaw
    /// 6 : speed
    double throttle = vars_.input(throttle_idx_);
    double roll_rate = vars_.input(roll_rate_idx_);
    double pitch_rate = vars_.input(pitch_rate_idx_);

    // Saturate control inputs
    throttle = std::clamp(throttle, -100.0, 100.0);
    roll_rate = std::clamp(roll_rate, -max_roll_rate_, max_roll_rate_);
    pitch_rate = std::clamp(pitch_rate, -max_pitch_rate_, max_pitch_rate_);

    double xy_speed = x[SPEED] * cos(x[PITCH]);
    dxdt[X] = xy_speed*cos(x[YAW]);
    dxdt[Y] = xy_speed*sin(x[YAW]);
    dxdt[Z] = -sin(x[PITCH])*x[SPEED];
    dxdt[ROLL] = roll_rate;
    dxdt[PITCH] = pitch_rate;
    // Adjust the length based on the speed
    double currentLength = length_ + lengthSlopePerSpeed_ * (x[SPEED] - speedTarget_);
    dxdt[YAW] = x[SPEED]/currentLength*tan(x[ROLL]);

    dxdt[SPEED] = throttle/5;
}

void GPUMotionModel::teleport(StatePtr &state) {
    state_info(X) = state->pos()[0];
    state_info(Y) = state->pos()[1];
    state_info(Z) = state->pos()[2];
    state_info(ROLL) = state->quat().roll();
    state_info(PITCH) = state->quat().pitch();
    state_info(YAW) = state->quat().yaw();
    state_info(SPEED) = state->vel()[0];
}
}  // namespace motion
}  // namespace scrimmage
