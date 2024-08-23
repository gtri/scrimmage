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

#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/gpu/GPUMotionModel.h>
#include <scrimmage/gpu/GPUMotionModelImplementation.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <memory>
#include <optional>

namespace scrimmage {
namespace OpenCLUtils {

bool supports_fp64(const cl::Device& device) {
  cl_int err;
  constexpr cl_device_fp_config MIN_FP_CAPABILITIES =
      (CL_FP_FMA | CL_FP_ROUND_TO_NEAREST | CL_FP_ROUND_TO_ZERO | CL_FP_ROUND_TO_INF);
  cl_device_fp_config fp_config = device.getInfo<CL_DEVICE_DOUBLE_FP_CONFIG>(&err);
  if (check_error(err, "Error Querying Device for Floating Point Information.")) {
    return false;
  }
  return (fp_config & MIN_FP_CAPABILITIES) == MIN_FP_CAPABILITIES;
}

bool check_error(cl_int err,
                 const std::string&& file,
                 const unsigned int linenumber,
                 const std::string&& msg) {
  return OpenCLUtils::check_error(err,
                                  "at " + file + "(" + std::to_string(linenumber) + "): " + msg);
}

std::size_t prefered_workgroup_size_multiples(const cl::Device& device) {
    cl_int err;
    std::size_t workgroup_size_mul = device.getInfo<CL_DEVICE_PREFERRED_WORK_GROUP_SIZE_AMD>(&err);
}

bool check_error(cl_int err, const std::string&& msg) {
  if (err != 0) {
    fprintf(stderr,
            "OpenCL Error: %s (CODE: %d): %s\n",
            CL_ERROR_MESSAGES.at(err).c_str(),
            err,
            msg.c_str());
    return true;
  }
  return false;
};

std::optional<std::size_t> cacheline_size(const cl::Device& device) {
  // Check if device has cache
  cl_int err;
  if (device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHE_TYPE>(&err) == CL_NONE) {
    check_error(err, "Error Querying Device Cache Type");
    return std::nullopt;
  }

  std::size_t cacheline_size = device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>(&err);

  if (check_error(err, "Error Querying Device Cacheline Size")) {
    return std::nullopt;
  }
  return std::make_optional(cacheline_size);
}

const std::map<cl_int, std::string> CL_ERROR_MESSAGES = {
    std::make_pair<cl_int, std::string>(CL_SUCCESS, "CL_SUCCESS"),
    std::make_pair<cl_int, std::string>(CL_DEVICE_NOT_FOUND, "CL_DEVICE_NOT_FOUND"),
    std::make_pair<cl_int, std::string>(CL_DEVICE_NOT_AVAILABLE, "CL_DEVICE_NOT_AVAILABLE"),
    std::make_pair<cl_int, std::string>(CL_COMPILER_NOT_AVAILABLE, "CL_COMPILER_NOT_AVAILABLE"),
    std::make_pair<cl_int, std::string>(CL_MEM_OBJECT_ALLOCATION_FAILURE,
                                        "CL_MEM_OBJECT_ALLOCATION_FAILURE"),
    std::make_pair<cl_int, std::string>(CL_OUT_OF_RESOURCES, "CL_OUT_OF_RESOURCES"),
    std::make_pair<cl_int, std::string>(CL_OUT_OF_HOST_MEMORY, "CL_OUT_OF_HOST_MEMORY"),
    std::make_pair<cl_int, std::string>(CL_PROFILING_INFO_NOT_AVAILABLE,
                                        "CL_PROFILING_INFO_NOT_AVAILABLE"),
    std::make_pair<cl_int, std::string>(CL_MEM_COPY_OVERLAP, "CL_MEM_COPY_OVERLAP"),
    std::make_pair<cl_int, std::string>(CL_IMAGE_FORMAT_MISMATCH, "CL_IMAGE_FORMAT_MISMATCH"),
    std::make_pair<cl_int, std::string>(CL_IMAGE_FORMAT_NOT_SUPPORTED,
                                        "CL_IMAGE_FORMAT_NOT_SUPPORTED"),
    std::make_pair<cl_int, std::string>(CL_BUILD_PROGRAM_FAILURE, "CL_BUILD_PROGRAM_FAILURE"),
    std::make_pair<cl_int, std::string>(CL_MAP_FAILURE, "CL_MAP_FAILURE"),
    std::make_pair<cl_int, std::string>(CL_MISALIGNED_SUB_BUFFER_OFFSET,
                                        "CL_MISALIGNED_SUB_BUFFER_OFFSET"),
    std::make_pair<cl_int, std::string>(CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST,
                                        "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST"),
    std::make_pair<cl_int, std::string>(CL_COMPILE_PROGRAM_FAILURE, "CL_COMPILE_PROGRAM_FAILURE"),
    std::make_pair<cl_int, std::string>(CL_LINKER_NOT_AVAILABLE, "CL_LINKER_NOT_AVAILABLE"),
    std::make_pair<cl_int, std::string>(CL_LINK_PROGRAM_FAILURE, "CL_LINK_PROGRAM_FAILURE"),
    std::make_pair<cl_int, std::string>(CL_DEVICE_PARTITION_FAILED, "CL_DEVICE_PARTITION_FAILED"),
    std::make_pair<cl_int, std::string>(CL_KERNEL_ARG_INFO_NOT_AVAILABLE,
                                        "CL_KERNEL_ARG_INFO_NOT_AVAILABLE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_VALUE, "CL_INVALID_VALUE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_DEVICE_TYPE, "CL_INVALID_DEVICE_TYPE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_PLATFORM, "CL_INVALID_PLATFORM"),
    std::make_pair<cl_int, std::string>(CL_INVALID_DEVICE, "CL_INVALID_DEVICE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_CONTEXT, "CL_INVALID_CONTEXT"),
    std::make_pair<cl_int, std::string>(CL_INVALID_QUEUE_PROPERTIES, "CL_INVALID_QUEUE_PROPERTIES"),
    std::make_pair<cl_int, std::string>(CL_INVALID_COMMAND_QUEUE, "CL_INVALID_COMMAND_QUEUE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_HOST_PTR, "CL_INVALID_HOST_PTR"),
    std::make_pair<cl_int, std::string>(CL_INVALID_MEM_OBJECT, "CL_INVALID_MEM_OBJECT"),
    std::make_pair<cl_int, std::string>(CL_INVALID_IMAGE_FORMAT_DESCRIPTOR,
                                        "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR"),
    std::make_pair<cl_int, std::string>(CL_INVALID_IMAGE_SIZE, "CL_INVALID_IMAGE_SIZE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_SAMPLER, "CL_INVALID_SAMPLER"),
    std::make_pair<cl_int, std::string>(CL_INVALID_BINARY, "CL_INVALID_BINARY"),
    std::make_pair<cl_int, std::string>(CL_INVALID_BUILD_OPTIONS, "CL_INVALID_BUILD_OPTIONS"),
    std::make_pair<cl_int, std::string>(CL_INVALID_PROGRAM, "CL_INVALID_PROGRAM"),
    std::make_pair<cl_int, std::string>(CL_INVALID_PROGRAM_EXECUTABLE,
                                        "CL_INVALID_PROGRAM_EXECUTABLE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_KERNEL_NAME, "CL_INVALID_KERNEL_NAME"),
    std::make_pair<cl_int, std::string>(CL_INVALID_KERNEL_DEFINITION,
                                        "CL_INVALID_KERNEL_DEFINITION"),
    std::make_pair<cl_int, std::string>(CL_INVALID_KERNEL, "CL_INVALID_KERNEL"),
    std::make_pair<cl_int, std::string>(CL_INVALID_ARG_INDEX, "CL_INVALID_ARG_INDEX"),
    std::make_pair<cl_int, std::string>(CL_INVALID_ARG_VALUE, "CL_INVALID_ARG_VALUE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_ARG_SIZE, "CL_INVALID_ARG_SIZE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_KERNEL_ARGS, "CL_INVALID_KERNEL_ARGS"),
    std::make_pair<cl_int, std::string>(CL_INVALID_WORK_DIMENSION, "CL_INVALID_WORK_DIMENSION"),
    std::make_pair<cl_int, std::string>(CL_INVALID_WORK_GROUP_SIZE, "CL_INVALID_WORK_GROUP_SIZE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_WORK_ITEM_SIZE, "CL_INVALID_WORK_ITEM_SIZE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_GLOBAL_OFFSET, "CL_INVALID_GLOBAL_OFFSET"),
    std::make_pair<cl_int, std::string>(CL_INVALID_EVENT_WAIT_LIST, "CL_INVALID_EVENT_WAIT_LIST"),
    std::make_pair<cl_int, std::string>(CL_INVALID_EVENT, "CL_INVALID_EVENT"),
    std::make_pair<cl_int, std::string>(CL_INVALID_OPERATION, "CL_INVALID_OPERATION"),
    std::make_pair<cl_int, std::string>(CL_INVALID_GL_OBJECT, "CL_INVALID_GL_OBJECT"),
    std::make_pair<cl_int, std::string>(CL_INVALID_BUFFER_SIZE, "CL_INVALID_BUFFER_SIZE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_MIP_LEVEL, "CL_INVALID_MIP_LEVEL"),
    std::make_pair<cl_int, std::string>(CL_INVALID_GLOBAL_WORK_SIZE, "CL_INVALID_GLOBAL_WORK_SIZE"),
    std::make_pair<cl_int, std::string>(CL_INVALID_PROPERTY, "CL_INVALID_PROPERTY"),
    std::make_pair<cl_int, std::string>(CL_INVALID_IMAGE_DESCRIPTOR, "CL_INVALID_IMAGE_DESCRIPTOR"),
    std::make_pair<cl_int, std::string>(CL_INVALID_COMPILER_OPTIONS, "CL_INVALID_COMPILER_OPTIONS"),
    std::make_pair<cl_int, std::string>(CL_INVALID_LINKER_OPTIONS, "CL_INVALID_LINKER_OPTIONS"),
    std::make_pair<cl_int, std::string>(CL_INVALID_DEVICE_PARTITION_COUNT,
                                        "CL_INVALID_DEVICE_PARTITION_COUNT"),
};

}  // namespace OpenCLUtils
}  // namespace scrimmage
