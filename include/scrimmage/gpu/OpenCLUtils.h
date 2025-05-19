#ifndef INCLUDE_SCRIMMAGE_GPU_OPENCLUTILS_H
#define INCLUDE_SCRIMMAGE_GPU_OPENCLUTILS_H

#include <scrimmage/gpu/GPUController.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <map>
#include <optional>

namespace scrimmage {
namespace OpenCLUtils {

#if ENABLE_GPU_ACCELERATION == 1
//  std::shared_ptr<GPUMotionModel> make_gpu_motion_model(
//      const cl::Kernel& kernel,
//      cl::CommandQueue& queue,
//      const KernelBuildOpts& opts);

bool supports_fp64(const cl::Device& device);

std::size_t prefered_workgroup_size_multiples(const cl::Kernel& kernel, const cl::Device& device);
std::size_t max_workgroup_size(const cl::Device& device);

#define CL_CHECK_ERROR(err, msg) OpenCLUtils::check_error(err, __FILE__, __LINE__, (msg));

bool check_error(cl_int err,
                 const std::string&& file,
                 const unsigned int linenumber,
                 const std::string&& msg);

bool check_error(cl_int err, const std::string&& msg);

std::optional<std::size_t> cacheline_size(const cl::Device& device);

extern const std::map<cl_int, std::string> CL_ERROR_MESSAGES;
#endif
}  // namespace OpenCLUtils
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_GPU_OPENCLUTILS_H
