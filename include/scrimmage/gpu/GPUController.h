#ifndef INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H
#define INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H

#include <scrimmage/fwd_decl.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <filesystem>
#include <map>
#include <string>
#include <vector>
#include <optional>

namespace scrimmage {

  struct KernelBuildOpts {
    bool single_precision;
    std::vector<std::filesystem::path> include_dirs;
    std::vector<std::filesystem::path> src_dirs;
    std::vector<std::string> preferred_platforms;
  };

  class GPUController {
    public:
      GPUController();
#if ENABLE_GPU_ACCELERATION == 1
      std::optional<std::pair<cl::Kernel, cl::CommandQueue>> build_kernel(
          const std::string& kernel_name,
          const std::vector<std::filesystem::path>& kernel_srcs, 
          const KernelBuildOpts& opts);

      KernelBuildOpts get_opts(const std::map<std::string, std::string>& overrides);

      std::map<std::string, GPUMotionModelPtr> build_motion_models(MissionParsePtr mp);

      std::optional<std::pair<cl::Kernel, cl::CommandQueue>> build_kernel(
          const std::string& kernel_name,
          const KernelBuildOpts& opts);

    private:
      struct KernelSource {
        KernelSource(std::string name, std::string src);
        std::string name_;
        std::string src_;

        std::size_t size() {
          return src_.size();
        }
      };

      // Kernel Name -> Kernel
      static std::string get_device_name(cl::Device& device);
      bool build_kernels();
      void set_kernel_sources();
      void set_compiler_options();

      KernelBuildOpts get_kernel_build_opts(const std::map<std::string, std::string>& attributes) const;

      cl::Program::Sources read_kernels(const std::vector<std::filesystem::path>& kernel_src_dirs);

      std::optional<cl::Device> pick_device(const KernelBuildOpts& opts);

      bool build_kernel(std::vector<KernelSource>, KernelBuildOpts opts);

      std::filesystem::path kernel_dir_;
      static constexpr char KERNEL_PATH_ENV_VAR[] = "SCRIMMAGE_KERNEL_PATH";
#endif
  };
}

#endif // INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H
