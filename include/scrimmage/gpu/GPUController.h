#include <CL/opencl.hpp>

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#ifndef INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H
#define INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H

namespace scrimmage {

class GPUController {
    public:
        GPUController();
        ~GPUController();
        bool init(const std::string& kernel_directory);
        bool init(std::filesystem::path kernel_directory);

    private:
        struct KernelSource {
            KernelSource(std::string name, std::string src, std::size_t size);
            std::string name_;
            std::string src_;
            std::size_t size_;
        };

        static constexpr char OPENCL_COMPILER_OPTIONS[] = "";

        std::filesystem::path kernel_directory_; 
        cl::Platform platform_;
        // Device Vendor+Name -> Device
        std::vector<cl::Device> devices_;
        cl::Context context_;
        cl::Program program_;
        // Kernel Name -> Kernel
        std::map<std::string, cl::Kernel> kernels_;

        cl::CommandQueue queue_;


        static std::string get_device_name(cl::Device& device);
        bool check_error(cl_int err, const std::string&& msg) const;
        bool build_kernels();
        std::vector<KernelSource> find_kernel_sources() const;
};

}

#endif // INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H
