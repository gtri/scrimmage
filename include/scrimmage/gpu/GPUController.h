#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <filesystem>
#include <map>
#include <string>
#include <vector>
#include <optional>

#ifndef INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H
#define INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H

namespace scrimmage {

class GPUController {
    public:
        GPUController();
        ~GPUController();
#if ENABLE_GPU_ACCELERATION == 1
        bool init(const std::string& kernel_directory);
        bool init(std::filesystem::path kernel_directory);
        bool init();

        const cl::Context& context() const { return context_; }
        const cl::CommandQueue& queue() const { return queue_; }
        const cl::Device& device() const { return devices_[0]; }
        std::optional<cl::Kernel> get_kernel(const std::string& name) {
          if (kernels_.count(name) == 0) { return std::nullopt; }
          return std::make_optional<cl::Kernel>(kernels_[name]);
        }

        std::optional<std::size_t> get_cacheline_size() const;

        static bool check_error(cl_int& err, const std::string&& msg);

    private:
        struct KernelSource {
            KernelSource(std::string name, std::string src, std::size_t size);
            std::string name_;
            std::string src_;
            std::size_t size_;
        };

        std::string opencl_compiler_options_;

        std::filesystem::path kernel_directory_; 
        cl::Platform platform_;
        // Device Vendor+Name -> Device
        std::vector<cl::Device> devices_;
        cl::Context context_;
        cl::Program program_;
        
        std::vector<KernelSource> kernel_sources_;
       
        
        // Kernel Name -> Kernel
        std::map<std::string, cl::Kernel> kernels_;

        cl::CommandQueue queue_;

        static std::string get_device_name(cl::Device& device);
        bool build_kernels();
        void set_kernel_sources();
        void set_compiler_options();

        inline static const std::map<cl_int, std::string> CL_ERROR_MESSAGES = {
          std::make_pair<cl_int, std::string>(CL_SUCCESS, "CL_SUCCESS"),
          std::make_pair<cl_int, std::string>(CL_DEVICE_NOT_FOUND, "CL_DEVICE_NOT_FOUND"),
          std::make_pair<cl_int, std::string>(CL_DEVICE_NOT_AVAILABLE, "CL_DEVICE_NOT_AVAILABLE"),
          std::make_pair<cl_int, std::string>(CL_COMPILER_NOT_AVAILABLE, "CL_COMPILER_NOT_AVAILABLE"),
          std::make_pair<cl_int, std::string>(CL_MEM_OBJECT_ALLOCATION_FAILURE, "CL_MEM_OBJECT_ALLOCATION_FAILURE"),
          std::make_pair<cl_int, std::string>(CL_OUT_OF_RESOURCES, "CL_OUT_OF_RESOURCES"),
          std::make_pair<cl_int, std::string>(CL_OUT_OF_HOST_MEMORY, "CL_OUT_OF_HOST_MEMORY"),
          std::make_pair<cl_int, std::string>(CL_PROFILING_INFO_NOT_AVAILABLE, "CL_PROFILING_INFO_NOT_AVAILABLE"),
          std::make_pair<cl_int, std::string>(CL_MEM_COPY_OVERLAP, "CL_MEM_COPY_OVERLAP"),
          std::make_pair<cl_int, std::string>(CL_IMAGE_FORMAT_MISMATCH, "CL_IMAGE_FORMAT_MISMATCH"),
          std::make_pair<cl_int, std::string>(CL_IMAGE_FORMAT_NOT_SUPPORTED, "CL_IMAGE_FORMAT_NOT_SUPPORTED"),
          std::make_pair<cl_int, std::string>(CL_BUILD_PROGRAM_FAILURE, "CL_BUILD_PROGRAM_FAILURE"),
          std::make_pair<cl_int, std::string>(CL_MAP_FAILURE, "CL_MAP_FAILURE"),
          std::make_pair<cl_int, std::string>(CL_MISALIGNED_SUB_BUFFER_OFFSET, "CL_MISALIGNED_SUB_BUFFER_OFFSET"),
          std::make_pair<cl_int, std::string>(CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST, "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST"),
          std::make_pair<cl_int, std::string>(CL_COMPILE_PROGRAM_FAILURE, "CL_COMPILE_PROGRAM_FAILURE"),
          std::make_pair<cl_int, std::string>(CL_LINKER_NOT_AVAILABLE, "CL_LINKER_NOT_AVAILABLE"),
          std::make_pair<cl_int, std::string>(CL_LINK_PROGRAM_FAILURE, "CL_LINK_PROGRAM_FAILURE"),
          std::make_pair<cl_int, std::string>(CL_DEVICE_PARTITION_FAILED, "CL_DEVICE_PARTITION_FAILED"),
          std::make_pair<cl_int, std::string>(CL_KERNEL_ARG_INFO_NOT_AVAILABLE, "CL_KERNEL_ARG_INFO_NOT_AVAILABLE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_VALUE, "CL_INVALID_VALUE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_DEVICE_TYPE, "CL_INVALID_DEVICE_TYPE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_PLATFORM, "CL_INVALID_PLATFORM"),
          std::make_pair<cl_int, std::string>(CL_INVALID_DEVICE, "CL_INVALID_DEVICE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_CONTEXT, "CL_INVALID_CONTEXT"),
          std::make_pair<cl_int, std::string>(CL_INVALID_QUEUE_PROPERTIES, "CL_INVALID_QUEUE_PROPERTIES"),
          std::make_pair<cl_int, std::string>(CL_INVALID_COMMAND_QUEUE, "CL_INVALID_COMMAND_QUEUE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_HOST_PTR, "CL_INVALID_HOST_PTR"),
          std::make_pair<cl_int, std::string>(CL_INVALID_MEM_OBJECT, "CL_INVALID_MEM_OBJECT"),
          std::make_pair<cl_int, std::string>(CL_INVALID_IMAGE_FORMAT_DESCRIPTOR, "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR"),
          std::make_pair<cl_int, std::string>(CL_INVALID_IMAGE_SIZE, "CL_INVALID_IMAGE_SIZE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_SAMPLER, "CL_INVALID_SAMPLER"),
          std::make_pair<cl_int, std::string>(CL_INVALID_BINARY, "CL_INVALID_BINARY"),
          std::make_pair<cl_int, std::string>(CL_INVALID_BUILD_OPTIONS, "CL_INVALID_BUILD_OPTIONS"),
          std::make_pair<cl_int, std::string>(CL_INVALID_PROGRAM, "CL_INVALID_PROGRAM"),
          std::make_pair<cl_int, std::string>(CL_INVALID_PROGRAM_EXECUTABLE, "CL_INVALID_PROGRAM_EXECUTABLE"),
          std::make_pair<cl_int, std::string>(CL_INVALID_KERNEL_NAME, "CL_INVALID_KERNEL_NAME"),
          std::make_pair<cl_int, std::string>(CL_INVALID_KERNEL_DEFINITION, "CL_INVALID_KERNEL_DEFINITION"),
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
          std::make_pair<cl_int, std::string>(CL_INVALID_DEVICE_PARTITION_COUNT, "CL_INVALID_DEVICE_PARTITION_COUNT"),
        };
#endif
};
}

#endif // INCLUDE_SCRIMMAGE_GPU_GPUCONTROLLER_H
