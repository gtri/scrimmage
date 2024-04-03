#include <scrimmage/gpu/GPUController.h>

#include <CL/opencl.hpp>

#include <stdio.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace scrimmage {

    std::string GPUController::get_device_name(cl::Device& device) {
        std::string vendor = device.getInfo<CL_DEVICE_VENDOR>();
        std::string name = device.getInfo<CL_DEVICE_NAME>();
        return vendor + ":" + name;
    }

    GPUController::KernelSource::KernelSource(std::string name, std::string src, std::size_t size) :
        name_{name}, src_{src}, size_{size} {};

    GPUController::GPUController() {};

    GPUController::~GPUController() {
        queue_.finish();
        queue_.flush();
    }

    // Returns true when an error as occured
    bool GPUController::check_error(cl_int err, const std::string&& msg) const {
        if (err != 0) {
            fprintf(stderr, "OpenCL Error %d: %s\n", err, msg.c_str());
            return true;
        }
        return false;
    };

    bool GPUController::init(const std::string& kernel_directory) {
        return init(std::filesystem::path{kernel_directory});
    }

    // Returns true if GPU is successfully initalized
    bool GPUController::init(std::filesystem::path kernel_directory) {
        kernel_directory_ = kernel_directory;

        cl_int err;
        // Get default platform for now. 
        platform_ = cl::Platform::getDefault(&err); 
        if(check_error(err, "Error Retreiving Default Platform")) { return false; }

        platform_.getDevices(CL_DEVICE_TYPE_GPU, &devices_);
        
        context_ = cl::Context{devices_, nullptr, nullptr, nullptr, &err};
        if(check_error(err, "Error Initalizeing Context")) { return false; }

        bool build_kernels_success = build_kernels();
        if(!build_kernels_success) { return false; }
    
        queue_ = cl::CommandQueue(context_, 0, &err);
        if(check_error(err, "Error Initalizeing Command Queue")) { return false; }

        return true;
    }

    // Returns true if all kernels are built successfully
    bool GPUController::build_kernels() {
        using KernelSource = GPUController::KernelSource; 
        cl_int err;

        auto kernel_src_to_cl_src = [&](KernelSource kernel_src) {
            #if !defined(CL_HPP_ENABLE_PROGRAM_CONSTRUCTION_FROM_ARRAY_COMPATIBILITY)
                return kernel_src.src_;
            #else
                return std::make_pair<const char*, std::size_t>(
                        kernel_src.src_.c_str(), kernel_src.size_);
            #endif
        };
        std::vector<KernelSource> kernel_srcs = find_kernel_sources();
        cl::Program::Sources cl_srcs; 
        std::transform(kernel_srcs.cbegin(), kernel_srcs.cend(), 
                std::back_insert_iterator(cl_srcs), kernel_src_to_cl_src);

        program_ = cl::Program{context_, cl_srcs, &err};
        if(check_error(err, "Unable to initalize OpenCL Program")) { return false; }
        err = program_.build(devices_, OPENCL_COMPILER_OPTIONS, nullptr, nullptr);
        
        // Display Compiler Error messages, if any.
        if(err != 0) {
            std::vector<std::pair<cl::Device, std::string>> build_logs =
                program_.getBuildInfo<CL_PROGRAM_BUILD_LOG>();
            std::string build_log = "";
            for(auto device_build_log : build_logs) {
                build_log += "Build Log for " + get_device_name(device_build_log.first) + "\n"
                + device_build_log.second + "\n";
            }
            check_error(err, "Error Building Kernels: Output Build Log:\n" + build_log);
            return false;
        }
        for(KernelSource kernel_src : kernel_srcs) {
            cl::Kernel kernel = cl::Kernel{program_, kernel_src.name_.c_str(), &err};
            if(!check_error(err, "Unable to Initalize Kernel " + kernel_src.name_)) {
              printf("Successfully Initalized Kernel \'%s\'\n", kernel_src.name_.c_str());
              kernels_[kernel_src.name_] = kernel;
            }
        }
        // All Kernels Should be initalized now
        return true;
    } 

    std::vector<GPUController::KernelSource> GPUController::find_kernel_sources() const {
        using KernelSource = GPUController::KernelSource; 
        using std::filesystem::directory_entry, std::filesystem::directory_iterator,
              std::filesystem::path;

        std::vector<KernelSource> kernel_sources;

        std::error_code ec;
        directory_iterator kernel_dir_it{
            kernel_directory_,
            std::filesystem::directory_options::follow_directory_symlink,
            ec};

        if(ec) {
            fprintf(stderr, "Error Accessing Kernel Directory %s: %s\n", 
                    kernel_directory_.string().c_str(),
                    ec.message().c_str());
            return kernel_sources;
        }

        std::ifstream kernel_reader;
        std::stringstream buffer;
        path kernel_file_path;
        for(directory_entry kernel_file : kernel_dir_it) {
            kernel_file_path = kernel_file.path();
            if(!kernel_file.is_regular_file() || kernel_file_path.extension() != ".cl") {
                continue;
            }
            kernel_reader.open(kernel_file_path);            
            buffer << kernel_reader.rdbuf();
            kernel_reader.close();

            kernel_sources.emplace_back(
                kernel_file_path.stem(),
                buffer.str(),
                kernel_file.file_size());

            kernel_reader.clear();
            buffer.str("");
            buffer.clear();
        }

        return kernel_sources;
    }
}
