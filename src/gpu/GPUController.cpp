#include <scrimmage/gpu/GPUController.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <stdio.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Useful document of OpenCL CPP wrappers: https://registry.khronos.org/OpenCL/specs/opencl-cplusplus-1.2.pdf

namespace scrimmage {
#if ENABLE_GPU_ACCELERATION == 1
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
  bool GPUController::check_error(cl_int& err, const std::string&& msg) {
    if (err != 0) {
      fprintf(stderr, "OpenCL Error: %s (CODE: %d)\n--%s\n",
          GPUController::CL_ERROR_MESSAGES.at(err).c_str(), err, msg.c_str());
      err = 0;
      return true;
    }
    return false;
  };

  bool GPUController::init() {
    // Empty initalization. Used for testing when we don't need 
    // any kernels, but still want access to a device
    cl_int err;
    std::vector<cl::Platform> platforms;

    err = cl::Platform::get(&platforms);
    if(check_error(err, "Error Retreiving System Platforms")) { return false; }

    // For this purpose, just pick the first platform that we can get a command queue from

    for (cl::Platform platform : platforms) {
      platform_ = platform;
      platform.getDevices(CL_DEVICE_TYPE_GPU, &devices_);
      std::string platform_name = platform.getInfo<CL_PLATFORM_NAME>();
      if (devices_.size() == 0) {
        std::cerr << "Platform \'" + platform_name + "\' does not have any devices!\n";
        continue;
      }
      context_ = cl::Context{devices_, nullptr, nullptr, nullptr, &err};
      if(check_error(err, "Error Initalizeing Context for platform \'" +
            platform_name + "\'")) { continue; }
      queue_ = cl::CommandQueue(context_, 0, &err);
      if(check_error(err, "Error Initalizeing Command Queue for platform \'" + 
            platform_name + "\'")) { continue; }

      return true;
    }
    return false;

  }

  bool GPUController::init(const std::string& kernel_directory) {
    return init(std::filesystem::path{kernel_directory});
  }

  // Returns true if GPU is successfully initalized
  bool GPUController::init(std::filesystem::path kernel_directory) {
    cl_int err;
    std::vector<cl::Platform> platforms;
    bool successfull_init = false;

    kernel_directory_ = kernel_directory;
    err = cl::Platform::get(&platforms); 
    if(check_error(err, "Error Retreiving System Platforms")) { return false; }

    // Cycle through platforms and select the first one our kernel
    // compiles on.
    for (cl::Platform& platform : platforms) {
      std::string platform_name = platform.getInfo<CL_PLATFORM_NAME>();
      std::cout << "Compiling for platform \'" + platform_name + "\'\n";

      set_kernel_sources();
      set_compiler_options();

      err = 0;
      platform_ = platform;
      devices_.clear();
      platform.getDevices(CL_DEVICE_TYPE_GPU, &devices_);
      if (devices_.size() == 0) {
        std::cerr << "Platform \'" + platform_name + "\' does not have any devices!\n";
        continue;
      }

      context_ = cl::Context{devices_, nullptr, nullptr, nullptr, &err};
      if(check_error(err, "Error Initalizeing Context for platform \'" +
            platform_name + "\'")) { continue; }

      bool build_kernels_success = build_kernels();
      if(!build_kernels_success) { continue; }

      // This creats a command queue for the first device in the context.
      // If there are more devices in the context, we could specalize this later
      queue_ = cl::CommandQueue(context_, 0, &err);
      if(check_error(err, "Error Initalizeing Command Queue for platform \'" + 
            platform_name + "\'")) { continue; }

      successfull_init = true;
      break;
    }

    if(successfull_init) {
      std::cout << "Successfully initalized GPU on platform \'" + 
        platform_.getInfo<CL_PLATFORM_NAME>() + "\' and compiled kernels\n"; 
    } else {
      std::cerr  << "Unable to successfully initalize any GPU's on the " <<
        "device or compile kernels successfully\n";
    }
    return successfull_init;
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
    cl::Program::Sources cl_srcs; 
    std::transform(kernel_sources_.cbegin(), kernel_sources_.cend(), 
        std::back_insert_iterator(cl_srcs), kernel_src_to_cl_src);

    program_ = cl::Program{context_, cl_srcs, &err};
    if(check_error(err, "Unable to initalize OpenCL Program")) { return false; }
    err = program_.build(devices_, opencl_compiler_options_.c_str(), nullptr, nullptr);

    // Display Compiler Error messages, if any.
    if(err != 0) {
      std::vector<std::pair<cl::Device, std::string>> build_logs =
        program_.getBuildInfo<CL_PROGRAM_BUILD_LOG>();
      std::string build_log = "";
      for(auto device_build_log : build_logs) {
        build_log += "Build Log for " + get_device_name(device_build_log.first) + "\n\n"
          + device_build_log.second + "\n";
      }
      check_error(err, "Error Building Kernels: Output Build Log:\n" + build_log);
      return false;
    }
    for(KernelSource kernel_src : kernel_sources_) {
      cl::Kernel kernel = cl::Kernel{program_, kernel_src.name_.c_str(), &err};
      if(!check_error(err, "Unable to Initalize Kernel " + kernel_src.name_)) {
        printf("Successfully Initalized Kernel \'%s\'\n", kernel_src.name_.c_str());
        kernels_[kernel_src.name_] = kernel;
      }
    }
    // All Kernels Should be initalized now
    return true;
  } 

  void GPUController::set_kernel_sources() {
    using KernelSource = GPUController::KernelSource; 
    using std::filesystem::directory_entry, 
          std::filesystem::recursive_directory_iterator,
          std::filesystem::path;

    std::error_code ec;
    recursive_directory_iterator kernel_dir_it{
      kernel_directory_,
        std::filesystem::directory_options::follow_directory_symlink,
        ec};

    if(ec) {
      fprintf(stderr, "Error Accessing Kernel Directory %s: %s\n", 
          kernel_directory_.string().c_str(),
          ec.message().c_str());
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

      kernel_sources_.emplace_back(
          kernel_file_path.stem(),
          buffer.str(),
          kernel_file.file_size());

      kernel_reader.clear();
      buffer.str("");
      buffer.clear();
    }
  }

  void GPUController::set_compiler_options() {
    std::string src_include_dir = kernel_directory_.string();
    opencl_compiler_options_ += " -I " + src_include_dir + " ";
  }

  // * Gets size of cacheline if exitsts. Otherwise return nullopt;
  std::optional<std::size_t> GPUController::get_cacheline_size() const {
    if (devices_.size() == 0) {
      std::cerr << "Error. Trying to get cacheline size, but GPUController is not aware of "
        "any devices on the platfrom\n";
      return std::nullopt;
    } 
    const cl::Device& device = devices_[0];
    if (device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHE_TYPE>() == CL_NONE) {
      return std::nullopt;
    }
    return std::make_optional(device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>());
  }
#endif
}
