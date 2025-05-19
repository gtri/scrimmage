#include <CL/opencl.hpp>

#include <unistd.h>

#include <filesystem>
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>



std::map<cl_int, std::string> CL_ERROR_MESSAGES = {
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

bool check_error(cl_int& err) {
  if(err != CL_SUCCESS) {
    std::cerr << "Error Code " << err << ": " << CL_ERROR_MESSAGES[err] << std::endl;
    err = 0;
    return true;
  }
  return false;
}

std::string parse_kernel_source(const std::string& kernel_path) {
  std::ifstream kernel_file;
  std::stringstream buffer;
  std::string kernel_src;

  kernel_file.open(kernel_path);
  buffer << kernel_file.rdbuf();
  kernel_src = buffer.str();
  kernel_file.close();

  return kernel_src;
}

std::vector<std::string> target_platforms;

void show_usage(const char* program_name) {
  fprintf(stderr, "Usage: %s [-p TARGET_PLATFORM] -- PATH/TO/KERNEL [OpenCL Compiler Arguments]\n\n", program_name);
}

void parse_args(int argc, char* argv[]) {
  int opt;
  if(argc < 2) {
    fprintf(stderr, "No Paramter supplied.\n\n");
    show_usage(argv[0]);
    exit(EXIT_FAILURE); 
  }
  while ((opt = getopt(argc, argv, "hp:")) != -1) {
    switch (opt) {
      case 'p':
        target_platforms.emplace_back(optarg); 
        break;
      case 'h':
        show_usage(argv[0]);
        exit(EXIT_SUCCESS); 
      default:
        fprintf(stderr, "%c is not a valid option\n", opt); 
        exit(EXIT_FAILURE); 
    }
  }
}

int main(int argc, char* argv[]) {
  parse_args(argc, argv);
  std::size_t param_start = optind;
  std::filesystem::path kernel_path{argv[param_start]};
  cl_int err;
  std::vector<cl::Device> devices;
  cl::Context context;
  cl::Program::Sources cl_srcs;
  std::string kernel_src;

  //std::string opencl_compiler_options = " -I " + kernel_path.parent_path().string() + " ";
  std::string opencl_compiler_options = "";
  for(int i = param_start + 1; i < argc; i++) {
    opencl_compiler_options += argv[i];
    opencl_compiler_options += " ";
  }

  std::vector<cl::Platform> platforms;
  err = cl::Platform::get(&platforms);
  if(check_error(err)) { return -1; }
  
  std::map<std::string, cl::Platform> platform_map;
  for(cl::Platform platform : platforms) {
    platform_map.emplace(std::make_pair(platform.getInfo<CL_PLATFORM_NAME>(), platform));
     
  }
  if (target_platforms.size() == 0) {
    for (auto platform_pair : platform_map) {
      target_platforms.push_back(platform_pair.first);
    }
  }
  for(std::string target_platform : target_platforms) {
    if(platform_map.count(target_platform) == 0) {
      std::cerr << "No platform named \'" << target_platform << "\' Found. ";
      if (platform_map.size() == 0) {
        std::cerr << "No Platforms found!" << std::endl; 
        exit(EXIT_FAILURE);
      } else {
        std::cerr << "Available Platforms are: \n";
        for(auto platform_pair : platform_map) {
          std::cerr << "    " << platform_pair.first << "\n";
        }
        exit(EXIT_FAILURE);
      }
    }
    cl::Platform platform = platform_map[target_platform];
    std::string title_string = " Checking program compilation for platform: \'" + target_platform + "\' ";
    std::string sep_string(title_string.size(), '-');
    std::cout << "\n" << sep_string  << "\n" << 
      title_string << "\n" << 
      sep_string << "\n";

    err = platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
    if(check_error(err)) { return -1; }
    // Also check that a device exits. 
    if( devices.size() == 0 ) {
      std::cout << "No devices found for platform \'" << target_platform << "\'!\n";
      continue;
    }

    context = cl::Context{devices, nullptr, nullptr, nullptr, &err};
    if(check_error(err)) { return -1; }

    kernel_src = parse_kernel_source(kernel_path.string());

    cl::Program program = cl::Program{context, kernel_src, false, &err};
    if(check_error(err)) { return -1; }

    err = program.build(devices, opencl_compiler_options.c_str(), nullptr, nullptr);

    if(check_error(err)) {
      std::vector<std::pair<cl::Device, std::string>> build_logs = 
        program.getBuildInfo<CL_PROGRAM_BUILD_LOG>();

      for(auto device_build_log : build_logs) {
        std::string build_log = device_build_log.second;
        std::cout << "Unable to build kernel " << kernel_path << 
          " for platform \'" << target_platform << "\'\n" << std::endl; 
        std::string build_log_title_str = "Build Log";
        std::string spacer((title_string.size() - build_log_title_str.size()) / 2, '-');

        std::cout << spacer + build_log_title_str + spacer << "\n" << build_log << std::endl;
      }
    } else {
      std::cout << kernel_path << " compiled succesfully on platform \'" << 
        target_platform << "\'" << std::endl;
    }
  }
  return 0;
}

