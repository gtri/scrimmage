#include <scrimmage/common/FileSearch.h>
#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/gpu/GPUMotionModel.h>
#include <scrimmage/gpu/OpenCLUtils.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// Useful document of OpenCL CPP wrappers:
// https://registry.khronos.org/OpenCL/specs/opencl-cplusplus-1.2.pdf
namespace fs = std::filesystem;

namespace scrimmage {
#if ENABLE_GPU_ACCELERATION == 1
std::string GPUController::get_device_name(cl::Device& device) {
    std::string vendor = device.getInfo<CL_DEVICE_VENDOR>();
    std::string name = device.getInfo<CL_DEVICE_NAME>();
    return vendor + ":" + name;
}

GPUController::KernelSource::KernelSource()
    : name_{""},
      src_{""} {};

GPUController::KernelSource::KernelSource(const std::string& name, const std::string& src)
    : name_{name},
      src_{src} {};

std::size_t GPUController::KernelSource::Hash::operator()(const KernelSource& ks) const {
    return std::hash<std::string>{}(ks.src_);
}

bool GPUController::KernelSource::EqualTo::operator()(const KernelSource& lhs,
                                                      const KernelSource& rhs) const {
    return lhs.src_ == rhs.src_;
}

GPUController::GPUController() {
    // const char* kernel_dir_str = std::getenv(KERNEL_PATH_ENV_VAR);
    const char* kernel_dir_str = std::getenv(KERNEL_PATH_ENV_VAR);
    if (kernel_dir_str == nullptr) {
        std::cerr << "SCRIMMAGE_KERNEL_PATH is not defined. Did you source your environment"
                  << std::endl;
    } else {
        std::vector<std::string> kernel_dirs =
            scrimmage::str2container<std::vector<std::string>>(std::string{kernel_dir_str}, ":");
        std::transform(
            kernel_dirs.cbegin(),
            kernel_dirs.cend(),
            std::back_inserter(kernel_dirs_),
            [](const std::string& kernel_dir_path) { return fs::path{kernel_dir_path}; });
    }
}

KernelBuildOpts GPUController::get_kernel_build_opts(
    const std::map<std::string, std::string>& attributes) const {
    KernelBuildOpts opts;
    opts.single_precision = scrimmage::get<bool>("single_precision", attributes, false);

    std::string include_dirs_str{
        scrimmage::get<std::string>("include_dirs", attributes, "kernels")};
    std::string src_dirs_str{scrimmage::get<std::string>("src_dirs", attributes, "kernels")};
    std::string preferred_platforms_str{scrimmage::get<std::string>("platforms", attributes, "")};

    std::vector<std::string> include_dirs =
        scrimmage::str2container<std::vector<std::string>>(include_dirs_str, ",");
    std::vector<std::string> src_dirs =
        scrimmage::str2container<std::vector<std::string>>(src_dirs_str, ",");

    opts.preferred_platforms =
        scrimmage::str2container<std::vector<std::string>>(preferred_platforms_str, ",");

    auto to_path = [&](std::string filename,
                       const fs::path& kernel_dir) -> std::optional<fs::path> {
        filename = scrimmage::expand_user(filename);
        if (!fs::exists(filename) && !kernel_dir.empty()) {
            // The kernel_dir is not part of the recursive iteration below. Explicitly check if it
            // matches
            if (kernel_dir.stem() == filename) {
                return std::optional<fs::path>{kernel_dir};
            }
            fs::recursive_directory_iterator kernel_dir_it{kernel_dir};
            for (auto dir_entry : kernel_dir_it) {
                if (dir_entry.is_directory() && dir_entry.path().stem() == filename) {
                    return std::optional<fs::path>{dir_entry.path()};
                }
            }
        }
        return std::nullopt;
    };

    std::vector<std::optional<fs::path>> include_dir_opts;
    std::vector<std::optional<fs::path>> src_dir_opts;

    for (const fs::path& kernel_dir : kernel_dirs_) {
        std::transform(include_dirs.cbegin(),
                       include_dirs.cend(),
                       std::back_inserter(include_dir_opts),
                       [&](const std::string& filename) { return to_path(filename, kernel_dir); });
        std::transform(src_dirs.cbegin(),
                       src_dirs.cend(),
                       std::back_inserter(src_dir_opts),
                       [&](const std::string& filename) { return to_path(filename, kernel_dir); });
    }

    for (auto include_dir_opt : include_dir_opts) {
        if (include_dir_opt.has_value()) {
            opts.include_dirs.push_back(include_dir_opt.value());
        }
    }

    for (auto src_dir_opt : src_dir_opts) {
        if (src_dir_opt.has_value()) {
            opts.src_dirs.push_back(src_dir_opt.value());
        }
    }

    return opts;
}

cl::Program::Sources GPUController::read_kernels(const std::vector<fs::path>& kernel_src_dirs) {
    auto kernel_src_to_cl_src = [&](KernelSource kernel_src) {
#ifndef CL_HPP_ENABLE_PROGRAM_CONSTRUCTION_FROM_ARRAY_COMPATIBILITY
        return kernel_src.src_;
#else
        return std::pair<const char*, std::size_t>{kernel_src.src_.c_str(), kernel_src.size()};
#endif
    };

    cl::Program::Sources ret_srcs;
    std::unordered_set<KernelSource, KernelSource::Hash, KernelSource::EqualTo> kernel_srcs;
    for (auto kernel_dir : kernel_src_dirs) {
        fs::directory_iterator kernel_dir_it{kernel_dir};
        for (auto dir_entry : kernel_dir_it) {
            std::ifstream ifstream;
            std::stringstream buffer;
            fs::path path = dir_entry.path();
            if (path.extension() != ".cl") {
                continue;
            }
            ifstream.open(path);
            if (ifstream.fail()) {
                std::cerr << "Error reading kernel file \'" << path << "\'";
            } else {
                buffer << ifstream.rdbuf();
                std::string src = buffer.str();
                kernel_srcs.emplace(path.stem(), src);
            }
        }
    }

    std::transform(kernel_srcs.cbegin(),
                   kernel_srcs.cend(),
                   std::back_inserter(ret_srcs),
                   kernel_src_to_cl_src);
    return ret_srcs;
}

void GPUController::init(MissionParsePtr mp) {
    auto build_plugin = [&](const std::string& plugin_name,
                                const std::map<std::string, std::string>& attributes) mutable {
        KernelBuildOpts opts = get_kernel_build_opts(attributes);
        std::string kernel_name = scrimmage::get<std::string>("kernel_name", attributes, "");

        if (plugin_params_.count(kernel_name) == 0) {
            std::optional<std::pair<cl::Kernel, cl::CommandQueue>> kernel_queue_opt =
                build_kernel(kernel_name, opts);
            if (kernel_queue_opt.has_value()) {
                auto kernel_queue = kernel_queue_opt.value();
                plugin_params_.emplace(
                    plugin_name,
                    GPUPluginBuildParams{
                        opts.single_precision, kernel_queue.first, kernel_queue.second});
            }
        }
    };

    const std::list<std::string>& kernel_names = mp->kernel_names();
    for (const std::string& kernel_name : kernel_names) {
        if (mp->attributes().count(kernel_name) == 0) {
            std::cerr << "No Attributes for GPU Kernel \'" << kernel_name << "\'" << std::endl;
            continue;
        }
        const std::map<std::string, std::string>& plugin_attributes =
            mp->attributes().at(kernel_name);
        build_plugin(kernel_name, plugin_attributes);
    }
}

const std::map<std::string, GPUPluginBuildParams>& GPUController::get_plugin_params() const {
    return plugin_params_;
}

std::optional<std::pair<cl::Kernel, cl::CommandQueue>> GPUController::build_kernel(
    const std::string& kernel_name, const KernelBuildOpts& opts) {
    cl_int err;
    std::optional<cl::Device> device_opt = pick_device(opts);
    if (!device_opt.has_value()) {
        return std::nullopt;
    }
    cl::Device& device = device_opt.value();
    cl::Program::Sources cl_srcs;
    std::string compiler_opts;

    cl_srcs = read_kernels(opts.src_dirs);

    cl::Context context{device, nullptr, nullptr, nullptr, &err};
    if (OpenCLUtils::check_error(err, __FILE__, __LINE__, "Unable to Initalize OpenCL Context")) {
        return std::nullopt;
    }

    cl::Program program = cl::Program{context, cl_srcs, &err};
    if (OpenCLUtils::check_error(err, __FILE__, __LINE__, "Unable to initalize OpenCL Program")) {
        return std::nullopt;
    }

    if (opts.single_precision) {
        compiler_opts += " -D SINGLE_PRECISION";
    }
    for (const std::filesystem::path& include_dir : opts.include_dirs) {
        compiler_opts += " -I " + include_dir.string();
    }

    err = program.build(device, compiler_opts.c_str(), nullptr, nullptr);

    // Display Compiler Error messages, if any.
    if (err != 0) {
        std::vector<std::pair<cl::Device, std::string>> build_logs =
            program.getBuildInfo<CL_PROGRAM_BUILD_LOG>();
        std::string build_log = "";
        for (auto device_build_log : build_logs) {
            build_log += "Build Log for " + get_device_name(device_build_log.first) + "\n\n" +
                         device_build_log.second + "\n";
        }
        OpenCLUtils::check_error(
            err, __FILE__, __LINE__, "Error Building Kernels: Output Build Log:\n" + build_log);
        return std::nullopt;
    }
    cl::Kernel kernel{program, kernel_name.c_str(), &err};
    if (OpenCLUtils::check_error(
            err, __FILE__, __LINE__, "Error creating Kernel \'" + kernel_name + "\'")) {
        return std::nullopt;
    }

    cl::CommandQueue queue{context, device, 0, &err};
    if (OpenCLUtils::check_error(err, __FILE__, __LINE__, "Error creating command Queue")) {
        return std::nullopt;
    }

    return std::pair<cl::Kernel, cl::CommandQueue>{kernel, queue};
}

std::optional<cl::Device> GPUController::pick_device(const KernelBuildOpts& opts) {
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    std::map<std::string, std::vector<cl::Device>> device_map;

    // Choose devices that can support the precision we want
    for (cl::Platform& platform : platforms) {
        std::vector<cl::Device> devices;
        std::string platform_name = platform.getInfo<CL_PLATFORM_NAME>();
        platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
        device_map.try_emplace(platform_name);
        for (auto it = devices.begin(); it != devices.end(); ++it) {
            if (opts.single_precision || OpenCLUtils::supports_fp64(*it)) {
                device_map[platform_name].push_back(*it);
            }
        }
    }

    if (device_map.size() == 0 && !opts.single_precision) {
        std::cerr << "Warning: No Platform found that supports double precision. Try enabling "
                     "single precision."
                  << std::endl;
        return std::nullopt;
    }

    std::vector<cl::Device> usable_devices;
    if (opts.preferred_platforms.empty()) {
        for (auto platform_name_devices : device_map) {
            const std::vector<cl::Device>& platform_devices = platform_name_devices.second;
            if (!platform_devices.empty()) {
                usable_devices.insert(
                    usable_devices.end(), platform_devices.begin(), platform_devices.end());
            }
        }
    } else {
        for (std::string preferred_platform : opts.preferred_platforms) {
            if (device_map.count(preferred_platform) == 0) {
                std::cerr << "Warning: No Platform named " << preferred_platform << " found.\n";
            } else {
                std::vector<cl::Device>& platform_devices = device_map[preferred_platform];
                usable_devices.insert(
                    usable_devices.end(), platform_devices.begin(), platform_devices.end());
            }
        }
    }

    if (usable_devices.empty()) {
        return std::nullopt;
    }

    using ClockFreqDevice = std::pair<cl_uint, cl::Device>;
    std::vector<std::pair<cl_uint, cl::Device>> clock_freq_devices;

    std::transform(
        usable_devices.begin(),
        usable_devices.end(),
        std::back_inserter(clock_freq_devices),
        [](cl::Device& device) {
            cl_int err;
            cl_uint max_clock_freq = device.getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>(&err);
            if (OpenCLUtils::check_error(
                    err, __FILE__, __LINE__, "Error querying device MAX_CLOCK_FREQUENCY")) {
                max_clock_freq =
                    std::numeric_limits<cl_uint>::min();  // Still need to return something,
                                                          // but will not picked as the device
                                                          // with the highest clock frequency
            }
            return std::pair{max_clock_freq, device};
        });

    auto max_it = std::max_element(clock_freq_devices.cbegin(),
                                   clock_freq_devices.cend(),
                                   [](const ClockFreqDevice& rhs, const ClockFreqDevice& lhs) {
                                       return rhs.first < lhs.first;
                                   });
    if (max_it == clock_freq_devices.end()) {
        return std::nullopt;
    }
    return std::make_optional(max_it->second);
}

#endif
}  // namespace scrimmage
