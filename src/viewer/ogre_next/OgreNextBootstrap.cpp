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
 */

#include "OgreNextBootstrap.h"

#include <cstdlib>
#include <csignal>
#include <cstring>
#include <cerrno>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <OGRE-Next/OgreArchiveManager.h>
#include <OGRE-Next/OgreCamera.h>
#include <OGRE-Next/OgreColourValue.h>
#include <OGRE-Next/OgreException.h>
#include <OGRE-Next/OgreHlmsManager.h>
#include <OGRE-Next/OgreLight.h>
#include <OGRE-Next/OgreRoot.h>
#include <OGRE-Next/OgreSceneManager.h>
#include <OGRE-Next/OgreSceneNode.h>
#include <OGRE-Next/OgreVector3.h>
#include <OGRE-Next/OgreWireAabb.h>
#include <OGRE-Next/OgreWindow.h>
#include <OGRE-Next/OgreWindowEventUtilities.h>
#include <OGRE-Next/Compositor/OgreCompositorManager2.h>
#include <OGRE-Next/Compositor/OgreCompositorNodeDef.h>
#include <OGRE-Next/Compositor/OgreCompositorWorkspaceDef.h>
#include <OGRE-Next/Compositor/Pass/OgreCompositorPassDef.h>
#include <OGRE-Next/Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h>
#include <OGRE-Next/Hlms/Pbs/OgreHlmsPbs.h>
#include <OGRE-Next/Hlms/Unlit/OgreHlmsUnlit.h>
#include <OGRE-Next/Hlms/Unlit/OgreHlmsUnlitDatablock.h>
#include <OGRE-Next/Math/Simple/C/OgreAabb.h>

#include "scrimmage/log/Logger.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"

#ifndef SCRIMMAGE_OGRE_NEXT_PLUGIN_PATH
#define SCRIMMAGE_OGRE_NEXT_PLUGIN_PATH "RenderSystem_GL3Plus.so"
#endif

#ifndef SCRIMMAGE_OGRE_NEXT_HLMS_SOURCE_PATH
#define SCRIMMAGE_OGRE_NEXT_HLMS_SOURCE_PATH ""
#endif

#ifndef SCRIMMAGE_OGRE_NEXT_HLMS_INSTALL_PATH
#define SCRIMMAGE_OGRE_NEXT_HLMS_INSTALL_PATH "/usr/local/share/scrimmage/ogre-next/Hlms"
#endif

namespace scrimmage {

namespace {

namespace fs = std::filesystem;

class VirtualDesktopSession {
 public:
    ~VirtualDesktopSession() {
        stop();
    }

    bool ensure_display() {
        if (has_display()) {
            const std::string current_display = get_env_or_default("DISPLAY", "");
            if (display_responding(current_display)) {
                return true;
            }

            std::cout << "DISPLAY is set to '" << current_display
                      << "' but is not reachable from the container. "
                      << "Starting the container-local virtual desktop fallback."
                      << std::endl;
        }

        stop();
        capture_original_environment();

        const bool display_overridden = env_set("SCRIMMAGE_VIEWER_DISPLAY");
        const std::string requested_display =
            get_env_or_default("SCRIMMAGE_VIEWER_DISPLAY", ":99");
        const std::string display = display_overridden
            ? requested_display
            : find_available_display(requested_display);
        const std::string screen =
            get_env_or_default("SCRIMMAGE_VIEWER_SCREEN", "1920x1080x24");
        const std::string vnc_port =
            get_env_or_default("SCRIMMAGE_VIEWER_VNC_PORT", "5900");
        const std::string http_port =
            get_env_or_default("SCRIMMAGE_VIEWER_HTTP_PORT", "6080");

        if (!command_exists("Xvfb") ||
            !command_exists("openbox") ||
            !command_exists("x11vnc") ||
            !command_exists("websockify")) {
            std::cerr << "Error: no usable desktop display was found and the virtual desktop "
                      << "fallback is unavailable. Install Xvfb, openbox, x11vnc, and websockify, "
                      << "or provide DISPLAY." << std::endl;
            return false;
        }

        if (!path_exists("/usr/share/novnc")) {
            std::cerr << "Error: no usable desktop display was found and noVNC assets are missing. "
                      << "Install the 'novnc' package or provide DISPLAY." << std::endl;
            return false;
        }

        if (use_software_gl()) {
            setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
        }

        if (!spawn({"Xvfb", display, "-screen", "0", screen, "-ac", "+extension", "GLX",
                    "+render", "-noreset"})) {
            stop();
            return false;
        }

        if (!wait_for_display(display, 5000)) {
            std::cerr << "Error: timed out waiting for Xvfb on " << display << std::endl;
            stop();
            return false;
        }

        setenv("DISPLAY", display.c_str(), 1);

        if (!spawn({"openbox"})) {
            stop();
            return false;
        }

        if (!spawn({"x11vnc", "-display", display, "-rfbport", vnc_port, "-forever", "-shared",
                "-nopw", "-localhost", "-noxdamage"})) {
            stop();
            return false;
        }

        if (!spawn({"websockify", "--web", "/usr/share/novnc", http_port,
                    "127.0.0.1:" + vnc_port})) {
            stop();
            return false;
        }

        std::cout << "No desktop display detected. Started a container-local virtual desktop."
                  << std::endl;
        std::cout << "Open http://127.0.0.1:" << http_port
                  << "/vnc.html?autoconnect=1&resize=scale to view the Ogre window."
                  << std::endl;
        return true;
    }

 private:
    struct EnvironmentSnapshot {
        // Preserve the caller's environment so the fallback desktop does not
        // leave DISPLAY or software-rendering overrides behind after shutdown.
        std::optional<std::string> display;
        std::optional<std::string> software_gl;
    };

    std::vector<pid_t> child_pids_;
    std::optional<EnvironmentSnapshot> original_env_;

    static bool has_display() {
        const char* display = std::getenv("DISPLAY");
        return display != nullptr && display[0] != '\0';
    }

    static bool path_exists(const std::string& path) {
        struct stat info;
        return stat(path.c_str(), &info) == 0;
    }

    static std::string get_env_or_default(const std::string& key, const std::string& fallback) {
        const char* value = std::getenv(key.c_str());
        return (value != nullptr && value[0] != '\0') ? value : fallback;
    }

    static bool env_set(const std::string& key) {
        const char* value = std::getenv(key.c_str());
        return value != nullptr && value[0] != '\0';
    }

    static std::optional<std::string> read_env(const char* key) {
        const char* value = std::getenv(key);
        if (value == nullptr || value[0] == '\0') {
            return std::nullopt;
        }
        return std::string(value);
    }

    static void restore_env_value(const char* key, const std::optional<std::string>& value) {
        if (value.has_value()) {
            setenv(key, value->c_str(), 1);
        } else {
            unsetenv(key);
        }
    }

    static bool command_exists(const std::string& command) {
        const char* path_env = std::getenv("PATH");
        if (path_env == nullptr) {
            return false;
        }

        std::stringstream ss(path_env);
        std::string path;
        while (std::getline(ss, path, ':')) {
            if (path.empty()) {
                continue;
            }
            const std::string candidate = path + "/" + command;
            if (access(candidate.c_str(), X_OK) == 0) {
                return true;
            }
        }

        return false;
    }

    static bool wait_for_path(const std::string& path, int timeout_ms) {
        const int interval_ms = 100;
        for (int waited = 0; waited < timeout_ms; waited += interval_ms) {
            if (path_exists(path)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
        return path_exists(path);
    }

    static bool wait_for_display(const std::string& display, int timeout_ms) {
        const int interval_ms = 100;
        for (int waited = 0; waited < timeout_ms; waited += interval_ms) {
            if (display_responding(display)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
        return display_responding(display);
    }

    static bool display_responding(const std::string& display) {
        if (command_exists("xdpyinfo")) {
            const std::string command = "DISPLAY=" + display + " xdpyinfo >/dev/null 2>&1";
            return std::system(command.c_str()) == 0;
        }

        if (display.size() >= 2 && display[0] == ':') {
            return path_exists("/tmp/.X11-unix/X" + display.substr(1));
        }

        return false;
    }

    static bool display_in_use(const std::string& display) {
        if (display.size() < 2 || display[0] != ':') {
            return true;
        }

        const std::string display_num = display.substr(1);
        return path_exists("/tmp/.X11-unix/X" + display_num) ||
               path_exists("/tmp/.X" + display_num + "-lock");
    }

    static std::string find_available_display(const std::string& preferred_display) {
        if (preferred_display.size() >= 2 && preferred_display[0] == ':' &&
            !display_in_use(preferred_display)) {
            return preferred_display;
        }

        for (int display_num = 99; display_num <= 120; ++display_num) {
            const std::string candidate = ":" + std::to_string(display_num);
            if (!display_in_use(candidate)) {
                return candidate;
            }
        }

        return preferred_display;
    }

    static bool use_software_gl() {
        const char* force = std::getenv("SCRIMMAGE_VIEWER_SOFTWARE_GL");
        if (force != nullptr && force[0] != '\0') {
            return std::strcmp(force, "0") != 0;
        }
        return access("/dev/dri/renderD128", R_OK | W_OK) != 0;
    }

    void capture_original_environment() {
        // Capture once before we mutate the process environment for the
        // virtual desktop bootstrap.
        original_env_ = EnvironmentSnapshot{
            read_env("DISPLAY"),
            read_env("LIBGL_ALWAYS_SOFTWARE")};
    }

    void restore_environment() {
        if (!original_env_.has_value()) {
            return;
        }

        // Undo any DISPLAY / software-GL changes made by ensure_display().
        restore_env_value("DISPLAY", original_env_->display);
        restore_env_value("LIBGL_ALWAYS_SOFTWARE", original_env_->software_gl);
        original_env_.reset();
    }

    bool spawn(const std::vector<std::string>& args) {
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const std::string& arg : args) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);

        pid_t pid = fork();
        if (pid < 0) {
            std::cerr << "Error: fork failed while launching " << args.front() << std::endl;
            return false;
        }

        if (pid == 0) {
            // Make helpers die if scrimmage dies abruptly before we can run
            // the normal shutdown path.
            if (prctl(PR_SET_PDEATHSIG, SIGTERM) != 0) {
                _exit(127);
            }
            if (getppid() == 1) {
                _exit(127);
            }

            setsid();

            const std::string lower = to_lower(args.front());
            const std::string log_path = "/tmp/scrimmage-" + lower + ".log";
            int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
            if (fd >= 0) {
                dup2(fd, STDOUT_FILENO);
                dup2(fd, STDERR_FILENO);
                close(fd);
            }

            execvp(argv[0], argv.data());
            _exit(127);
        }

        child_pids_.push_back(pid);
        return true;
    }

    static bool wait_for_pid_exit(pid_t pid, int timeout_ms) {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeout_ms);

        while (std::chrono::steady_clock::now() < deadline) {
            int status = 0;
            const pid_t result = waitpid(pid, &status, WNOHANG);
            if (result == pid || (result < 0 && errno == ECHILD)) {
                return true;
            }
            if (result < 0 && errno != EINTR) {
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        return false;
    }

    static void terminate_process_group(pid_t pid, int signal_number) {
        if (pid <= 0) {
            return;
        }

        if (kill(-pid, signal_number) != 0 && errno != ESRCH) {
            kill(pid, signal_number);
        }
    }

    void stop() {
        // Shut helpers down in reverse startup order and restore the caller's
        // environment even when startup failed partway through.
        for (auto it = child_pids_.rbegin(); it != child_pids_.rend(); ++it) {
            terminate_process_group(*it, SIGTERM);
        }

        for (auto it = child_pids_.rbegin(); it != child_pids_.rend(); ++it) {
            if (!wait_for_pid_exit(*it, 2000)) {
                terminate_process_group(*it, SIGKILL);
                int status = 0;
                if (waitpid(*it, &status, 0) < 0 && errno != ECHILD) {
                    std::cerr << "Warning: failed waiting for viewer helper process "
                              << *it << " to exit." << std::endl;
                }
            }
        }

        child_pids_.clear();
        restore_environment();
    }

    static std::string to_lower(const std::string& input) {
        std::string out = input;
        for (char& ch : out) {
            if (ch >= 'A' && ch <= 'Z') {
                ch = static_cast<char>(ch - 'A' + 'a');
            }
        }
        return out;
    }
};

std::vector<double> parse_camera_vector(
    const std::map<std::string, std::string>& camera_params,
    const std::string& key,
    const std::string& default_value) {
    std::vector<double> values;
    const auto it = camera_params.find(key);
    const std::string vector_str = it == camera_params.end() ? default_value : it->second;
    if (!str2container(vector_str, ",", values, 3)) {
        values.clear();
    }
    return values;
}

Ogre::Vector3 to_ogre_vector(const std::vector<double>& values, const Ogre::Vector3& fallback) {
    if (values.size() != 3) {
        return fallback;
    }

    return Ogre::Vector3(values[0], values[1], values[2]);
}

std::vector<std::string> split_paths(const std::string& value, char delimiter) {
    std::vector<std::string> paths;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        if (!item.empty()) {
            paths.push_back(item);
        }
    }
    return paths;
}

bool is_hlms_root(const fs::path& root) {
    return fs::is_directory(root / "Common") &&
           fs::is_directory(root / "Pbs") &&
           fs::is_directory(root / "Unlit");
}

std::vector<fs::path> hlms_root_candidates() {
    std::vector<fs::path> candidates;

    const char* override_root = std::getenv("SCRIMMAGE_OGRE_NEXT_HLMS_PATH");
    if (override_root != nullptr && override_root[0] != '\0') {
        candidates.emplace_back(override_root);
    }

    const char* data_path = std::getenv("SCRIMMAGE_DATA_PATH");
    if (data_path != nullptr && data_path[0] != '\0') {
        for (const std::string& entry : split_paths(data_path, ':')) {
            candidates.emplace_back(fs::path(entry) / "ogre-next" / "Hlms");
        }
    }

    if (std::strlen(SCRIMMAGE_OGRE_NEXT_HLMS_SOURCE_PATH) > 0) {
        candidates.emplace_back(SCRIMMAGE_OGRE_NEXT_HLMS_SOURCE_PATH);
    }

    if (std::strlen(SCRIMMAGE_OGRE_NEXT_HLMS_INSTALL_PATH) > 0) {
        candidates.emplace_back(SCRIMMAGE_OGRE_NEXT_HLMS_INSTALL_PATH);
    }

    candidates.emplace_back("/usr/share/scrimmage/ogre-next/Hlms");
    candidates.emplace_back("/usr/local/share/scrimmage/ogre-next/Hlms");

    return candidates;
}

std::string find_hlms_root() {
    for (const fs::path& candidate : hlms_root_candidates()) {
        if (is_hlms_root(candidate)) {
            return candidate.string();
        }
    }
    return "";
}

struct HlmsArchiveSet {
    Ogre::Archive* data_folder = nullptr;
    Ogre::ArchiveVec library_folders;
};

bool load_hlms_archive_set(const std::string& data_folder,
                           const std::vector<std::string>& library_folders,
                           HlmsArchiveSet& archive_set) {
    Ogre::ArchiveManager& archive_manager = Ogre::ArchiveManager::getSingleton();
    archive_set.data_folder = archive_manager.load(data_folder, "FileSystem", true);
    if (archive_set.data_folder == nullptr) {
        return false;
    }

    for (const std::string& library_folder : library_folders) {
        archive_set.library_folders.push_back(
            archive_manager.load(library_folder, "FileSystem", true));
    }

    return true;
}

void unload_hlms_archive_set(HlmsArchiveSet& archive_set) {
    Ogre::ArchiveManager* archive_manager = Ogre::ArchiveManager::getSingletonPtr();
    if (archive_manager == nullptr) {
        archive_set.data_folder = nullptr;
        archive_set.library_folders.clear();
        return;
    }

    for (Ogre::Archive* archive : archive_set.library_folders) {
        if (archive != nullptr) {
            archive_manager->unload(archive);
        }
    }
    archive_set.library_folders.clear();

    if (archive_set.data_folder != nullptr) {
        archive_manager->unload(archive_set.data_folder);
        archive_set.data_folder = nullptr;
    }
}

}  // namespace

struct OgreNextBootstrap::Impl : public Ogre::WindowEventListener {
    std::unique_ptr<Ogre::Root> root;
    VirtualDesktopSession virtual_desktop;
    MissionParsePtr mission_parse;
    std::map<std::string, std::string> camera_params;

    Ogre::SceneManager* scene_manager = nullptr;
    Ogre::Camera* camera = nullptr;
    Ogre::Window* window = nullptr;
    Ogre::CompositorWorkspace* workspace = nullptr;
    Ogre::WireAabb* test_primitive_wire_aabb = nullptr;
    HlmsArchiveSet hlms_unlit_archives;
    HlmsArchiveSet hlms_pbs_archives;

    std::atomic<bool> running{false};
    bool listener_registered = false;
    bool render_system_plugin_loaded = false;

    const Ogre::String workspace_name = "ScrimmageOgreNextWorkspace";
    const Ogre::String test_primitive_datablock_name = "ScrimmageOgreNextTestPrimitiveMaterial";
    const Ogre::String render_system_plugin_path = SCRIMMAGE_OGRE_NEXT_PLUGIN_PATH;

    bool initialize_on_current_thread() {
        if (mission_parse == nullptr) {
            std::cerr << "Error: Ogre-Next bootstrap was not configured with a mission." << std::endl;
            return false;
        }

        root.reset(new Ogre::Root("", "", "scrimmage-ogre-next.log", "SCRIMMAGE"));
        root->loadPlugin(render_system_plugin_path, false, nullptr);
        render_system_plugin_loaded = true;

        Ogre::RenderSystem* render_system = select_render_system();
        if (render_system == nullptr) {
            std::cerr << "Error: no Ogre-Next render system is available." << std::endl;
            return false;
        }

        root->setRenderSystem(render_system);
        root->initialise(false);

        std::cout << "Ogre bootstrap: creating render window." << std::endl;

        window = root->createRenderWindow(
            "SCRIMMAGE Ogre-Next",
            mission_parse->window_width(),
            mission_parse->window_height(),
            mission_parse->full_screen());

        if (window == nullptr) {
            std::cerr << "Error: failed to create the Ogre-Next render window." << std::endl;
            return false;
        }

        Ogre::WindowEventUtilities::addWindowEventListener(window, this);
        listener_registered = true;

        std::cout << "Ogre bootstrap: registering HLMS." << std::endl;

        if (!register_hlms()) {
            return false;
        }

        std::cout << "Ogre bootstrap: HLMS registered." << std::endl;

        std::cout << "Ogre bootstrap: render window created." << std::endl;

        std::cout << "Ogre bootstrap: creating scene manager." << std::endl;

        scene_manager = root->createSceneManager(
            Ogre::ST_GENERIC,
            1u,
            Ogre::String());
        std::cout << "Ogre bootstrap: scene manager created." << std::endl;

        scene_manager->setAmbientLight(
            Ogre::ColourValue(0.35f, 0.35f, 0.40f),
            Ogre::ColourValue(0.10f, 0.10f, 0.12f),
            Ogre::Vector3(0.0f, 0.0f, 1.0f),
            1.0f);
        std::cout << "Ogre bootstrap: ambient light configured." << std::endl;

        camera = scene_manager->createCamera("ScrimmageOgreNextCamera");
        std::cout << "Ogre bootstrap: camera created." << std::endl;
        camera->setNearClipDistance(0.5f);
        camera->setAutoAspectRatio(true);

        const Ogre::Vector3 default_camera_position(0.0f, 1.0f, 200.0f);
        const Ogre::Vector3 default_camera_focal_point(0.0f, 0.0f, 0.0f);
        const Ogre::Vector3 camera_position = to_ogre_vector(
            parse_camera_vector(camera_params, "pos", "0, 1, 200"),
            default_camera_position);
        const Ogre::Vector3 camera_focal_point = to_ogre_vector(
            parse_camera_vector(camera_params, "focal_point", "0, 0, 0"),
            default_camera_focal_point);

        camera->setPosition(camera_position);
        camera->lookAt(camera_focal_point);
        std::cout << "Ogre bootstrap: camera configured." << std::endl;

        if (!create_test_primitive()) {
            return false;
        }

        if (!create_basic_workspace()) {
            return false;
        }

        std::cout << "Initialized Ogre-Next Unlit compositor bootstrap viewer." << std::endl;
        return true;
    }

    bool create_test_primitive() {
        if (scene_manager == nullptr) {
            return false;
        }

        Ogre::HlmsManager* hlms_manager = root != nullptr ? root->getHlmsManager() : nullptr;
        Ogre::Hlms* unlit_hlms = hlms_manager != nullptr ? hlms_manager->getHlms(Ogre::HLMS_UNLIT) : nullptr;
        if (unlit_hlms == nullptr) {
            std::cerr << "Error: Ogre-Next Unlit HLMS is unavailable for the test primitive."
                      << std::endl;
            return false;
        }

        Ogre::HlmsDatablock* datablock = unlit_hlms->getDatablock(test_primitive_datablock_name);
        if (datablock == nullptr) {
            datablock = unlit_hlms->createDatablock(
                test_primitive_datablock_name,
                test_primitive_datablock_name,
                Ogre::HlmsMacroblock(),
                Ogre::HlmsBlendblock(),
                Ogre::HlmsParamVec());
        }

        Ogre::HlmsUnlitDatablock* unlit_datablock =
            dynamic_cast<Ogre::HlmsUnlitDatablock*>(datablock);
        if (unlit_datablock == nullptr) {
            std::cerr << "Error: failed to create an Ogre-Next Unlit datablock for the test primitive."
                      << std::endl;
            return false;
        }

        unlit_datablock->setUseColour(true);
        unlit_datablock->setColour(Ogre::ColourValue(0.95f, 0.35f, 0.15f, 1.0f));

        test_primitive_wire_aabb = scene_manager->createWireAabb();
        test_primitive_wire_aabb->setDatablock(unlit_datablock);
        test_primitive_wire_aabb->setToAabb(
            Ogre::Aabb(Ogre::Vector3::ZERO, Ogre::Vector3(20.0f, 20.0f, 20.0f)));

        std::cout << "Ogre bootstrap: created visible test primitive." << std::endl;
        return true;
    }

    bool create_basic_workspace() {
        if (root == nullptr || scene_manager == nullptr || camera == nullptr || window == nullptr) {
            return false;
        }

        Ogre::CompositorManager2* compositor_manager = root->getCompositorManager2();
        if (compositor_manager == nullptr) {
            std::cerr << "Error: Ogre-Next compositor manager is unavailable." << std::endl;
            return false;
        }

        Ogre::TextureGpu* final_render_target = window->getTexture();
        if (final_render_target == nullptr) {
            std::cerr << "Error: Ogre-Next window did not provide a render texture." << std::endl;
            return false;
        }

        if (workspace != nullptr) {
            compositor_manager->removeWorkspace(workspace);
            workspace = nullptr;
        }

        const Ogre::IdString workspace_def_name(workspace_name);
        const Ogre::IdString empty_shadow_node_name;
        if (!compositor_manager->hasWorkspaceDefinition(workspace_def_name)) {
            compositor_manager->createBasicWorkspaceDef(
                workspace_name,
                Ogre::ColourValue(0.10f, 0.10f, 0.12f, 1.0f),
                empty_shadow_node_name);
        }

        workspace = compositor_manager->addWorkspace(
            scene_manager,
            final_render_target,
            camera,
            workspace_def_name,
            true);

        if (workspace == nullptr) {
            std::cerr << "Error: failed to create the Ogre-Next compositor workspace." << std::endl;
            return false;
        }

        std::cout << "Ogre bootstrap: compositor workspace created." << std::endl;
        return true;
    }

    ~Impl() override {
        destroy();
    }

    Ogre::RenderSystem* select_render_system() const {
        if (!root) {
            return nullptr;
        }

        if (Ogre::RenderSystem* render_system =
                root->getRenderSystemByName("OpenGL 3+ Rendering Subsystem")) {
            return render_system;
        }

        const Ogre::RenderSystemList& renderers = root->getAvailableRenderers();
        for (Ogre::RenderSystem* render_system : renderers) {
            if (render_system != nullptr &&
                boost::icontains(render_system->getName(), "OpenGL")) {
                return render_system;
            }
        }

        return renderers.empty() ? nullptr : renderers.front();
    }

    bool register_hlms() {
        if (!root) {
            return false;
        }

        Ogre::HlmsManager* hlms_manager = root->getHlmsManager();
        if (hlms_manager == nullptr) {
            std::cerr << "Error: Ogre-Next HLMS manager is unavailable." << std::endl;
            return false;
        }

        const std::string hlms_root = find_hlms_root();
        if (hlms_root.empty()) {
            std::cerr << "Error: Ogre-Next HLMS assets were not found. Checked SCRIMMAGE_DATA_PATH, "
                      << "SCRIMMAGE_OGRE_NEXT_HLMS_PATH, and standard SCRIMMAGE share paths." << std::endl;
            return false;
        }

        const std::string media_root = fs::path(hlms_root).parent_path().string();
        Ogre::String main_folder_path;
        Ogre::StringVector library_folder_paths;

        Ogre::HlmsUnlit::getDefaultPaths(main_folder_path, library_folder_paths);
        {
            std::vector<std::string> library_folders;
            library_folders.reserve(library_folder_paths.size());
            for (const Ogre::String& folder : library_folder_paths) {
                library_folders.push_back(media_root + "/" + folder);
            }

            if (!load_hlms_archive_set(media_root + "/" + main_folder_path,
                                       library_folders,
                                       hlms_unlit_archives)) {
                std::cerr << "Error: failed to load Ogre-Next Unlit HLMS assets from "
                          << hlms_root << std::endl;
                return false;
            }
        }

        if (hlms_manager->getHlms(Ogre::HLMS_UNLIT) == nullptr) {
            hlms_manager->registerHlms(
                OGRE_NEW Ogre::HlmsUnlit(
                    hlms_unlit_archives.data_folder,
                    &hlms_unlit_archives.library_folders));
        }

        std::cout << "Ogre bootstrap: HLMS assets loaded from " << hlms_root << std::endl;
        return true;
    }

    void destroy() {
        running = false;

        if (window != nullptr && listener_registered) {
            Ogre::WindowEventUtilities::removeWindowEventListener(window, this);
            listener_registered = false;
        }

        if (root != nullptr) {
            Ogre::CompositorManager2* compositor_manager = root->getCompositorManager2();
            if (workspace != nullptr && compositor_manager != nullptr) {
                compositor_manager->removeWorkspace(workspace);
                workspace = nullptr;
            }

            if (scene_manager != nullptr) {
                if (test_primitive_wire_aabb != nullptr) {
                    scene_manager->destroyWireAabb(test_primitive_wire_aabb);
                    test_primitive_wire_aabb = nullptr;
                }

                if (camera != nullptr) {
                    scene_manager->destroyCamera(camera);
                    camera = nullptr;
                }

                root->destroySceneManager(scene_manager);
                scene_manager = nullptr;
            }
        }

        window = nullptr;
        root.reset();
        render_system_plugin_loaded = false;

        unload_hlms_archive_set(hlms_pbs_archives);
        unload_hlms_archive_set(hlms_unlit_archives);
    }

    bool windowClosing(Ogre::Window* closing_window) override {
        if (closing_window == window) {
            running = false;
        }
        return true;
    }

    void windowClosed(Ogre::Window* closed_window) override {
        if (closed_window == window) {
            running = false;
        }
    }
};

OgreNextBootstrap::OgreNextBootstrap() : impl_(new Impl()) {}

OgreNextBootstrap::~OgreNextBootstrap() = default;

bool OgreNextBootstrap::init(
    const MissionParsePtr& mp,
    const std::map<std::string, std::string>& camera_params) {
    if (!impl_->virtual_desktop.ensure_display()) {
        return false;
    }
    impl_->destroy();
    impl_->mission_parse = mp;
    impl_->camera_params = camera_params;
    return true;
}

bool OgreNextBootstrap::run() {
    if (impl_->root == nullptr || impl_->window == nullptr) {
        try {
            if (!impl_->initialize_on_current_thread()) {
                impl_->destroy();
                return false;
            }
        } catch (const Ogre::Exception& exception) {
            LOG_ERROR(exception.getFullDescription());
            impl_->destroy();
            return false;
        } catch (const std::exception& exception) {
            LOG_ERROR(exception.what());
            impl_->destroy();
            return false;
        }
    }

    impl_->running = true;

    try {
        while (impl_->running && !impl_->window->isClosed()) {
            Ogre::WindowEventUtilities::messagePump();
            if (impl_->workspace != nullptr) {
                if (!impl_->root->renderOneFrame()) {
                    break;
                }
            } else {
                impl_->window->swapBuffers();
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
        }
    } catch (const Ogre::Exception& exception) {
        LOG_ERROR(exception.getFullDescription());
        impl_->running = false;
        return false;
    } catch (const std::exception& exception) {
        LOG_ERROR(exception.what());
        impl_->running = false;
        return false;
    }

    impl_->running = false;
    return true;
}

void OgreNextBootstrap::stop() {
    if (impl_) {
        impl_->running = false;
    }
}

}  // namespace scrimmage