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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_set>

#include <signal.h>

#include "scrimmage/autonomy/Autonomy.h"
#include "scrimmage/common/Utilities.h"
#include "scrimmage/entity/Contact.h"
#include "scrimmage/entity/Entity.h"
#include "scrimmage/metrics/Metrics.h"
#include "scrimmage/network/Interface.h"
#include "scrimmage/parse/MissionParse.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/PluginManager.h"
#include "scrimmage/simcontrol/SimControl.h"
#include "scrimmage/simcontrol/SimUtils.h"
#if ENABLE_VTK == 1
#include "scrimmage/viewer/Viewer.h"
#endif
#if ENABLE_OGRE == 1
#include "scrimmage/viewer/ogre/OgreViewer.h"
#endif

#include <boost/optional.hpp>
#include <getopt.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include "scrimmage/log/Log.h"

using std::cout;
using std::cerr;
using std::endl;

namespace sc = scrimmage;

// VNC process management
namespace {
    pid_t xvfb_pid = 0;
    pid_t x11vnc_pid = 0;
    pid_t websockify_pid = 0;
    bool vnc_started_global = false;
    
    void cleanup_vnc() {
        if (!vnc_started_global) return;
        vnc_started_global = false;  // Prevent double cleanup
        // Kill entire process groups (negative PID)
        if (xvfb_pid > 0) {
            kill(-xvfb_pid, SIGKILL);
            kill(xvfb_pid, SIGKILL);
        }
        if (x11vnc_pid > 0) {
            kill(-x11vnc_pid, SIGKILL);
            kill(x11vnc_pid, SIGKILL);
        }
        if (websockify_pid > 0) {
            kill(-websockify_pid, SIGKILL);
            kill(websockify_pid, SIGKILL);
        }
    }
    
    pid_t start_process(const char* cmd) {
        pid_t pid = fork();
        if (pid == 0) {
            // Child - die when parent dies
            prctl(PR_SET_PDEATHSIG, SIGKILL);
            // Become a new process group leader
            setpgid(0, 0);
            // Redirect output to /dev/null and exec
            freopen("/dev/null", "w", stdout);
            freopen("/dev/null", "w", stderr);
            execl("/bin/sh", "sh", "-c", cmd, nullptr);
            _exit(1);
        }
        // Parent - also set process group (handles race condition)
        if (pid > 0) setpgid(pid, pid);
        return pid;
    }
    
    bool start_vnc_display() {
        // Kill any existing VNC processes (including orphans from previous runs)
        system("pkill -9 -f 'Xvfb :99' 2>/dev/null");
        system("pkill -9 -f 'x11vnc.*:99' 2>/dev/null");  
        system("pkill -9 -f 'websockify.*6080' 2>/dev/null");
        usleep(500000);  // 0.5s
        
        // Start Xvfb
        xvfb_pid = start_process("exec Xvfb :99 -screen 0 1280x800x24 +extension GLX +render -noreset");
        
        // Wait for Xvfb to be ready (up to 5 seconds)
        bool xvfb_ready = false;
        for (int i = 0; i < 50; i++) {
            usleep(100000);  // 100ms
            if (kill(xvfb_pid, 0) != 0) {
                cerr << "Xvfb process died" << endl;
                break;
            }
            // Check if display :99 is available
            if (system("xdpyinfo -display :99 >/dev/null 2>&1") == 0) {
                xvfb_ready = true;
                break;
            }
        }
        if (!xvfb_ready) {
            cerr << "Failed to start Xvfb - display :99 not available" << endl;
            return false;
        }
        
        // Start x11vnc
        x11vnc_pid = start_process("exec x11vnc -display :99 -forever -nopw -shared -rfbport 5900");
        usleep(500000);
        
        // Start websockify for noVNC
        websockify_pid = start_process("exec websockify --web=/usr/share/novnc 6080 localhost:5900");
        usleep(500000);
        
        cout << "\n=============================================" << endl;
        cout << "View in browser: http://localhost:6080/vnc.html" << endl;
        cout << "=============================================\n" << endl;
        
        setenv("DISPLAY", ":99", 1);
        vnc_started_global = true;
        return true;
    }
    
    bool display_available() {
        const char* display = getenv("DISPLAY");
        if (!display || strlen(display) == 0) return false;
        // Verify the display actually works
        if (system("xdpyinfo >/dev/null 2>&1") != 0) return false;
        return true;
    }
}

// Callback function for shutdown
namespace {

// https://stackoverflow.com/a/48164204
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) {
    shutdown_handler(signal);
}

}  // namespace

int main(int argc, char* argv[]) {
    // Register VNC cleanup to run on any exit
    std::atexit(cleanup_vnc);
    
    sc::SimControl simcontrol;

    // Handle kill signals
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    shutdown_handler = [&](int /*s*/) {
        cout << endl << "Exiting gracefully" << endl;
        cleanup_vnc();  // Clean up VNC on signal
        simcontrol.force_exit();
    };
    sa.sa_handler = signal_handler;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int job_id = -1;
    int task_id = -1;

    bool seed_set = false;
    std::string seed = "";
    std::ostringstream overrides;
    bool first_override = true;
    bool use_ogre = false;

    // Long options for getopt
    static struct option long_options[] = {
        {"ogre", no_argument, 0, 'o'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, "t:j:s:oh", long_options, &option_index)) != -1) {
        switch (opt) {
            case 't':
                task_id = std::stoi(std::string(optarg));
                break;
            case 'j':
                job_id = std::stoi(std::string(optarg));
                break;
            case 's':
                seed = std::string(optarg);
                seed_set = true;
                break;
            case 'o':
                use_ogre = true;
                break;
            case 'h':
                cout << "usage: " << argv[0] << " [options] [var:=value ...] scenario.xml [var:=value ...]" << endl;
                cout << "Options:" << endl;
                cout << "  -t <task_id>    Set task ID" << endl;
                cout << "  -j <job_id>     Set job ID" << endl;
                cout << "  -s <seed>       Set random seed" << endl;
                cout << "  --ogre, -o      Use Ogre3D viewer instead of VTK" << endl;
                cout << "  --help, -h      Show this help message" << endl;
                return 0;
            case '?':
                if (optopt == 't' || optopt == 'j' || optopt == 's') {
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                } else {
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                }
                return 1;
            default:
                exit(EXIT_FAILURE);
        }
    }

    if (optind >= argc || argc < 2) {
        cout << "usage: " << argv[0] << " [var:=value ...] scenario.xml [var:=value ...]" << endl;
        return -1;
    }

    // Parse positional arguments: args with ":=" are overrides, first without is mission file
    std::string mission_file;
    for (int i = optind; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find(":=") != std::string::npos) {
            // This is an override - normalize := to =
            if (!first_override) {
                overrides << ",";
            }
            first_override = false;
            // Replace := with =
            std::string normalized = arg;
            size_t pos = normalized.find(":=");
            normalized.replace(pos, 2, "=");
            overrides << normalized;
        } else if (mission_file.empty()) {
            // First non-override arg is the mission file
            mission_file = arg;
        }
    }

    if (mission_file.empty()) {
        std::cerr << "Error: No mission file specified" << endl;
        std::cerr << "usage: " << argv[0] << " [var:=value ...] scenario.xml [var:=value ...]" << endl;
        return -1;
    }

    // Overwrite mission parameters from command line
    if (task_id != -1)
        simcontrol.mp()->set_task_number(task_id);
    if (job_id != -1)
        simcontrol.mp()->set_job_number(job_id);
    simcontrol.mp()->set_overrides(overrides.str());
    if (not simcontrol.init(mission_file)) {
        cout << "Failed to initialize SimControl with mission file: " << mission_file << endl;
        return -1;
    }

    if (seed_set)
        simcontrol.mp()->params()["seed"] = seed;

    simcontrol.run_send_shapes();  // draw any intial shapes

    std::shared_ptr<std::thread> viewer_thread = nullptr;

#if ENABLE_VTK == 0 && ENABLE_OGRE == 0
    // If no GUI was built, un-pause by default.
    (void)use_ogre;  // Unused when no viewer available
    simcontrol.pause(false);
#else
    // Runtime viewer selection
#if ENABLE_VTK == 1
    std::shared_ptr<scrimmage::Viewer> vtk_viewer = nullptr;
#endif
#if ENABLE_OGRE == 1
    std::shared_ptr<scrimmage::viewer::OgreViewer> ogre_viewer = nullptr;
#endif

    if (simcontrol.enable_gui()) {
        // Start VNC if no display is available
        if (!display_available()) {
            if (!start_vnc_display()) {
                cout << "Warning: Could not start VNC display, running without GUI" << endl;
                simcontrol.pause(false);
                goto skip_viewer;
            }
        }

        auto outgoing = simcontrol.outgoing_interface();
        auto incoming = simcontrol.incoming_interface();

        // Get the camera params from mission file, if they exist
        std::map<std::string, std::string> camera_params;
        auto it_camera = simcontrol.mp()->attributes().find("camera");
        if (it_camera != simcontrol.mp()->attributes().end()) {
            camera_params = it_camera->second;
        }

#if ENABLE_OGRE == 1 && ENABLE_VTK == 1
        // Both viewers available - select based on flag
        if (use_ogre) {
            ogre_viewer = std::make_shared<scrimmage::viewer::OgreViewer>();
            ogre_viewer->set_incoming_interface(outgoing);
            ogre_viewer->set_outgoing_interface(incoming);
            ogre_viewer->set_enable_network(false);
            if (!ogre_viewer->init(simcontrol.mp(), camera_params)) {
                return -1;
            }
            viewer_thread = std::make_shared<std::thread>([&]() { ogre_viewer->run(); });
        } else {
            vtk_viewer = std::make_shared<scrimmage::Viewer>();
            vtk_viewer->set_incoming_interface(outgoing);
            vtk_viewer->set_outgoing_interface(incoming);
            vtk_viewer->set_enable_network(false);
            if (!vtk_viewer->init(simcontrol.mp(), camera_params)) {
                return -1;
            }
            viewer_thread = std::make_shared<std::thread>([&]() { vtk_viewer->run(); });
        }
#elif ENABLE_OGRE == 1
        // Only Ogre available
        (void)use_ogre;
        ogre_viewer = std::make_shared<scrimmage::viewer::OgreViewer>();
        ogre_viewer->set_incoming_interface(outgoing);
        ogre_viewer->set_outgoing_interface(incoming);
        ogre_viewer->set_enable_network(false);
        if (!ogre_viewer->init(simcontrol.mp(), camera_params)) {
            return -1;
        }
        viewer_thread = std::make_shared<std::thread>([&]() { ogre_viewer->run(); });
#elif ENABLE_VTK == 1
        // Only VTK available
        (void)use_ogre;
        vtk_viewer = std::make_shared<scrimmage::Viewer>();
        vtk_viewer->set_incoming_interface(outgoing);
        vtk_viewer->set_outgoing_interface(incoming);
        vtk_viewer->set_enable_network(false);
        if (!vtk_viewer->init(simcontrol.mp(), camera_params)) {
            return -1;
        }
        viewer_thread = std::make_shared<std::thread>([&]() { vtk_viewer->run(); });
#endif
    } else {
        // If the GUI isn't enabled, un-pause by default.
        simcontrol.pause(false);
    }
skip_viewer:
#endif

    // Run SimControl::run() blocking function, which steps through simulation
    if (not simcontrol.run()) {
        cout << "SimControl::run() failed." << endl;
        return -1;  // cleanup_vnc() called via atexit
    }

    if (not simcontrol.shutdown()) {
        cout << "Failed to shutdown properly." << endl;
        return -1;  // cleanup_vnc() called via atexit
    }

    // Join the viewer thread, if it was created
    if (viewer_thread != nullptr) {
        viewer_thread->join();
    }

    // cleanup_vnc() called automatically via atexit
    return 0;
}
