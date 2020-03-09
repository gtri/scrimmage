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
#include <signal.h>

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/network/Interface.h>
#if ENABLE_VTK == 1
#include <scrimmage/viewer/Viewer.h>
#endif

#include <scrimmage/common/Timer.h>
#include <scrimmage/log/Log.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono> // NOLINT
#include <thread> // NOLINT

#include <boost/filesystem.hpp>

using std::cout;
using std::endl;

namespace fs = boost::filesystem;
namespace sc = scrimmage;
namespace sp = scrimmage_proto;

// Handle kill signal
std::mutex mutex_;
bool quit = false;
void HandleSignal(int s) {
    mutex_.lock();
    quit = true;
    mutex_.unlock();
    cout << endl << "Exiting gracefully" << endl;
}

void playback_loop(std::shared_ptr<sc::Log> log,
                   sc::InterfacePtr in_interface,
                   sc::InterfacePtr out_interface) {
    // Get dt from first two frames
    double dt = 0.1;
    if (log->frames().size() >= 2) {
        auto it = log->frames().begin();
        double t0 = (*it)->time();
        ++it;
        double t1 = (*it)->time();
        dt = t1 - t0;
    } else {
        cout << "Fewer than two frames parsed. Using dt: " << dt << endl;
    }

    bool paused = true;

    double time_warp = 5;
    double rate = 1.0 / dt;

    sc::Timer timer;
    timer.set_iterate_rate(rate);
    timer.set_time_warp(time_warp);
    timer.update_time_config();

    bool exit_loop = false;

    auto it_shapes = log->shapes().begin();
    auto it_utm_terrain = log->utm_terrain().begin();
    auto it_contact_visual = log->contact_visual().begin();

    timer.start_overall_timer();
    for (auto it = log->frames().begin(); it != log->frames().end(); it++) {
        timer.start_loop_timer();
        // Send all other messages up to current frame time before sending
        // current frame
        while (it_shapes != log->shapes().end() &&
               (*it_shapes)->time() <= (*it)->time()) {
            out_interface->send_shapes(**it_shapes);
            ++it_shapes;
        }

        while (it_utm_terrain != log->utm_terrain().end() &&
               (*it_utm_terrain)->time() <= (*it)->time()) {
            out_interface->send_utm_terrain(*it_utm_terrain);
            ++it_utm_terrain;
        }

        while (it_contact_visual != log->contact_visual().end() &&
               (*it_contact_visual)->time() <= (*it)->time()) {
            out_interface->send_contact_visual(*it_contact_visual);
            ++it_contact_visual;
        }

        out_interface->send_frame(*it);

        // Wait loop timer.
        // Stay in loop if currently paused.
        do {
            timer.loop_wait();

            mutex_.lock();
            bool quit_test = quit;
            mutex_.unlock();

            if (quit_test) {
                exit_loop = true;
            }

            bool single_step = false;
            // Do we have any simcontrol message updates from GUI?
            if (in_interface->gui_msg_update()) {
                in_interface->gui_msg_mutex.lock();
                auto &control = in_interface->gui_msg();
                auto it = control.begin();
                while (it != control.end()) {
                    if (it->inc_warp()) {
                        timer.inc_warp();
                    } else if (it->dec_warp()) {
                        timer.dec_warp();
                    } else if (it->toggle_pause()) {
                        paused = !paused;
                    } else if (it->single_step()) {
                        single_step = true;
                    }
                    control.erase(it++);
                }
                in_interface->gui_msg_mutex.unlock();
            }

            scrimmage_proto::SimInfo info;
            info.set_time((*it)->time());
            info.set_desired_warp(timer.time_warp());
            info.set_actual_warp(timer.time_warp());
            out_interface->send_sim_info(info);

            if (single_step) {
                break;
            }
        } while (paused && !exit_loop);
        if (exit_loop) break;
    }
}

int main(int argc, char *argv[]) {
    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = HandleSignal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (optind >= argc || argc < 2) {
        cout << "usage: " << argv[0] << " /path/to/log-dir" << endl;
        return -1;
    }

    // Setup Logger
    std::shared_ptr<sc::Log> log(new sc::Log);
    log->init(std::string(argv[1]), sc::Log::READ);

    sc::InterfacePtr to_gui_interface(new sc::Interface);
    sc::InterfacePtr from_gui_interface(new sc::Interface);

    to_gui_interface->set_mode(sc::Interface::shared);
    from_gui_interface->set_mode(sc::Interface::shared);

    //// Kick off server thread
    // std::thread server_thread(&Interface::init_network, &(*incoming_interface_),
    //                           Interface::server, "localhost", 50051);

    cout << "Frames parsed: " << log->frames().size() << endl;

    std::thread playback(playback_loop, log, from_gui_interface,
                         to_gui_interface);
    playback.detach(); // todo

    auto mp = std::make_shared<sc::MissionParse>();
    mp->set_log_dir("");

    sc::Viewer viewer;
    viewer.set_enable_network(false);
    viewer.set_incoming_interface(to_gui_interface);
    viewer.set_outgoing_interface(from_gui_interface);

    double dt;
    if (log->frames().size() >= 2) {
        auto frame2 = *(std::next(log->frames().begin(), 1));
        auto frame1 = log->frames().front();
        dt = frame2->time() - frame1->time();
    } else {
        dt = 1.0e-6;
    }
    mp->set_dt(dt);

    viewer.init(mp, {});
    viewer.run();

    cout << "Playback Complete" << endl;
    return 0;
}
