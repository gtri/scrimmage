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

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/simcontrol/SimControl.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/metrics/Metrics.h>

#include <signal.h>
#include <cstdlib>

#include <iostream>
#include <ctime>

#include <unordered_set>
#include <string>
#include <ostream>
#include <memory>
#if ENABLE_VIEWER == 1
#include <scrimmage/viewer/Viewer.h>
#endif

#include <scrimmage/log/Log.h>

#if ENABLE_PYTHON_BINDINGS == 1
#include <Python.h>
#endif

using std::cout;
using std::endl;

namespace sc = scrimmage;

sc::SimControl simcontrol;

// Handle kill signal
void HandleSignal(int s) {
    cout << endl << "Exiting gracefully" << endl;
    simcontrol.force_exit();
}

int main(int argc, char *argv[]) {
    // Handle kill signals
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = HandleSignal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int job_id = -1;
    int task_id = -1;

    bool seed_set = false;
    std::string seed = "";

    int opt;
    while ((opt = getopt(argc, argv, "t:j:s:")) != -1) {
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
        case '?':
            if (optopt == 't') {
                fprintf(stderr, "Option -%d requires an integer argument.\n", optopt);
            } else {
                fprintf(stderr,
                         "Unknown option character `\\x%x'.\n",
                         optopt);
            }
            return 1;
        default:
            exit(EXIT_FAILURE);
        }
    }

    if (optind >= argc || argc < 2) {
        cout << "usage: " << argv[0] << " scenario.xml" << endl;
        return -1;
    }

    // Parse mission config file:
    sc::MissionParsePtr mp = std::make_shared<sc::MissionParse>();

    // If the task_num was defined, tell the mission parser, so it can be
    // stored in the directory log name
    if (task_id != -1) {
        mp->set_task_number(task_id);
    }
    if (job_id != -1) {
        mp->set_job_number(job_id);
    }
    if (!mp->parse(argv[optind])) {
        cout << "Failed to parse file: " << argv[optind] << endl;
        return -1;
    }
    mp->create_log_dir();

    // Overwrite the seed if it's set
    if (seed_set) {
        mp->params()["seed"] = seed;
    }

    std::string output_type = sc::get("output_type", mp->params(), std::string("frames"));
    bool output_all = output_type.find("all") != std::string::npos;
    bool output_frames = output_all || output_type.find("frames") != std::string::npos;
    bool output_summary = output_all || output_type.find("summary") != std::string::npos;
    bool output_git = output_all || output_type.find("git_commits");

    // Setup Logger
    std::shared_ptr<sc::Log> log(new sc::Log());
    if (output_frames) {
        log->set_enable_log(true);
        log->init(mp->log_dir(), sc::Log::WRITE);
    } else {
        log->set_enable_log(false);
        log->init(mp->log_dir(), sc::Log::NONE);
    }

    simcontrol.set_log(log);

    sc::InterfacePtr to_gui_interface = std::make_shared<sc::Interface>();
    sc::InterfacePtr from_gui_interface = std::make_shared<sc::Interface>();
    to_gui_interface->set_log(log);
    from_gui_interface->set_log(log);

    simcontrol.set_incoming_interface(from_gui_interface);
    simcontrol.set_outgoing_interface(to_gui_interface);

    // Split off SimControl in it's own thread
#if ENABLE_PYTHON_BINDINGS == 1
    Py_Initialize();
#endif
    simcontrol.set_mission_parse(mp);
    if (!simcontrol.init()) {
        cout << "SimControl init() failed." << endl;
        return -1;
    }

    bool display_progress = sc::get("display_progress", mp->params(), true);
    simcontrol.display_progress(display_progress);

#if ENABLE_VIEWER == 0
    simcontrol.pause(false);
    simcontrol.run();
#else
    if (simcontrol.enable_gui()) {
        simcontrol.start();
        scrimmage::Viewer viewer;
        viewer.set_incoming_interface(to_gui_interface);
        viewer.set_outgoing_interface(from_gui_interface);
        viewer.set_enable_network(false);
        viewer.init(mp->attributes()["camera"]);
        viewer.run();

        // When the viewer finishes, tell simcontrol to exit
        simcontrol.force_exit();
        simcontrol.join();
    } else {
        simcontrol.pause(false);
        simcontrol.run();
    }
#endif

#if ENABLE_PYTHON_BINDINGS == 1
    Py_Finalize();
#endif

    // runtime
    std::ofstream runtime_file(mp->log_dir() + "/runtime_seconds.txt");
    double t = simcontrol.timer().elapsed_time().total_milliseconds() / 1000.0;
    double sim_t = simcontrol.t();
    runtime_file << "wall: " << t << std::endl;
    runtime_file << "sim: " << sim_t << std::endl;
    runtime_file.close();

    // summary
    if (output_summary) {
        std::map<int, double> team_scores;
        std::map<int, std::map<std::string, double>> team_metrics;
        std::list<std::string> headers;

        // Loop through each of the metrics plugins.
        for (sc::MetricsPtr metrics : simcontrol.metrics()) {
            cout << sc::generate_chars("=", 80) << endl;
            cout << metrics->name() << endl;
            cout << sc::generate_chars("=", 80) << endl;
            metrics->calc_team_scores();
            metrics->print_team_summaries();

            // Add all elements from individual metrics plugin to overall
            // metrics data structure
            for (auto const &team_str_double : metrics->team_metrics()) {
                team_metrics[team_str_double.first].insert(team_str_double.second.begin(),
                                                           team_str_double.second.end());
            }

            // Calculate aggregated team scores:
            for (auto const &team_score : metrics->team_scores()) {
                if (team_scores.count(team_score.first) == 0) {
                    team_scores[team_score.first] = 0;
                }
                team_scores[team_score.first] += team_score.second;
            }

            // Create list of all csv headers
            headers.insert(headers.end(), metrics->headers().begin(),
                           metrics->headers().end());
        }

        // Create headers string
        std::string csv_str = "team_id,score";
        for (std::string header : headers) {
            csv_str += "," + header;
        }
        csv_str += "\n";

        // Loop over each team and generate csv output
        for (auto const &team_str_double : team_metrics) {

            // Each line starts with team_id,score
            csv_str += std::to_string(team_str_double.first);
            csv_str += "," + std::to_string(team_scores[team_str_double.first]);

            // Loop over all possible headers, if the header doesn't exist for
            // a specific team, default the value for that header to zero.
            for (std::string header : headers) {
                csv_str += ",";

                auto it = team_str_double.second.find(header);
                if (it != team_str_double.second.end()) {
                    csv_str += std::to_string(it->second);
                } else {
                    csv_str += std::to_string(0.0);
                }
            }
            csv_str += "\n";
        }

        // Write CSV string to file
        std::string out_file = mp->log_dir() + "/summary.csv";
        std::ofstream summary_file(out_file);
        if (!summary_file.is_open()) {
            std::cout << "could not open " << out_file
                      << " for writing metrics" << std::endl;
            return -1;
        }
        summary_file << csv_str << std::flush;
        summary_file.close();

        // Print Overall Scores
        cout << sc::generate_chars("=", 80) << endl;
        cout << "Overall Scores" << endl;
        cout << sc::generate_chars("=", 80) << endl;
        for (auto const &team_score : team_scores) {
            cout << "Team ID: " << team_score.first << endl;
            cout << "Score: " << team_score.second << endl;
            cout << sc::generate_chars("-", 80) << endl;
        }
    }

    if (output_git) {
        std::map<std::string, std::unordered_set<std::string>> commits =
            simcontrol.plugin_manager()->get_commits();
        std::string scrimmage_version = sc::get_version();

        if (scrimmage_version != "") {
            commits[scrimmage_version].insert("scrimmage");
        }

        for (auto &kv : commits) {
            std::string output = kv.first + ":";
            for (const std::string &plugin_name : kv.second) {
                output += plugin_name + ",";
            }
            output.pop_back();
            log->write_ascii(output);
        }
    }

    if (sc::get("plot_tracks", mp->params(), false)) {
        std::string plot_cmd = "plot_3d_fr.py " + log->frames_filename();
        int result = std::system(plot_cmd.c_str());
        if (result != 0) {
            cout << "plot_tracks failed with return code: "
                 << result << endl;
        }
    }

    // Close the log file
    log->close_log();

    cout << "Simulation Complete" << endl;
    return 0;
}
