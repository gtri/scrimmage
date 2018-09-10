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
#include <scrimmage/common/Utilities.h>
#include <scrimmage/log/Log.h>
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/common/FileSearch.h>

#include <iostream>
#include <string>
#include <iomanip>
#include <chrono> // NOLINT
#include <ctime>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

namespace fs = boost::filesystem;
namespace sc = scrimmage;

using std::cout;
using std::endl;

void usage(char *argv[]) {
    cout << endl << "Usage: " << argv[0] << " -d ~/.scrimmage/logs"
         << endl << endl;
}

int main(int argc, char *argv[]) {

    if (argc < 2) {
        cout << "usage: " << argv[0] << " <directory of filter results>" << endl;
        return -1;
    }

    // Directory holding all the runs (typically, ~/scrimmage-log)
    std::string log_dir = std::string(argv[1]);

    if (!fs::exists(fs::path(log_dir))) {
        cout << "Log directory doesn't exist: " << log_dir << endl;
        usage(argv);
        return -1;
    }

    // Find all summary.csv files under the directory
    std::vector<std::string> paths;
    fs::path root = log_dir;
    if (fs::exists(root) && fs::is_directory(root)) {
        fs::recursive_directory_iterator it(root);
        fs::recursive_directory_iterator endit;

        while (it != endit) {
            const std::string summary_csv = "summary.csv";
            if (fs::is_regular_file(*it) && it->path().filename() == summary_csv) {
                std::string full_path = fs::absolute(it->path()).string();
                paths.push_back(full_path);
            }
            ++it;
        }
    } else {
        cout << "Path doesn't exist: " << log_dir << endl;
    }

    // Create output directory for aggregated results
    std::string output_dir = log_dir + "/aggregate/team-vs-team";

    if (fs::exists(output_dir) && !fs::remove_all(output_dir)) {
        cout << "Failed to remove old directory: " << output_dir << endl;
        return -1;
    }

    if (!fs::exists(output_dir)) {
        if (!fs::create_directories(output_dir)) {
            cout << "Failed to create output directory: " << output_dir << endl;
            return -1;
        }
    }

    int number_of_runs = paths.size();
    cout << "Aggregating " << number_of_runs << " runs. " << endl;

    int count = 0;
    std::map<int, int> team_wins;
    std::map<int, int> team_draws;

    std::clock_t start = std::clock();
    std::ofstream summary_file(log_dir + "/aggregate/all_runs.csv");

    // Using the score from each summary_csv, keep track of team wins
    for (std::string &filename : paths) {
        if (!fs::exists(fs::path(filename))) {
            cout << "summary.csv doesn't exist: " << filename << endl;
            return -1;
        }

        std::ifstream csv_file(filename);

        std::string line;
        std::getline(csv_file, line); // skip header comment

        std::map<int, double> team_scores;
        while (std::getline(csv_file, line)) {
            std::vector<std::string> t;
            boost::split(t, line, boost::is_any_of(","));

            if (t.size() < 2) continue;

            int team_id = std::stoi(t[0]);
            team_scores[team_id] = std::stod(t[1]);
        }

        // Determine which teams lost, won, and drew
        double max_score = -std::numeric_limits<double>::infinity();
        std::vector<int> winning_team;
        for (auto &kv : team_scores) {
            if (std::abs(kv.second-max_score) < 0.000001) {
                // A possible draw
                winning_team.push_back(kv.first);
            } else if (kv.second > max_score) {
                max_score = kv.second;
                winning_team.clear();
                winning_team.push_back(kv.first);
            }
        }

        std::string result_filename = "";
        if (winning_team.size() == 0) {
            cout << "Warning: Couldn't determine winner of: " << filename << endl;
            continue;
        } else if (winning_team.size() == 1) {
            result_filename = "team_" + std::to_string(winning_team[0]);
            // One winner, write a win file
            if (team_wins.count(winning_team[0]) > 0) {
                team_wins[winning_team[0]] += 1;
            } else {
                team_wins[winning_team[0]] = 1;
            }
        } else if (winning_team.size() > 1) {
            // Draw for multiple winners
            result_filename = "draw";
            for (auto &team : winning_team) {
                result_filename += "_" + std::to_string(team);

                if (team_draws.count(team) > 0) {
                    team_draws[team] += 1;
                } else {
                    team_draws[team] = 1;
                }
            }
        } else {
            cout << "Warning" << endl;
        }

        result_filename = output_dir + "/" + result_filename + ".result";

        // Write log directory to file
        std::ofstream result_file;
        result_file.open(result_filename, std::ios::out | std::ios::app);

        // Write the parent directory of this specific simulation to the file.
        result_file << fs::path(filename).parent_path().string() << endl;
        result_file.close();

        count++;
        scrimmage::display_progress(count / static_cast<float>(number_of_runs));
    }
    cout << endl;

    double duration = (std::clock() - start) / static_cast<double>(CLOCKS_PER_SEC);
    cout << "Total time to process log files: " << duration << endl;

    // Make a map of the available team ids, so we can loop over it while
    // printing out their records.
    std::map<int, int> team_ids;
    for (auto &kv : team_wins) {
        team_ids[kv.first] = kv.first;
    }
    for (auto &kv : team_draws) {
        team_ids[kv.first] = kv.first;
    }

    int col_wid = 16;
    std::vector<std::string> headings;
    headings.push_back("Team ID");
    headings.push_back("Wins");
    headings.push_back("Draws");
    headings.push_back("Total");

    cout << "-----------------------------------------------------" << endl;
    for (auto &i : headings) {
        cout << std::left << std::setw(col_wid) << i;
    }
    cout << endl;

    for (auto &kv : team_ids) {
        int wins = 0;
        int draws = 0;

        auto it = team_wins.find(kv.first);
        if (it != team_wins.end()) wins = it->second;

        it = team_draws.find(kv.first);
        if (it != team_draws.end()) draws = it->second;

        cout << std::left << std::setw(col_wid) << kv.first;
        cout << std::left << std::setw(col_wid) << wins;
        cout << std::left << std::setw(col_wid) << draws;
        cout << std::left << std::setw(col_wid) << paths.size() << endl;
    }

    summary_file.close();
    return 0;
}
