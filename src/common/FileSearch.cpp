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

#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <unordered_set>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/range/algorithm/find_if.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/adaptor/uniqued.hpp>
#include <boost/tokenizer.hpp>
#include <boost/optional.hpp>

namespace fs = ::boost::filesystem;
namespace ba = boost::adaptors;
namespace br = boost::range;

namespace scrimmage {

void FileSearch::clear() {cache_.clear();}

boost::optional<std::string> FileSearch::find_mission(std::string mission,
        bool verbose) {
    if (!boost::algorithm::ends_with(mission, "xml")) {
        mission = mission + ".xml";
    }
    mission = expand_user(mission);
    if (fs::exists(mission)) return mission;

    std::string out;
    auto search = [&](auto env) {return this->find_file(
            mission, "xml", env, out, verbose);};
    std::list<std::string> env_vars
        {"SCRIMMAGE_MISSION_PATH",
            "/usr/share/scrimmage",
            "/usr/local/share/scrimmage"};
    auto it = br::find_if(env_vars, search);
    return it ==
        env_vars.end() ? boost::none : boost::optional<std::string>(out);
}

bool FileSearch::find_file(const std::string &search,
        std::string ext, const std::string &env_var, std::string &result,
        bool verbose) {
    // Find the xml file.
    // Search order:
    // 1. search could be the full path
    // 2. search could be just the name of the plugin (e.g., QuadTest_plugin)
    //    a. Is the file located in the plugin path environment variable?

    // Determine if the passed in file exists (could be full path)
    std::list<std::string> filenames;
    if (!ext.empty() && ext[0] != '.') {
        ext = std::string(".") + ext;
    }
    std::string search_filename = search;

    // If the search term already ends in the extension, don't add the extension
    // Otherwise, add the extension
    if (!boost::algorithm::ends_with(search, ext)) {
        search_filename = search + ext;
    }

    auto dbg = [&](std::string msg) {
        if (verbose) std::cout << "find_file: " << msg << std::endl;
    };
    dbg(std::string("looking for ") + search_filename);

    if (!fs::exists(search)) {
        // files[search_filename] = list of full paths
        dbg(std::string("not an absolute path, checking recursively in ")
                + env_var);
        std::unordered_map<std::string, std::list<std::string>> files;
        find_files(env_var, ext, files, verbose);
        filenames = files[search_filename];
    } else {
        filenames.push_back(search);
    }

    if (filenames.empty()) {
        dbg(std::string("Failed to find xml filename: ") + search);
        return false;
    }

    // Use the last XML file that was found
    result = filenames.back();

    if (filenames.size() > 1) {
        std::cout <<
            "===============================================" << std::endl;
        std::cout <<
            "WARNING: Multiple XML files with same name found" << std::endl;
        for (std::string &full_path : filenames) {
            std::cout << full_path << std::endl;
        }
        std::cout << "Using XML file at: " << result << std::endl;
        std::cout <<
            "===============================================" << std::endl;
    }
    return true;
}

// Give an environment variable and a file extension:
// Find the absolute path to all files in environment variable paths
void FileSearch::find_files(std::string env_var, const std::string &ext,
        std::unordered_map<std::string, std::list<std::string>> &out,
        bool verbose) {
    auto dbg = [&](std::string msg) {
        if (verbose) std::cout << "find_files: " << msg << std::endl;
    };

    auto cache_it = cache_.find(env_var);
    auto ext_it = cache_[env_var].find(ext);
    if (cache_it != cache_.end()) {
        if (ext_it != cache_it->second.end()) {
            out = ext_it->second;
            return;
        } else {
            cache_it->second[ext];
            ext_it = cache_it->second.find(ext);
        }
    } else {
        cache_[env_var][ext];
        ext_it = cache_[env_var].find(ext);
    }

    out.clear();

    // Get the environment variable
    std::string env_path;
    if (env_var.find("/") == std::string::npos) {
        const char* env_p = std::getenv(env_var.c_str());
        if (env_p == NULL) {
            std::cout << env_var <<
                " environment variable not set" << std::endl;
            return;
        }

        env_path = std::string(env_p);
        dbg(env_var + " = " + env_path);
    } else {
        // assume is is a path rather than an environment variable
        // since slashes cannot be in an environment variable
        env_path = env_var;
    }

    // Tokenize the path and loop through each directory
    boost::char_separator<char> sep(":");
    boost::tokenizer< boost::char_separator<char> > tokens(env_path, sep);

    // we are checking recursively anyway so remove
    // more specific directories
    std::vector<std::string> tok(tokens.begin(), tokens.end());
    br::sort(tok);
    auto it = tok.begin();

    char native_path_sep = fs::path("/").make_preferred().native().at(0);

    while (it != tok.end()) {
        auto starts_with = [&](std::string &s) {
            return (*it == s) ||
                (boost::starts_with(s, *it) &&
                s.size() > it->size() &&
                s.at(it->size()) == native_path_sep);
        };
        tok.erase(std::remove_if(std::next(it), tok.end(), starts_with),
                tok.end());
        it++;
    }

    dbg(std::string("not found in cache, looping recursively in ") + env_path);
    for (const std::string &t : tok) {
        // Search for all files in the current directory with
        // the extension
        fs::path root = t;

        if (fs::exists(root) && fs::is_directory(root)) {
            dbg(t);

            fs::recursive_directory_iterator it(root);
            fs::recursive_directory_iterator endit;

            while (it != endit) {
                fs::path path = it->path();
                if (fs::is_regular_file(*it) && path.extension() == ext) {
                    std::string fname = path.filename().string();
                    std::string full_path = fs::absolute(path).string();
                    dbg(std::string("   ") + fname);
                    ext_it->second[fname].push_back(full_path);
                }
                ++it;
            }
        } else if (env_path != env_var) {
            std::cout << "Search path doesn't exist: " << t << std::endl;
        }
    }

    out = ext_it->second;
    return;
}

}  // namespace scrimmage
