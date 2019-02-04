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
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/plugin_manager/PluginManager.h>

#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>
#include <list>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

using std::cout;
using std::endl;

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace sc = scrimmage;

#ifdef __APPLE__
#define LIB_EXT ".dylib"
#else
#define LIB_EXT ".so"
#endif

int main(int argc, char *argv[]) {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("plugin-name,p", po::value<std::string>(), "the plugin name")
        ("verbose,v", "increase debugging output");

    po::positional_options_description p;
    p.add("plugin-name", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    std::map<std::string, std::string> overrides;
    std::string plugin_name_xml;
    if (vm.count("plugin-name")) {
        plugin_name_xml = vm["plugin-name"].as<std::string>();
    } else {
        cout << "plugin-name was not set.\n";
        return -1;
    }

    bool verbose = vm.count("verbose");
    sc::FileSearch file_search;
    sc::ConfigParse config_parse;
    config_parse.set_required("library");
    if (!config_parse.parse(overrides, plugin_name_xml, "SCRIMMAGE_PLUGIN_PATH", file_search, verbose)) {
        const std::string plugin_type = "autonomy";
        std::cout << "Failed to parse: " << plugin_name_xml << " for type " << plugin_type << std::endl;
        return -2;
    }

    std::string library_name = config_parse.params()["library"];

    cout << "==========================================" << endl;
    cout << "Plugin found." << endl;
    cout << "Name: " << plugin_name_xml << endl;
    cout << "File: " << config_parse.filename() << endl;
    cout << "Library: " << library_name << endl;
    cout << "-------------------------" << endl;
    cout << "Params: ";
    config_parse.print_params();

    // Find the paths to the libraries
    std::unordered_map<std::string, std::list<std::string>> so_files;
    file_search.find_files("SCRIMMAGE_PLUGIN_PATH", LIB_EXT, so_files);

    scrimmage::PluginManager plugin_mgr;
    std::list<std::string> plugins_found;
    plugin_mgr.find_matching_plugins(library_name, so_files, plugins_found);

    cout << "---------------------------" << endl;
    cout << "Matching plugin libraries found: " << endl;
    for (const std::string &str  : plugins_found) {
        cout << str << endl;
    }
    return 0;
}
