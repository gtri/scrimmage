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

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/ConfigParse.h>

#include <iostream>
#include <fstream>
#include <vector>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/algorithm/string.hpp>

#include <rapidxml/rapidxml.hpp>

namespace fs = boost::filesystem;
namespace rx = rapidxml;

using std::cout;
using std::endl;

namespace scrimmage {

ConfigParse::ConfigParse() {}

void ConfigParse::set_required(std::string node_name) {
    required_.push_back(node_name);
}

void ConfigParse::recursive_params(rx::xml_node<char> *root,
        const std::map<std::string, std::string> &overrides,
        std::map<std::string, std::string> &params,
        const std::string &prev) {
    // End condition
    if (root == 0 || root->name() == std::string("")
        || root->name() == std::string(" ")) {
        return;
    }

    // Process this node
    std::string name = root->name();
    if (prev != "") {
        name = prev + "/" + name;
    }

    if (params.count(name) > 0) {
        // Name already exists, consider this a list. Does the size
        // parameter exist yet?
        std::string name_size = name + ":size";
        int size = 2;
        if (params.count(name_size) > 0) {
            // It does exist, grab the current size and increment.
            size = std::stoi(params[name_size]) + 1;
        }
        name = name + "_" + std::to_string(size-1);
        params[name_size] = std::to_string(size);
    }

    if (overrides.count(name)) {
        params[name] = overrides.at(name);
    } else {
        params[name] = root->value();
    }

    // Recurse (depth-first)
    recursive_params(root->first_node(), overrides, params, name);

    // Recurse (sibling node)
    recursive_params(root->next_sibling(), overrides, params, prev);
}

bool ConfigParse::parse(const std::map<std::string, std::string> &overrides,
        std::string filename, std::string env_var,
        FileSearch &file_search, bool verbose) {

    std::string result = "";
    bool status = file_search.find_file(filename, "xml", env_var, result, verbose);
    if (!status) {
        if (boost::algorithm::to_lower_copy(filename) != "sphere") {
            // sphere does not have a filename so we do not
            // need a warning message for this
            cout << "Failed to find configuration: " << filename << endl;
        }
        return false;
    } else if (verbose) {
        cout << "ConfigParse: found " << result << std::endl;
    }
    filename_ = result;

    rx::xml_document<> doc;
    std::ifstream file(filename_.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    std::string content(buffer.str());
    doc.parse<0>(&content[0]);

    rx::xml_node<> *config_node = doc.first_node("params");
    if (config_node == 0) {
        cout << "Missing tag: params" << endl;
        return false;
    }

    params_.clear();
    params_["XML_DIR"] = this->directory() + "/";
    params_["XML_FILENAME"] = filename_;

    recursive_params(config_node->first_node(), overrides, params_, "");

    // Determine if there were any overrides (XML attributes) specified in the
    // mission file that weren't declared in the Plugin's XML
    // file. Automatically add these overrides to the params block.
    for (auto &kv : overrides) {
        if (params_.count(kv.first) == 0) {
            params_[kv.first] = kv.second;
        }
    }

    for (std::string &node_name : required_) {
        if (params_.count(node_name) == 0) {
            cout << "Config file is missing XML tag: " << node_name << endl;
            return false;
        }
    }
    return true;
}

std::map<std::string, std::string> &ConfigParse::params() { return params_; }

std::string ConfigParse::filename() { return filename_; }

std::string ConfigParse::directory() {
    if (fs::exists(filename_)) {
        return fs::path(filename_).parent_path().string();
    }
    return "";
}

std::string ConfigParse::extension() {
    if (fs::exists(filename_)) {
        return fs::path(filename_).extension().string();
    }
    return "";
}

std::string ConfigParse::stem() {
    if (fs::exists(filename_)) {
        return fs::path(filename_).stem().string();
    }
    return "";
}

void ConfigParse::print_params() {
    for (auto &kv : params_) {
        cout << kv.first << "=" << kv.second << endl;
    }
}

std::ostream& operator<<(std::ostream& os, ConfigParse& cp) {
    for (auto &kv : cp.params()) {
        os << kv.first << "=" << kv.second << endl;
    }
    return os;
}
} // namespace scrimmage
