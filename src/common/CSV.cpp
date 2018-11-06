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

#include <scrimmage/common/CSV.h>

#include <iostream>
#include <vector>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

using std::cout;
using std::endl;

namespace scrimmage {

CSV::~CSV() {
    this->close_output();
}

void CSV::set_column_headers(const Headers &headers, bool write) {
    column_headers_.clear();

    int i = 0;
    for (const std::string &header : headers) {
        column_headers_[header] = i;
        i++;
    }

    if (write) {
        if (!file_out_.is_open()) {
            cout << "File isn't open. Can't write CSV headers." << endl;
        } else {
            this->write_headers();
        }
    }
}

void CSV::set_column_headers(const std::string &headers, bool write) {
    std::list<std::string> headers_vec = get_csv_line_elements(headers);
    set_column_headers(headers_vec, write);
}

bool CSV::append(const Pairs &pairs, bool write, bool keep_in_memory) {

    for (std::pair<std::string, double> pair : pairs) {
        auto it = column_headers_.find(pair.first);
        if (it == column_headers_.end()) {
            cout << "Warning: column header doesn't exist: "
                 << pair.first << endl;
        }
        table_[next_row_][it->second] = pair.second;
    }

    if (write) {
        if (!file_out_.is_open()) {
            cout << "File isn't open. Can't write CSV" << endl;
            return false;
        }
        this->write_row(next_row_);
    }

    if (keep_in_memory) next_row_++;

    return true;
}

bool CSV::open_output(const std::string &filename, std::ios_base::openmode mode) {
    file_out_.open(filename, mode);
    return file_out_.is_open();
}

bool CSV::output_is_open() {
    return file_out_.is_open();
}

bool CSV::close_output() {
    if (file_out_.is_open()) {
        file_out_.close();
    }
    return !file_out_.is_open();
}

bool CSV::to_csv(const std::string &filename) {
    this->open_output(filename);

    if (!file_out_.is_open()) {
        cout << "File isn't open. Can't write CSV" << endl;
        return false;
    }

    // Write the headers out
    this->write_headers();

    // Write all the rows out
    for (unsigned int i = 0; i < table_.size(); i++) {
        this->write_row(i);
    }
    return true;
}

bool CSV::read_csv(const std::string &filename, bool contains_header) {
    file_in_.open(filename, std::ios::in);
    if (!file_in_.is_open()) {
        cout << "Unable to open CSV file:" << filename << endl;
        return false;
    }

    table_.clear();
    column_headers_.clear();

    std::string line;
    if (contains_header) {
        // Use the first line to specify the headers
        std::getline(file_in_, line);
        std::list<std::string> headers = get_csv_line_elements(line);

        int i = 0;
        for (std::string str : headers) {
            column_headers_[str] = i;
            i++;
        }
    }

    int row = 0;
    while (std::getline(file_in_, line)) {
        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(","));
        for (unsigned int i = 0; i < tokens.size(); i++) {
            table_[row][i] = std::stod(tokens[i]);
        }

        // If this is the first line and the file doesn't contain a header,
        // populate the column headers with indices based on the number of
        // comma separated values
        if (row == 0 && !contains_header) {
            for (unsigned int i = 0; i < tokens.size(); i++) {
                column_headers_[std::to_string(i)] = i;
            }
        }

        // Print a warning if the number of comma separated values doesn't
        // match the number of columns
        if (column_headers_.size() != tokens.size()) {
            cout << "Warning the number of values (" << tokens.size()
                 << ") on line number " << row
                 << " doesn't match the number of column headers: "
                 << column_headers_.size() << endl;
        }
        row++;
    }

    return true;
}

void CSV::set_no_value_string(const std::string &str) { no_value_str_ = str; }

size_t CSV::rows() { return table_.size(); }

double CSV::at(int row, const std::string &header) {
    const int column = column_headers_.at(header);
    return table_.at(row).at(column);
}

std::list<std::string> CSV::get_csv_line_elements(const std::string &str) {
    std::list<std::string> elems;
    std::vector<std::string> tokens;
    boost::split(tokens, str, boost::is_any_of(","));
    for (unsigned int i = 0; i < tokens.size(); i++) {
        // Remove whitespace
        tokens[i].erase(
            std::remove_if(tokens[i].begin(),
                           tokens[i].end(),
                           [](unsigned char x){return std::isspace(x);}),
            tokens[i].end());

        if (tokens[i] != "") {
            elems.push_back(tokens[i]);
        }
    }
    return elems;
}

void CSV::write_headers() {
    // Get an ordered vector of headers
    std::vector<std::string> headers(column_headers_.size());
    for (auto &kv : column_headers_) {
        headers[kv.second] = kv.first;
    }

    unsigned int i = 0;
    for (std::string header : headers) {
        file_out_ << header;

        if (i+1 < headers.size()) {
            file_out_ << ",";
        }

        i++;
    }
    file_out_ << endl;
}

void CSV::write_row(int row) {
    // Initialize a vector with no value string. Iterate over
    // column_headers, use column index to fill in columne for values.
    std::vector<std::string> values(column_headers_.size(), no_value_str_);
    for (auto &kv : table_[row]) {
        if (static_cast<int64_t>(kv.second) == kv.second) {
            values[kv.first] = std::to_string(static_cast<int64_t>(kv.second));
        } else {
            values[kv.first] = std::to_string(kv.second);
        }
    }

    // Write out the values to the csv file
    unsigned int i = 0;
    for (std::string str : values) {
        file_out_ << str;

        if (i+1 < values.size()) {
            file_out_ << ",";
        }
        i++;
    }
    file_out_ << endl;
}

} // namespace scrimmage
