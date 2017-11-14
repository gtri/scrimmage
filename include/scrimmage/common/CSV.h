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

#ifndef INCLUDE_SCRIMMAGE_COMMON_CSV_H_
#define INCLUDE_SCRIMMAGE_COMMON_CSV_H_

#include <iostream>
#include <fstream>
#include <map>
#include <list>
#include <vector>
#include <string>
#include <utility>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

using std::cout;
using std::endl;

namespace scrimmage {

class CSV {
 public:
    typedef std::list<std::string> Headers;
    typedef std::list<std::pair<std::string, double>> Pairs;

    CSV() { }

    ~CSV() {
        this->close_output();
    }

    void set_column_headers(Headers headers, bool write = true) {
        column_headers_.clear();

        int i = 0;
        for (std::string header : headers) {
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

    void set_column_headers(std::string headers, bool write = true) {
        std::list<std::string> headers_vec = get_csv_line_elements(headers);
        set_column_headers(headers_vec, write);
    }

    template <class T>
        bool append(T pairs, bool write = true, bool keep_in_memory = false) {

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

    bool open_output(std::string filename,
                     std::ios_base::openmode mode = (std::ios_base::out |
                                                     std::ios_base::trunc)) {
        file_out_.open(filename, mode);
        return file_out_.is_open();
    }

    bool output_is_open() {
        return file_out_.is_open();
    }

    bool close_output() {
        if (file_out_.is_open()) file_out_.close();
        return !file_out_.is_open();
    }

    bool to_csv(std::string filename) {
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

    bool read_csv(std::string filename, bool contains_header = true) {
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

    void set_no_value_string(std::string str) { no_value_str_ = str; }

    int rows() { return table_.size(); }

    double at(int row, std::string header) {
        return table_[row][column_headers_[header]];
    }

 protected:
    std::list<std::string> get_csv_line_elements(std::string &str) {
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

            elems.push_back(tokens[i]);
        }
        return elems;
    }

    void write_headers() {
        // Get an ordered vector of headers
        std::vector<std::string> headers(column_headers_.size());
        for (auto &kv : column_headers_) {
            headers[kv.second] = kv.first;
        }

        unsigned int i = 0;
        for (std::string header : headers) {
            file_out_ << header;

            if (i+1 < headers.size()) {
                file_out_ << ", ";
            }

            i++;
        }
        file_out_ << endl;
    }

    void write_row(int row) {
        // Initialize a vector with no value string. Iterate over
        // column_headers, use column index to fill in columne for values.
        std::vector<std::string> values(column_headers_.size(), no_value_str_);
        for (auto &kv : table_[row]) {
            values[kv.first] = std::to_string(kv.second);
        }

        // Write out the values to the csv file
        unsigned int i = 0;
        for (std::string str : values) {
            file_out_ << str;

            if (i+1 < values.size()) {
                file_out_ << ", ";
            }
            i++;
        }
        file_out_ << endl;
    }

    // Key   : column header (name)
    // Value : column index
    std::map<std::string, int> column_headers_;

    // Key 1 : Row Index
    // Key 2 : Column Index
    // Value : Cell Value
    std::map<int, std::map<int, double>> table_;
    int next_row_ = 0;

    std::ofstream file_out_;
    std::ifstream file_in_;

    std::string no_value_str_ = "NaN";
};
} // namespace scrimmage

#endif // INCLUDE_SCRIMMAGE_COMMON_CSV_H_
