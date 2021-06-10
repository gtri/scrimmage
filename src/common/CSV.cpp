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
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <vector>
#include <fstream>

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

std::string CSV::to_string() const {
    return headers_to_string() + "\n" + rows_to_string();
}

std::ostream& operator<<(std::ostream& os, const CSV& csv) {
    os << csv.to_string();
    return os;
}

std::string CSV::headers_to_string() const {
    std::string result = "";

    // Get an ordered vector of headers
    std::vector<std::string> headers(column_headers_.size());
    for (auto &kv : column_headers_) {
        headers[kv.second] = kv.first;
    }

    unsigned int i = 0;
    for (std::string header : headers) {
        result += header;

        if (i+1 < headers.size()) {
            result += ",";
        }
        i++;
    }
    return result;
}

std::string CSV::rows_to_string() const {
    std::string result = "";
    // Write all the rows out
    for (unsigned int i = 0; i < table_.size(); i++) {
        result += this->row_to_string(i);
        if (i < table_.size() - 1) {
            result += "\n";
        }
    }
    return result;
}

std::string CSV::row_to_string(const int& row) const {
    std::string result = "";

    // Initialize a vector with no value string. Iterate over
    // column_headers, use column index to fill in columne for values.
    std::vector<std::string> values(column_headers_.size(), no_value_str_);
    auto it_row = table_.find(row);
    for (auto &kv : it_row->second) {
        if (static_cast<int64_t>(kv.second) == kv.second) {
            values[kv.first] = std::to_string(static_cast<int64_t>(kv.second));
        } else {
            values[kv.first] = std::to_string(kv.second);
        }
    }

    // Append the rows to the resultant string
    unsigned int i = 0;
    for (std::string str : values) {
        result += str;

        if (i+1 < values.size()) {
            result += ",";
        }
        i++;
    }
    return result;
}


bool CSV::to_csv(const std::string &filename) {
    if (not this->open_output(filename)) {
        cout << "CSV::to_csv: Failed to open file: " << filename << endl;
        return false;
    }

    // Write the headers out
    this->write_headers();

    // Write all the rows out
    for (unsigned int i = 0; i < table_.size(); i++) {
        this->write_row(i);
    }
    if (not this->close_output()) {
        cout << "Failed to close CSV file." << endl;
    }
    return true;
}

bool CSV::read_csv_from_string(const std::string &csv_str, const bool& contains_header) {
    table_.clear();
    column_headers_.clear();

    std::vector<std::string> line_tokens;
    boost::split(line_tokens, csv_str, boost::is_any_of("\n"));
    int row_num = 0;
    for (unsigned int line_num = 0; line_num < line_tokens.size(); line_num++) {
        // Strip "whitespace" characters.
        std::string line = remove_whitespace(line_tokens[line_num]);

        if (line == "") continue; // Ignore lines that are empty

        if (line_num == 0 && contains_header) {
            std::list<std::string> headers = get_csv_line_elements(line);
            int i = 0;
            for (auto &str : headers) {
                column_headers_[str] = i;
                i++;
            }
        } else {
            std::vector<std::string> tokens;
            boost::split(tokens, line, boost::is_any_of(","));
            for (unsigned int i = 0; i < tokens.size(); i++) {
                table_[row_num][i] = std::stod(tokens[i]);
            }

            // If this is the first line and the file doesn't contain a header,
            // populate the column headers with indices based on the number of
            // comma separated values
            if (row_num == 0 && not contains_header) {
                for (unsigned int i = 0; i < tokens.size(); i++) {
                    column_headers_[std::to_string(i)] = i;
                }
            }

            // Print a warning if the number of comma separated values doesn't
            // match the number of columns
            if (column_headers_.size() != tokens.size()) {
                cout << "Warning the number of values (" << tokens.size()
                     << ") on line number " << row_num
                     << " doesn't match the number of column headers: "
                     << column_headers_.size() << endl;
            }
            row_num++;
        }
    }
    return true;
}

bool CSV::read_csv(const std::string &filename, const bool& contains_header) {
    std::ifstream file(filename);
    if (not file.is_open()) {
        cout << "Unable to open CSV file:" << filename << endl;
        return false;
    }

    // Read in the CSV file as a string and parse it
    std::string str((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
    return read_csv_from_string(str, contains_header);
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
    file_out_ << headers_to_string() << endl;
}

void CSV::write_row(const int& row) {
    file_out_ << row_to_string(row) << endl;
}

bool CSV::equals(const CSV& other) {
    if (not std::equal(column_headers_.begin(), column_headers_.end(),
                       other.column_headers_.begin(),
                       other.column_headers_.end())) {
        return false;
    }


    if (not std::equal(table_.begin(), table_.end(),
                       other.table_.begin(),
                       other.table_.end())) {
        return false;
    }

    return true;
}

} // namespace scrimmage
