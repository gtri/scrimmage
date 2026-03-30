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

#include "scrimmage/common/CSV.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <scrimmage/parse/ParseUtils.h>

using std::cout;
using std::endl;

namespace scrimmage {

// Function to escape special characters in a CSV field
std::string escape_csv_field(std::string_view field) {
    // If the field contains a comma, a quote, or a newline, wrap it in quotes
    bool need_to_escape = field.find_first_of(",\"\n") != std::string::npos;
    if (!need_to_escape) {
        return std::string(field);
    }

    std::ostringstream escaped;
    escaped << '"';
    for (char c : field) {
        if (c == '"') {
            escaped << "\"\"";  // Escape double quotes as ""
        } else {
            escaped << c;
        }
    }
    escaped << '"';
    return escaped.str();
}

// Function to unescape a single CSV field
std::string unescape_csv_field(std::string_view field) {
    // If the field starts and ends with a quote, it is escaped
    bool is_quoted_string = field.size() >= 2 && field.front() == '"' && field.back() == '"';
    if (!is_quoted_string) {
        return std::string(field);
    }

    std::ostringstream unescaped;
    bool isQuotePair = false;

    // Process the part within the quotes
    for (size_t i = 1; i < field.size() - 1; ++i) {
        char c = field[i];
        if (isQuotePair) {
            if (c == '"') {  // This is the double quote -> unescape it
                unescaped << '"';
                isQuotePair = false;
            } else {
                // CSV is invalid if we find a singe quote within a single quoted field
                throw std::runtime_error(
                    "Invalid CSV: field wrapped by quotes but quote inside is not escaped properly "
                    "(ex. \"field \"\"with\"\" quote\"): "
                    + std::string(field));
            }
        } else {
            if (c == '"') {
                // First quote of a pair
                isQuotePair = true;
            } else {
                unescaped << c;  // Regular character
            }
        }
    }

    // If `isQuotePair` is still true here, it means the quotes were improperly escaped
    if (isQuotePair) {
        throw std::runtime_error("Invalid CSV: unbalanced quotes in field");
    }

    return unescaped.str();
}

// Function to split a CSV string by some token while respecting escape sequences of
// 2 double quotes """"
std::vector<std::string> split_csv_string(std::string_view row, const char token) {
    std::vector<std::string> fields;  // Stores the extracted fields
    std::ostringstream currentField;  // Accumulates characters for the current field
    bool insideQuotes = false;        // Tracks if we're inside a quoted field

    for (size_t i = 0; i < row.size(); ++i) {
        char c = row[i];

        if (insideQuotes) {
            if (c == '"') {
                // If we see a double-quote while inside quotes, check for escape (i.e., `""`)
                if (i + 1 < row.size() && row[i + 1] == '"') {
                    currentField << "\"\"";
                    ++i;  // Skip the next quote (because this is an escape sequence)
                } else {
                    // Exiting the quoted field
                    insideQuotes = false;
                    currentField << c;
                }
            } else {
                // Append any other character inside quotes
                // This is where newlines and commas can appear
                // as valid text inside a cell
                currentField << c;
            }
        } else {
            if (c == token) {
                // token outside quotes: field delimiter
                fields.push_back(currentField.str());  // Add the current field to the list
                currentField.str("");                  // Reset for the next field
                currentField.clear();                  // Clear any error flags on the object
            } else if (c == '"') {
                // Start of a quoted field
                currentField << c;
                insideQuotes = true;
            } else {
                // Regular character outside quotes
                currentField << c;
            }
        }
    }

    // Add the last field (if any)
    fields.push_back(currentField.str());

    return fields;
}

CSV::~CSV() {
    this->close_output();
}

void CSV::set_column_headers(const Headers& headers, bool write) {
    column_headers_.clear();

    int i = 0;
    for (const std::string& header : headers) {
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

void CSV::set_column_headers(std::string_view headers, bool write) {
    std::vector<std::string> headers_vec = split_csv_string(headers, ',');
    set_column_headers(headers_vec, write);
}

std::string CSV::get_csv_string(const PossibleVariantTypes& v) const {
    return std::visit(
        StringifyVisitor{
            .double_is_fixed = double_is_fixed_,
            .double_is_scientific = double_is_scientific_,
            .double_precision = double_precision_},
        v);
}

bool CSV::append(const Pairs& pairs, bool write, bool keep_in_memory) {

    for (const auto& pair : pairs) {
        auto it = column_headers_.find(pair.first);
        if (it == column_headers_.end()) {
            cout << "Warning: column header doesn't exist: " << pair.first << endl;
        }
        table_[next_row_][it->second] = escape_csv_field(get_csv_string(pair.second));
    }

    if (write) {
        if (!file_out_.is_open()) {
            cout << "File isn't open. Can't write CSV" << endl;
            return false;
        }
        this->write_row(next_row_);
    }

    if (keep_in_memory)
        next_row_++;

    return true;
}

bool CSV::open_output(const std::string& filename, std::ios_base::openmode mode) {
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
    std::ostringstream result;

    // Get an ordered vector of headers
    std::vector<std::string> headers(column_headers_.size());
    for (auto& kv : column_headers_) {
        headers[kv.second] = kv.first;
    }

    unsigned int i = 0;
    for (std::string header : headers) {
        result << escape_csv_field(header);

        if (i + 1 < headers.size()) {
            result << ",";
        }
        i++;
    }
    return result.str();
}

std::string CSV::rows_to_string() const {
    std::ostringstream result;
    // Write all the rows out
    for (unsigned int i = 0; i < table_.size(); i++) {
        result << this->row_to_string(i);
        if (i < table_.size() - 1) {
            result << "\n";
        }
    }
    return result.str();
}

std::string CSV::row_to_string(const int& row) const {
    std::ostringstream result;

    auto it_row = table_.find(row);
    if (it_row == table_.end()) {
        return "";
    }

    // Append the rows to the resultant string
    unsigned int i = 0;
    for (auto& kv : it_row->second) {
        result << escape_csv_field(kv.second);

        if (i + 1 < it_row->second.size()) {
            result << ",";
        }
        i++;
    }
    return result.str();
}

bool CSV::to_csv(const std::string& filename) {
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

bool CSV::read_csv_from_string(const std::string& csv_str, const bool& contains_header) {
    table_.clear();
    column_headers_.clear();

    std::vector<std::string> line_tokens = split_csv_string(csv_str, '\n');
    int row_num = 0;
    for (unsigned int line_num = 0; line_num < line_tokens.size(); line_num++) {
        std::string line = line_tokens[line_num];

        if (line == "")
            continue;  // Ignore lines that are empty

        if (line_num == 0 && contains_header) {
            std::vector<std::string> headers = split_csv_string(line, ',');
            int i = 0;
            for (const auto& str : headers) {
                column_headers_[unescape_csv_field(str)] = i;
                i++;
            }
        } else {
            std::vector<std::string> tokens = split_csv_string(line, ',');
            for (unsigned int i = 0; i < tokens.size(); i++) {
                table_[row_num][i] = unescape_csv_field(tokens[i]);
            }

            // If this is the first line and the file doesn't contain a header,
            // populate the column headers with indices based on the number of
            // comma separated values
            if (row_num == 0 && !contains_header) {
                for (unsigned int i = 0; i < tokens.size(); i++) {
                    column_headers_[std::to_string(i)] = i;
                }
            }

            // Print a warning if the number of comma separated values doesn't
            // match the number of columns
            if (column_headers_.size() != tokens.size()) {
                cout << "Warning the number of values (" << tokens.size() << ") on line number "
                     << row_num
                     << " doesn't match the number of column headers: " << column_headers_.size()
                     << endl;
            }
            row_num++;
        }
    }
    return true;
}

bool CSV::read_csv(const std::string& filename, const bool& contains_header) {
    std::ifstream file(filename);
    if (not file.is_open()) {
        cout << "Unable to open CSV file:" << filename << endl;
        return false;
    }

    // Read in the CSV file as a string and parse it
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return read_csv_from_string(str, contains_header);
}

void CSV::set_no_value_string(const std::string& str) {
    no_value_str_ = str;
}

size_t CSV::rows() {
    return table_.size();
}

void CSV::write_headers() {
    file_out_ << headers_to_string() << endl;
}

void CSV::write_row(const int& row) {
    file_out_ << row_to_string(row) << endl;
}

bool CSV::equals(const CSV& other) {
    if (not std::equal(
            column_headers_.begin(),
            column_headers_.end(),
            other.column_headers_.begin(),
            other.column_headers_.end())) {
        return false;
    }

    if (not std::equal(table_.begin(), table_.end(), other.table_.begin(), other.table_.end())) {
        return false;
    }

    return true;
}

}  // namespace scrimmage
