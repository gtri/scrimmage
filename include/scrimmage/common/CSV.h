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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

#include "scrimmage/parse/ParseUtils.h"

namespace {
struct StringifyVisitor {

    bool double_is_fixed = true;
    bool double_is_scientific = true;
    int double_precision = 13;

    std::string operator()(bool value) const { return value ? "true" : "false"; }
    std::string operator()(uint64_t value) const { return std::to_string(value); }
    std::string operator()(int64_t value) const { return std::to_string(value); }
    std::string operator()(const std::string& value) const { return value; }
    std::string operator()(double value) const {
        // default precision values for double are not enough in many cases
        std::ostringstream conv;
        if (double_is_fixed) {
            conv << std::fixed;
        }
        if (double_is_scientific) {
            conv << std::scientific;
        }
        conv << std::setprecision(double_precision) << value;
        return conv.str();
    }
};
}  // namespace

namespace scrimmage {

class CSV {
 public:
    typedef std::list<std::string> Headers;
    typedef std::variant<bool, uint64_t, int64_t, std::string, double> PossibleVariantTypes;
    typedef std::list<std::pair<std::string, PossibleVariantTypes>> Pairs;

    ~CSV();

    void set_column_headers(const Headers& headers, bool write = true);

    void set_column_headers(const std::string& headers, bool write = true);

    bool append(const Pairs& pairs, bool write = true, bool keep_in_memory = false);

    bool open_output(
        const std::string& filename,
        std::ios_base::openmode mode = (std::ios_base::out | std::ios_base::trunc));

    bool output_is_open();

    bool close_output();

    bool to_csv(const std::string& filename);

    bool read_csv(const std::string& filename, const bool& contains_header = true);

    bool read_csv_from_string(const std::string& str, const bool& contains_header = true);

    void set_no_value_string(const std::string& str);

    std::string to_string() const;

    size_t rows();

    template <class T1>
    T1 at(int row, const std::string& header) {
        const int column = column_headers_.at(header);
        return convert<T1>(table_.at(row).at(column));
    }

    friend std::ostream& operator<<(std::ostream& os, const CSV& csv);

    bool equals(const CSV& other);

    // double parameters
    void set_double_precision(int precision) { double_precision_ = precision; }
    void set_double_fixed(bool is_fixed) { double_is_fixed_ = is_fixed; }
    void set_double_scientific(bool is_scientific) { double_is_scientific_ = is_scientific; }

 protected:
    std::string get_csv_string(const PossibleVariantTypes& val) const;

    std::list<std::string> get_csv_line_elements(const std::string& str);

    void write_headers();

    void write_row(const int& row);

    // Key   : column header (name)
    // Value : column index
    std::map<std::string, int> column_headers_;

    // Key 1 : Row Index
    // Key 2 : Column Index
    // Value : Cell Value
    std::map<int, std::map<int, std::string>> table_;
    int next_row_ = 0;

    std::ofstream file_out_;
    std::string no_value_str_ = "NaN";

 private:
    int double_precision_ = 13;
    bool double_is_fixed_ = true;
    bool double_is_scientific_ = false;
    std::string headers_to_string() const;
    std::string rows_to_string() const;
    std::string row_to_string(const int& i) const;
};
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_COMMON_CSV_H_
