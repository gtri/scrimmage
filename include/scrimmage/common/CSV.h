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
#include <map>
#include <list>
#include <string>
#include <utility>
#include <memory>

namespace scrimmage {

class CSV {
 public:
    typedef std::list<std::string> Headers;
    typedef std::list<std::pair<std::string, double>> Pairs;

    ~CSV();

    void set_column_headers(const Headers &headers, bool write = true);

    void set_column_headers(const std::string &headers, bool write = true);

    bool append(const Pairs &pairs, bool write = true, bool keep_in_memory = false);

    bool open_output(const std::string &filename,
                     std::ios_base::openmode mode = (std::ios_base::out |
                                                     std::ios_base::trunc));

    bool output_is_open();

    bool close_output();

    bool to_csv(const std::string &filename);

    bool read_csv(const std::string &filename, bool contains_header = true);

    void set_no_value_string(const std::string &str);

    size_t rows();

    double at(int row, const std::string &header);

 protected:
    std::list<std::string> get_csv_line_elements(const std::string &str);

    void write_headers();

    void write_row(int row);

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
