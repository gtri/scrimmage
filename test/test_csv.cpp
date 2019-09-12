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

#include <gtest/gtest.h>

#include <scrimmage/common/CSV.h>
#include <scrimmage/parse/ParseUtils.h>

#include <string>
#include <vector>
#include <cmath>

namespace sc = scrimmage;

class CSVTest : public ::testing::Test {
 protected:
    void SetUp() override {
        // Construct the CSV string
        for (auto &line : csv_lines) {
            csv_str += line + "\n";
        }
    }
    std::string csv_str = "";
    std::vector<std::string> csv_lines = {
        "my_test_bool,my_test_int,my_test_float,my_test_double",
        "1,1,2.100000,99.987000",
        "0,1,2.100000,99.987000",
        "0,2,2.100000,99.987000",
        "0,2,3.200000,99.987000",
        "0,2,3.200000,98.987000"};
};

std::list<sc::CSV::Pairs> construct_list_of_pairs(const std::vector<std::string> &csv_lines) {
    EXPECT_GT(csv_lines.size(), static_cast<unsigned int>(0));

    // Use the csv_lines list of strings to append to the CSV object
    std::list<sc::CSV::Pairs> list_of_pairs;
    std::vector<std::string> headers = sc::str2container<std::vector<std::string>>(csv_lines[0], ",");
    for (unsigned int i = 1; i < csv_lines.size(); i++) {
        std::vector<std::string> items = sc::str2container<std::vector<std::string>>(csv_lines[i], ",");
        EXPECT_EQ(headers.size(), items.size());

        sc::CSV::Pairs pairs;
        for (unsigned int k = 0; k < headers.size(); k++) {
            pairs.push_back(std::make_pair(headers[k], std::stod(items[k])));
        }
        list_of_pairs.push_back(pairs);
    }
    return list_of_pairs;
}

TEST_F(CSVTest, from_string) {
    std::string filename = "from_string.csv";

    // Create a CSV file from a string
    sc::CSV csv_from_string;
    EXPECT_TRUE(csv_from_string.read_csv_from_string(csv_str));

    // Write it to a file
    EXPECT_TRUE(csv_from_string.to_csv(filename));

    // Read the CSV file into a new CSV object
    sc::CSV csv_from_file;
    EXPECT_TRUE(csv_from_file.read_csv(filename));

    // The two CSV files should be equal to each other
    EXPECT_TRUE(csv_from_string.equals(csv_from_file));
}

TEST_F(CSVTest, from_append) {
    ASSERT_GT(csv_lines.size(), static_cast<unsigned int>(0));

    // Create a CSV file using the recommended plugin API (pairs):
    std::string filename("from_append.csv");
    sc::CSV csv;
    csv.open_output(filename);
    csv.set_column_headers(csv_lines[0]); // Write the column headers

    // Create a list of pairs to pass to CSV
    auto list_of_pairs = construct_list_of_pairs(csv_lines);
    for (auto &pairs : list_of_pairs) {
        csv.append(pairs, true, true);
    }
    csv.close_output();

    // Read the CSV file and compare to other CSV objects
    sc::CSV csv_from_file_2;
    EXPECT_TRUE(csv_from_file_2.read_csv(filename));

    // Compare against other CSV objects.
    EXPECT_TRUE(csv_from_file_2.equals(csv));
}
