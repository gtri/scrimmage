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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 25 June 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */


#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>

#include <filesystem>

#include <scrimmage/parse/XMLParser/XMLParser.h>
#include <scrimmage/parse/XMLParser/LibXML2Parser.h>
#include <scrimmage/parse/XMLParser/RapidXMLParser.h>

// Enum hack to get around being unable to type parameterize with CRTP
enum ParserType {
  RAPIDXML,
  LIB2XML
};

namespace fs = std::filesystem;

class XMLParserTest : public testing::TestWithParam<ParserType> {

  protected:
    void SetUp() override {
      pt = GetParam();
      char* scrimmage_missions = std::getenv("SCRIMMAGE_MISSION_PATH");
      ASSERT_NE(scrimmage_missions, nullptr);
      char* path = std::strtok(scrimmage_missions, ":"); // Path is ":" delimited
      bool mission_file_found = false;
      while(path != nullptr && !mission_file_found) {
        xml_file = std::string{path};
        xml_file /= "test/simple.xml";
        mission_file_found = fs::exists(xml_file);
        path = std::strtok(nullptr, ":");
      }
      ASSERT_TRUE(mission_file_found); 
    }

    fs::path xml_file;
    ParserType pt;
};

template<class Parser>
inline void check_node(typename Parser::XMLNode& node, const std::string& expected_name) {
  ASSERT_TRUE(node.is_valid());
  ASSERT_STREQ(node.name().c_str(), expected_name.c_str());
}

template<typename Parser>
inline void test_sibling_iterator(fs::path xml_file) {
  auto check_sibling_iterator = [](typename Parser::XMLNode parent, const std::vector<std::string>& expected_names) {
    auto child_node = parent.first_node();
    auto child_start = parent.begin();
    auto child_end = parent.end(); 
    std::size_t i = 0;
    for(auto it = child_start; it != child_end; ++it) {
      check_node<Parser>(child_node, expected_names.at(i++));
      ASSERT_EQ(child_node, *it);
      child_node = child_node.next_sibling();
    }
    ASSERT_EQ(i, expected_names.size());
  };

  Parser parser;
  parser.parse(xml_file);

  // Test ROOT NODE
  auto root_node = parser.first_node();
  check_node<Parser>(root_node, "root");
  auto root_start = parser.begin();
  auto root_end = parser.end(); 

  ASSERT_EQ(*root_start, root_node); 
  ASSERT_EQ(++root_start, root_end);

  // Test First Level Children Nodes
  std::vector<std::string> expected_child_names = {"first_child", "second_child", "third_child"};
  check_sibling_iterator(root_node, expected_child_names);

  // Test Second Level Children Nodes
  auto first_child = root_node.first_node();
  auto second_child = first_child.next_sibling();
  auto third_child = second_child.next_sibling();

  check_sibling_iterator(first_child, {"first_grand_child", "second_grand_child"});
  check_sibling_iterator(second_child, {"third_grand_child"});
  check_sibling_iterator(third_child, std::vector<std::string>{});
}

template<class Parser>
inline void test_recursive_iterator(const fs::path xml_file) {
  Parser parser;
  parser.parse(xml_file);

  std::vector<std::string> expected_names = {
    "root",
    "first_child",
    "first_grand_child",
    "second_grand_child",
    "second_child",
    "third_grand_child",
    "first_grand_grand_child",
    "third_child"
  };

  std::size_t i = 0;
  for(auto it = parser.recur_begin(); it != parser.recur_end(); ++it) {
    check_node<Parser>(*it, expected_names.at(i++));
  }
  ASSERT_EQ(i, expected_names.size());
}

TEST_P(XMLParserTest, test_parse) {
  switch(pt) {
    case ParserType::RAPIDXML: 
      test_sibling_iterator<scrimmage::RapidXMLParser>(xml_file);
      test_recursive_iterator<scrimmage::RapidXMLParser>(xml_file);
      break;
    case ParserType::LIB2XML: 
      test_sibling_iterator<scrimmage::LibXML2Parser>(xml_file);
      test_recursive_iterator<scrimmage::LibXML2Parser>(xml_file);
      break;
    default:
      FAIL();
  }
}


INSTANTIATE_TEST_SUITE_P(XMLParserTestSuite, XMLParserTest, 
    testing::Values(ParserType::RAPIDXML, ParserType::LIB2XML)
    );
