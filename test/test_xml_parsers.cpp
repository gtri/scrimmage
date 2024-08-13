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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <regex>

#include <scrimmage/parse/XMLParser/XMLParser.h>
#include <scrimmage/parse/XMLParser/LibXML2Parser.h>
#include <scrimmage/parse/XMLParser/RapidXMLParser.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

// Enum hack to get around being unable to type parameterize with CRTP
enum ParserType {
  RAPIDXML,
  LIB2XML
};

namespace fs = boost::filesystem;

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

      test_file = xml_file.parent_path();
      test_file /= xml_file.stem().concat("_test.xml");
      ASSERT_FALSE(fs::exists(test_file));
    }

    void TearDown() override {
      if(fs::exists(test_file)) {
        fs::remove(test_file);
      }
    }

    fs::path xml_file;
    fs::path test_file;
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
    auto child_start = parent.child_begin();
    auto child_end = parent.child_end(); 
    std::size_t i = 0;
    for(auto it = child_start; it != child_end; ++it) {
      check_node<Parser>(child_node, expected_names.at(i++));
      ASSERT_EQ(child_node, *it);
      child_node = child_node.next_sibling();
    }
    ASSERT_EQ(i, expected_names.size());
  };

  Parser parser;
  parser.parse(xml_file.string());

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
  parser.parse(xml_file.string());

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

template <typename Actual, typename Expected>
testing::AssertionResult AreNodesEqual(Actual actual, Expected expected) {
  auto remove_whitespace = [](std::string s) -> std::string {
    auto end_it = std::remove_if(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
    s.erase(end_it, s.end());
    return s;
  };

  // We can sometimes get differences in whitespace. We don't really care about that
  std::string a_name = remove_whitespace(actual.name());
  std::string a_value = remove_whitespace(actual.value());
  std::string e_name = remove_whitespace(expected.name());
  std::string e_value = remove_whitespace(expected.value());

  bool same_name = a_name == e_name;
  bool same_value = a_value == e_value;

  if (same_name && same_value) { return testing::AssertionSuccess(); }

  //// We want to print out actual values, i.e. string literals.
  return testing::AssertionFailure() << "Expected Name: \'" << e_name << "\', Value: \'" << e_value 
    << "\'. Actual has Name: \'" << a_name << "\', Value: \'" << a_value << "\'";
}

template<class ParserA, class ParserB>
inline void test_xml_out(const fs::path& original_file, const fs::path& test_file) {
  using NodeA = typename ParserA::XMLNode;
  using NodeB = typename ParserB::XMLNode;

  using AttributeA = typename NodeA::XMLAttribute;
  using AttributeB = typename NodeB::XMLAttribute;

  ParserA parserA;
  ParserB parserB;
  parserA.parse(original_file.string());


  std::ofstream out;
  out.open(test_file.string());
  ASSERT_TRUE(fs::exists(test_file));
  out << parserA;  
  out.close();

  parserB.parse(test_file.string());

  auto recur_itA = parserA.recur_begin();
  auto recur_itB = parserB.recur_begin();


    for(; recur_itA != parserA.recur_end() || recur_itB != parserB.recur_end(); ++recur_itA, ++recur_itB) {
      auto attr_itA = recur_itA->attr_begin();
      auto attr_itB = recur_itB->attr_begin();

      testing::AssertionResult NodesEqual = AreNodesEqual<NodeB, NodeA>(*recur_itB, *recur_itA);
      EXPECT_TRUE(NodesEqual);
  
      for(; attr_itA != recur_itA->attr_end() && attr_itB != recur_itB->attr_end(); ++attr_itA, ++attr_itB) {
        testing::AssertionResult AttributesEqual = AreNodesEqual<AttributeB, AttributeA>(*attr_itB, *attr_itA);
        EXPECT_TRUE(AttributesEqual);
      }
  
      ASSERT_EQ(attr_itA, recur_itA->attr_end());
      ASSERT_EQ(attr_itB, recur_itB->attr_end());
    }
  
    ASSERT_EQ(recur_itA, parserA.recur_end());
    ASSERT_EQ(recur_itB, parserB.recur_end());
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

TEST_P(XMLParserTest, test_xml_out) {
  switch(pt) {
    case ParserType::RAPIDXML:
      test_xml_out<scrimmage::RapidXMLParser, scrimmage::RapidXMLParser>(xml_file, test_file);
      break;
    case ParserType::LIB2XML:
      test_xml_out<scrimmage::LibXML2Parser, scrimmage::LibXML2Parser>(xml_file, test_file);
      break;
    default:
      FAIL();
  }
}

INSTANTIATE_TEST_SUITE_P(XMLParserTestSuite, XMLParserTest, 
    testing::Values(ParserType::RAPIDXML, ParserType::LIB2XML)
    );
