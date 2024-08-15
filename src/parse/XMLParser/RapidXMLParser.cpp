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
 * @author Wesley Ford <wesley.ford@gtri.gatech.edu>
 * @date March 8 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 *
 * A general interface for XML parsing used by MissionParse
 * similar to RapidXML interface
 */

#include <scrimmage/parse/XMLParser/RapidXMLParser.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <rapidxml/rapidxml.hpp>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

namespace fs = boost::filesystem;

namespace scrimmage {

RapidXMLParserAttribute::RapidXMLParserAttribute(rapidxml::xml_attribute<> *attribute)
    : attribute_(attribute) {}

RapidXMLParserAttribute RapidXMLParserAttribute::next_attribute(const std::string &name) const {
  return RapidXMLParserAttribute{attribute_->next_attribute(name.c_str())};
}

RapidXMLParserAttribute RapidXMLParserAttribute::next_attribute() const {
  return RapidXMLParserAttribute{attribute_->next_attribute()};
}

std::string RapidXMLParserAttribute::attribute_name() const {
  if (!this->is_valid() || attribute_->value_size() == 0) {
    return "";
  }
  return std::string{attribute_->name(), attribute_->name_size()};
}

std::string RapidXMLParserAttribute::attribute_value() const {
  // Values may not always be null-terminated, depending of parsing settings
  if (!this->is_valid() || attribute_->value_size() == 0) {
    return "";
  }
  return std::string{attribute_->value(), attribute_->value_size()};
}

bool RapidXMLParserAttribute::is_valid_attribute() const { return attribute_ != nullptr; }

RapidXMLParserNode::RapidXMLParserNode(rapidxml::xml_node<> *node)
    : node_(node) {}

RapidXMLParserNode RapidXMLParserNode::get_first_node(const std::string &name) const {
  return RapidXMLParserNode{node_->first_node(name.c_str())};
}

RapidXMLParserNode RapidXMLParserNode::get_first_node() const {
  return RapidXMLParserNode{node_->first_node()};
}

RapidXMLParserNode RapidXMLParserNode::get_next_sibling(const std::string &name) const {
  return RapidXMLParserNode{node_->next_sibling(name.c_str())};
}

RapidXMLParserNode RapidXMLParserNode::get_next_sibling() const {
  return RapidXMLParserNode{node_->next_sibling()};
}

RapidXMLParserAttribute RapidXMLParserNode::get_first_attribute(const std::string &name) const {
  return RapidXMLParserAttribute{node_->first_attribute(name.c_str())};
}

RapidXMLParserAttribute RapidXMLParserNode::get_first_attribute() const {
  return RapidXMLParserAttribute{node_->first_attribute()};
}

std::string RapidXMLParserNode::node_name() const {
  if (!this->is_valid() || node_->name_size() == 0) {
    return std::string{""};
  }
  return std::string{node_->name(), node_->name_size()};
}

std::string RapidXMLParserNode::node_value() const {
  if (!this->is_valid() || node_->value_size() == 0) {
    return std::string{""};
  }
  return std::string{node_->value(), node_->value_size()};
}

bool RapidXMLParserNode::is_valid_node() const { return node_ != nullptr; }

bool RapidXMLParserDocument::parse_document(const std::string &filename) {
  fs::path path{filename};
  if (fs::is_symlink(path)) {
    path = fs::read_symlink(path);
  }
  assert(fs::exists(path) && fs::is_regular_file(path));
  std::ifstream file(path.string());
  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();
  std::string filecontent_str = buffer.str();
  std::vector<char> filecontent(filecontent_str.cbegin(), filecontent_str.cend());
  filename_ = path.string();
  return parse_document(filecontent);
}

bool RapidXMLParserDocument::parse_document(const std::vector<char> &filecontent) {
  filecontent_ = filecontent;
  try {
    // Note: This parse function can hard fail (seg fault, no exception) on
    //       badly formatted xml data. Sometimes it'll except, sometimes not.
    doc_.parse<rapidxml::parse_non_destructive>(filecontent_.data());
  } catch (...) {
    std::cerr << "scrimmage::MissionParse::parse: Exception during "
                 "rapidxml::xml_document<>.parse<>()."
                 "Your xml mission file may be ill-formatted\n";
    return false;
  }
  return true;
}

RapidXMLParserNode RapidXMLParserDocument::find_first_node(const std::string &name) const {
  return RapidXMLParserNode{doc_.first_node(name.c_str())};
}

RapidXMLParserNode RapidXMLParserDocument::find_first_node() const {
  return RapidXMLParserNode{doc_.first_node()};
}

}  // namespace scrimmage
