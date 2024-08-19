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
 * similar to LibXML2 interface
 */

#include <scrimmage/parse/XMLParser/LibXML2Parser.h>

#include <libxml/tree.h>
#include <libxml/xinclude.h>
#include <libxml/xmlreader.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

namespace fs = boost::filesystem;

namespace scrimmage {
LibXML2ParserAttribute::LibXML2ParserAttribute(xmlAttrPtr attribute)
    : attribute_(attribute) {}

LibXML2ParserAttribute LibXML2ParserAttribute::next_attribute(const std::string &name) const {
  xmlAttrPtr cur_attr = nullptr;
  const xmlChar *search_name = reinterpret_cast<const xmlChar *>(name.data());
  for (cur_attr = attribute_->next; cur_attr != nullptr; cur_attr = cur_attr->next) {
    if (cur_attr->type == XML_ATTRIBUTE_NODE && xmlStrEqual(search_name, cur_attr->name)) {
      break;
    }
  }
  return LibXML2ParserAttribute{cur_attr};
}

LibXML2ParserAttribute LibXML2ParserAttribute::next_attribute() const {
  return LibXML2ParserAttribute{attribute_->next};
}

std::string LibXML2ParserAttribute::attribute_name() const {
  return std::string{reinterpret_cast<const char *>(attribute_->name)};
}

std::string LibXML2ParserAttribute::attribute_value() const {
  return std::string{reinterpret_cast<const char *>(
      xmlNodeListGetString(attribute_->doc, attribute_->children, 1))};
}

bool LibXML2ParserAttribute::is_valid_attribute() const { return attribute_ != nullptr; }

LibXML2ParserNode::LibXML2ParserNode(xmlNodePtr node)
    : node_(node) {}

LibXML2ParserNode LibXML2ParserNode::get_first_node(const std::string &name) const {
  xmlNode *cur_node = nullptr;
  const xmlChar *search_name = reinterpret_cast<const xmlChar *>(name.data());
  if (is_valid_node()) {
    for (cur_node = node_->children; cur_node != nullptr; cur_node = cur_node->next) {
      if (cur_node->type == XML_ELEMENT_NODE && xmlStrEqual(search_name, cur_node->name)) {
        break;
      }
    }
  }
  return LibXML2ParserNode{cur_node};
}

LibXML2ParserNode LibXML2ParserNode::get_first_node() const {
  xmlNode *cur_node = nullptr;
  if (is_valid_node()) {
    for (cur_node = node_->children; cur_node != nullptr; cur_node = cur_node->next) {
      if (cur_node->type == XML_ELEMENT_NODE) {
        break;
      }
    }
  }
  return LibXML2ParserNode{cur_node};
}

LibXML2ParserNode LibXML2ParserNode::get_next_sibling(const std::string &name) const {
  xmlNodePtr cur_node = nullptr;
  const xmlChar *search_name = reinterpret_cast<const xmlChar *>(name.data());
  for (cur_node = node_->next; cur_node != nullptr; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE && xmlStrEqual(search_name, cur_node->name)) {
      break;
    }
  }
  return LibXML2ParserNode{cur_node};
}

LibXML2ParserNode LibXML2ParserNode::get_next_sibling() const {
  xmlNodePtr cur_node = nullptr;
  for (cur_node = node_->next; cur_node != nullptr; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE) {
      break;
    }
  }
  return LibXML2ParserNode{cur_node};
}

LibXML2ParserAttribute LibXML2ParserNode::get_first_attribute(const std::string &name) const {
  xmlAttrPtr cur_attr = nullptr;
  const xmlChar *search_name = reinterpret_cast<const xmlChar *>(name.data());
  for (cur_attr = node_->properties; cur_attr != nullptr; cur_attr = cur_attr->next) {
    if (cur_attr->type == XML_ATTRIBUTE_NODE && xmlStrEqual(search_name, cur_attr->name)) {
      break;
    }
  }
  return LibXML2ParserAttribute{cur_attr};
}

LibXML2ParserAttribute LibXML2ParserNode::get_first_attribute() const {
  return LibXML2ParserAttribute{node_->properties};
}

std::string LibXML2ParserNode::node_name() const {
  return std::string{reinterpret_cast<const char *>(node_->name)};
}

std::string LibXML2ParserNode::node_value() const {
  char *value = reinterpret_cast<char *>(xmlNodeGetContent(node_));
  std::string value_str = std::string{(value != nullptr) ? value : ""};
  xmlFree(value);
  return value_str;
}

bool LibXML2ParserNode::is_valid_node() const { return node_ != nullptr; }

LibXML2ParserDocument::~LibXML2ParserDocument() {
  xmlFreeDoc(doc_);
  xmlCleanupParser();
}

bool LibXML2ParserDocument::parse_document(const std::string &filename) {
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
  filecontents_.assign(filecontent_str.cbegin(), filecontent_str.cend());
  filename_ = path.string();
  return parse_document(filecontents_);
}

bool LibXML2ParserDocument::parse_document(std::vector<char> &filecontent) {
  LIBXML_TEST_VERSION;
  //doc_ = xmlReadFile(filename_.c_str(), NULL, LibXML2ParserDocument::PARSING_OPTIONS);
  // Filecontents can not be null terminated
  if(filecontent.back() == '\0') {
    filecontent.pop_back();
  }
  doc_ = xmlReadMemory(filecontent.data(), filecontent.size(), filename_.c_str(), NULL, LibXML2ParserDocument::PARSING_OPTIONS);
  if (doc_ != nullptr) {
    int ret = xmlXIncludeProcess(doc_);
    return ret == 0;
  } else {
    return false;
  }
}

LibXML2ParserNode LibXML2ParserDocument::find_first_node(const std::string &name) const {
  xmlNodePtr cur_node = nullptr;
  const xmlChar *search_name = reinterpret_cast<const xmlChar *>(name.data());
  for (cur_node = doc_->children; cur_node != nullptr; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE && xmlStrEqual(search_name, cur_node->name)) {
      break;
    }
  }
  return LibXML2ParserNode{cur_node};
}

LibXML2ParserNode LibXML2ParserDocument::find_first_node() const {
  xmlNodePtr cur_node = nullptr;
  for (cur_node = doc_->children; cur_node != nullptr; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE) {
      break;
    }
  }
  return LibXML2ParserNode{cur_node};
}

}  // namespace scrimmage
