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

#ifndef INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H 
#define INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H 

#include <filesystem>
#include <string>
#include <vector>

namespace scrimmage {

template <typename child>
struct XMLParserTraits;

template<class T>
class XMLParserAttribute {
  public:
    T next(const std::string& name) {
      return static_cast<T*>(this)->next_attribute(name);
    }

    T next() {
      return static_cast<T*>(this)->next_attribute();
    }

    T prev(const std::string& name) {
      return static_cast<T*>(this)->prev_attribute(name);
    }

    T prev() {
      return static_cast<T*>(this)->prev_attribute();
    }

    std::string name() {
      return static_cast<T*>(this)->attribute_name();
    }

    std::string value() { 
      return static_cast<T*>(this)->attribute_value();
    }

    bool is_valid() {
      return static_cast<T*>(this)->is_valid_attribute();
    }
};

template<class T>
class XMLParserNode {
  using XMLAttribute = typename XMLParserTraits<T>::child;
  
  public:
    T first_node(const std::string& name) {
      return static_cast<T*>(this)->get_first_node(name);
    }

    T first_node() {
      return static_cast<T*>(this)->get_first_node();
    }

    T next_sibling(const std::string& name) {
      return static_cast<T*>(this)->get_next_sibling(name);
    }

    T next_sibling() {
      return static_cast<T*>(this)->get_next_sibling();
    }

    T prev_sibling(const std::string& name) {
      return static_cast<T*>(this)->get_prev_sibling(name);
    }

    T prev_sibling() {
      return static_cast<T*>(this)->get_prev_sibling();
    }

    XMLAttribute first_attribute(const std::string& name) {
      return static_cast<T*>(this)->get_first_attribute(name);
    }

    XMLAttribute first_attribute() {
      return static_cast<T*>(this)->get_first_attribute();
    }

    std::string name() {
      return static_cast<T*>(this)->node_name();
    }

    std::string value() {
      return static_cast<T*>(this)->node_value();
    }

    bool is_valid() {
      return static_cast<T*>(this)->is_valid_node();
    }
};

template<class T>
class XMLParserDocument {
  public:
    using XMLNode = typename XMLParserTraits<T>::child;
    
    bool parse(const std::filesystem::path& path) {
      return static_cast<T*>(this)->parse_document(path);   
    }

    bool parse(std::vector<char>& filecontent) {
      return static_cast<T*>(this)->parse_document(filecontent);   
    }

    XMLNode first_node(const std::string& name) {
      return static_cast<T*>(this)->find_first_node(name);
    }

    void set_filename(const std::string& filename) {
      filename_ = filename;
    }

    std::string filename() {
      return filename_;
    }

  protected:
    std::string filename_;
};

} // namespace scrimmage

#endif //INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H 
