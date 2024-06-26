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
 */

#ifndef INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_RAPIDXMLPARSER_H 
#define INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_RAPIDXMLPARSER_H 

#include <scrimmage/parse/XMLParser/XMLParser.h>
#include <rapidxml/rapidxml.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace scrimmage {

  class RapidXMLParserAttribute : public XMLParserAttribute<RapidXMLParserAttribute> {
    public:
      RapidXMLParserAttribute(rapidxml::xml_attribute<>* attribute);

      RapidXMLParserAttribute next_attribute(const std::string& name);
      RapidXMLParserAttribute next_attribute();

      RapidXMLParserAttribute prev_attribute(const std::string& name);
      RapidXMLParserAttribute prev_attribute();

      std::string attribute_name();
      std::string attribute_value();

      bool is_valid_attribute();

    protected:
      rapidxml::xml_attribute<>* attribute_;
  };

  template<>
    struct XMLParserTraits<class RapidXMLParserNode> {
      using child = RapidXMLParserAttribute;
    };

  class RapidXMLParserNode : public XMLParserNode<RapidXMLParserNode> {
    public:

      RapidXMLParserNode(rapidxml::xml_node<>* node); 

      RapidXMLParserNode get_first_node(const std::string& name);
      RapidXMLParserNode get_first_node();
      RapidXMLParserNode get_next_sibling(const std::string& name);
      RapidXMLParserNode get_next_sibling();
      RapidXMLParserNode get_prev_sibling(const std::string& name);
      RapidXMLParserNode get_prev_sibling();

      RapidXMLParserAttribute get_first_attribute(const std::string& name);
      RapidXMLParserAttribute get_first_attribute();

      std::string node_name() const;
      std::string node_value() const;

      bool is_valid_node() const;

      friend bool operator==(const RapidXMLParserNode& lhs, const RapidXMLParserNode& rhs) {
        return lhs.node_ == rhs.node_;
      }

    protected:
      rapidxml::xml_node<>* node_;
  };

  template<>
    struct XMLParserTraits<class RapidXMLParserDocument> {
      using child = RapidXMLParserNode;
    };

  class RapidXMLParserDocument : public XMLParserDocument<RapidXMLParserDocument> {
    public:
      bool parse_document(std::filesystem::path path);
      bool parse_document(std::vector<char>& filecontent);
      RapidXMLParserNode find_first_node(const std::string& name);
      RapidXMLParserNode find_first_node();

    protected:
      std::vector<char> filecontent_;
      rapidxml::xml_document<> doc_;
  };

  using RapidXMLParser = RapidXMLParserDocument;
}

#endif //INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_RAPIDXMLPARSER_H 
