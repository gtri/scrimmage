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

#ifndef INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_LIBXML2PARSER_H 
#define INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_LIBXML2PARSER_H 

#include <scrimmage/parse/XMLParser/XMLParser.h>

#include <libxml/xmlreader.h>
#include <libxml/tree.h>

#include <filesystem>
#include <string>
#include <vector>

namespace scrimmage {

  class LibXML2ParserAttribute : public XMLParserAttribute<LibXML2ParserAttribute> {
    public:
      LibXML2ParserAttribute(xmlAttrPtr attribute);

      LibXML2ParserAttribute next_attribute(const std::string& name) const;
      LibXML2ParserAttribute next_attribute() const;

      LibXML2ParserAttribute prev_attribute(const std::string& name) const;
      LibXML2ParserAttribute prev_attribute() const;

      std::string attribute_name() const;
      std::string attribute_value() const;

      bool is_valid_attribute() const;

      friend bool operator==(const LibXML2ParserAttribute& lhs, const LibXML2ParserAttribute& rhs) {
        return lhs.attribute_ == rhs.attribute_;  
      }

    protected:
      xmlAttrPtr attribute_;
  };

  template<>
    struct XMLParserTraits<class LibXML2ParserNode> {
      using child = LibXML2ParserAttribute;
    };

  class LibXML2ParserNode : public XMLParserNode<LibXML2ParserNode> {
    public:

      LibXML2ParserNode(xmlNodePtr node);
 
      LibXML2ParserNode get_first_node(const std::string& name) const ;
      LibXML2ParserNode get_first_node() const;

      LibXML2ParserNode get_next_sibling(const std::string& name) const;
      LibXML2ParserNode get_next_sibling() const;

      LibXML2ParserNode get_prev_sibling(const std::string& name) const;
      LibXML2ParserNode get_prev_sibling() const;

      LibXML2ParserAttribute get_first_attribute(const std::string& name) const;
      LibXML2ParserAttribute get_first_attribute() const;

      std::string node_name() const;
      std::string node_value() const;

      bool is_valid_node() const;

      friend bool operator==(const LibXML2ParserNode& lhs, const LibXML2ParserNode& rhs) {
        return lhs.node_ == rhs.node_;
      }

    protected:
      xmlNodePtr node_;
  };

  template<>
    struct XMLParserTraits<class LibXML2ParserDocument> {
      using child = LibXML2ParserNode;
    };

  class LibXML2ParserDocument : public XMLParserDocument<LibXML2ParserDocument> {
    public:
      ~LibXML2ParserDocument();
      bool parse_document(std::filesystem::path path);
      bool parse_document(std::vector<char>& filecontent);
      LibXML2ParserNode find_first_node(const std::string& name) const;
      LibXML2ParserNode find_first_node() const;

    protected:
      static constexpr int PARSING_OPTIONS = XML_PARSE_XINCLUDE | XML_PARSE_NOBLANKS; 

      xmlDocPtr doc_;
      std::vector<char> filecontents_;
  };

  using LibXML2Parser = LibXML2ParserDocument;
}

#endif //INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_LIBXML2PARSER_H 
