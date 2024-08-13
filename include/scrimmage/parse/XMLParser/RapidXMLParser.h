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

#ifndef INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_RAPIDXMLPARSER_H_
#define INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_RAPIDXMLPARSER_H_

#include <scrimmage/parse/XMLParser/XMLParser.h>

#include <cassert>
#include <string>
#include <vector>

#include <rapidxml/rapidxml.hpp>

namespace scrimmage {

class RapidXMLParserAttribute : public XMLParserAttribute<RapidXMLParserAttribute> {
 public:
    explicit RapidXMLParserAttribute(rapidxml::xml_attribute<> *attribute);

    RapidXMLParserAttribute next_attribute(const std::string &name) const;
    RapidXMLParserAttribute next_attribute() const;

    RapidXMLParserAttribute prev_attribute(const std::string &name) const;
    RapidXMLParserAttribute prev_attribute() const;

    std::string attribute_name() const;
    std::string attribute_value() const;

    bool is_valid_attribute() const;

    friend bool inline operator==(const RapidXMLParserAttribute &lhs,
                                  const RapidXMLParserAttribute &rhs) {
        return lhs.attribute_ == rhs.attribute_;
    }

    friend bool inline operator!=(const RapidXMLParserAttribute &lhs,
                                  const RapidXMLParserAttribute &rhs) {
        return !(lhs == rhs);
    }

 protected:
    rapidxml::xml_attribute<> *attribute_;
};

template <>
struct XMLParserTraits<class RapidXMLParserNode> {
    using child = RapidXMLParserAttribute;
};

class RapidXMLParserNode : public XMLParserNode<RapidXMLParserNode> {
 public:
    explicit RapidXMLParserNode(rapidxml::xml_node<> *node);

    RapidXMLParserNode get_first_node(const std::string &name) const;
    RapidXMLParserNode get_first_node() const;

    RapidXMLParserNode get_next_sibling(const std::string &name) const;
    RapidXMLParserNode get_next_sibling() const;

    RapidXMLParserNode get_prev_sibling(const std::string &name) const;
    RapidXMLParserNode get_prev_sibling() const;

    RapidXMLParserAttribute get_first_attribute(const std::string &name) const;
    RapidXMLParserAttribute get_first_attribute() const;

    std::string node_name() const;
    std::string node_value() const;

    bool is_valid_node() const;

    friend bool inline operator==(const RapidXMLParserNode &lhs, const RapidXMLParserNode &rhs) {
        return lhs.node_ == rhs.node_;
    }

    friend bool inline operator!=(const RapidXMLParserNode &lhs, const RapidXMLParserNode &rhs) {
        return !(lhs == rhs);
    }

 protected:
    rapidxml::xml_node<> *node_;
};

template <>
struct XMLParserTraits<class RapidXMLParserDocument> {
    using child = RapidXMLParserNode;
};

class RapidXMLParserDocument : public XMLParserDocument<RapidXMLParserDocument> {
 public:
    bool parse_document(const std::string &filename);
    bool parse_document(const std::vector<char> &filecontent);

    RapidXMLParserNode find_first_node(const std::string &name) const;
    RapidXMLParserNode find_first_node() const;

 protected:
    std::vector<char> filecontent_;
    rapidxml::xml_document<> doc_;
};

using RapidXMLParser = RapidXMLParserDocument;
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_RAPIDXMLPARSER_H_
