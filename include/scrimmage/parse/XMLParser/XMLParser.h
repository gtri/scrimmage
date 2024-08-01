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

#ifndef INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H_
#define INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H_

#include <cassert>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <ostream>
#include <string>
#include <vector>

namespace scrimmage {
template <typename child>
struct XMLParserTraits;

template <class T>
class XMLParserAttribute {
 public:
    class Iterator {
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = T *;
        using reference = T &;

     public:
        explicit Iterator(T attr)
            : attribute_{attr} {}
        Iterator(const Iterator &other)
            : attribute_{other.attribute_} {}

        reference operator*() { return attribute_; }
        pointer operator->() { return &attribute_; }

        Iterator &operator++() {
            if (attribute_ != T{nullptr}) {
                attribute_ = attribute_.next();
                if (!attribute_.is_valid()) {
                    attribute_ = T{nullptr};
                }
            }
            return *this;
        }

        Iterator &operator++(int) {
            XMLParserAttribute &old = *this;
            ++(*this);
            return old;
        }

        friend bool operator==(const Iterator &rhs, const Iterator &lhs) {
            return (lhs.attribute_ == rhs.attribute_) ||
                   (!lhs.attribute_.is_valid() && !rhs.attribute_.is_valid());
        }

        friend bool operator!=(const Iterator &rhs, const Iterator &lhs) { return !(lhs == rhs); }

     private:
        T attribute_;
    };

    T next(const std::string &name) const {
        return static_cast<const T *>(this)->next_attribute(name);
    }

    T next() const { return static_cast<const T *>(this)->next_attribute(); }

    T prev(const std::string &name) const {
        return static_cast<const T *>(this)->prev_attribute(name);
    }

    T prev() const { return static_cast<const T *>(this)->prev_attribute(); }

    std::string name() const { return static_cast<const T *>(this)->attribute_name(); }

    std::string value() const { return static_cast<const T *>(this)->attribute_value(); }

    bool is_valid() const { return static_cast<const T *>(this)->is_valid_attribute(); }
};

template <class T>
class XMLParserNode {
 public:
    using XMLAttribute = typename XMLParserTraits<T>::child;
    /*
     * Iterates over only the siblings of a node
     */
    class ChildIterator {
     public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = T *;
        using reference = T &;

        explicit ChildIterator(T node)
            : node_{node} {}
        ChildIterator(const ChildIterator &other)
            : node_{other.node_} {}
        ~ChildIterator() {}

        reference operator*() { return node_; }
        pointer operator->() { return &node_; }

        ChildIterator &operator++() {
            if (node_ != T{nullptr}) {
                node_ = node_.next_sibling();
                if (!node_.is_valid()) {
                    node_ = T{nullptr};
                }
            }
            return *this;
        }

        ChildIterator &operator++(int) {
            ChildIterator &old{*this};
            ++(*this);
            return old;
        }

        friend bool operator==(const ChildIterator &lhs, const ChildIterator &rhs) {
            return (lhs.node_ == rhs.node_) || (!lhs.node_.is_valid() && !rhs.node_.is_valid());
        }

        friend bool operator!=(const ChildIterator &lhs, const ChildIterator &rhs) {
            return (lhs.node_ != rhs.node_) && (lhs.node_.is_valid() || rhs.node_.is_valid());
        }

     private:
        T node_;
    };

    /*
     * Iterates over every Node that has this node as a parent.
     * Will perform an in-order walk of the tree with this parents root
     */
    class RecursiveIterator {
     public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = T *;
        using reference = T &;

        explicit RecursiveIterator(T node)
            : node_{node} {}

        RecursiveIterator(const RecursiveIterator &other)
            : node_{other.node_},
              ancestors_{other.ancestors_} {}
        ~RecursiveIterator() {}

        reference operator*() { return node_; }
        pointer operator->() { return &node_; }

        RecursiveIterator &operator++() {
            if (node_ != T{nullptr}) {  // Avoid incrementing if this is the end node
                T child = node_.get_first_node();
                T sibling = node_.get_next_sibling();
                if (child.is_valid()) {
                    ancestors_.push_back(node_);
                    node_ = child;
                } else if (sibling.is_valid()) {
                    node_ = sibling;
                } else {
                    // No child or sibling.
                    node_ = T{nullptr};
                    while (!ancestors_.empty() && (!node_.is_valid())) {
                        T parent = ancestors_.back();
                        ancestors_.pop_back();
                        node_ = parent.get_next_sibling();
                    }
                }
                if (!node_.is_valid()) {
                    node_ = T{nullptr};
                }
            }
            return *this;
        }

        RecursiveIterator &operator++(int) {
            RecursiveIterator &old{*this};
            ++(*this);
            return old;
        }

        friend bool operator==(const RecursiveIterator &lhs, const RecursiveIterator &rhs) {
            return (lhs.node_ == rhs.node_) || (!lhs.node_.is_valid() && !rhs.node_.is_valid());
        }

        friend bool operator!=(const RecursiveIterator &lhs, const RecursiveIterator &rhs) {
            return !(lhs == rhs);
        }

     private:
        bool is_valid() { return node_ != nullptr && node_->is_valid(); }

        T node_;
        std::vector<T> ancestors_;
    };

    T first_node(const std::string &name) const {
        return static_cast<const T *>(this)->get_first_node(name);
    }

    T first_node() const { return static_cast<const T *>(this)->get_first_node(); }

    T next_sibling(const std::string &name) const {
        return static_cast<const T *>(this)->get_next_sibling(name);
    }

    T next_sibling() const { return static_cast<const T *>(this)->get_next_sibling(); }

    T prev_sibling(const std::string &name) const {
        return static_cast<const T *>(this)->get_prev_sibling(name);
    }

    T prev_sibling() const { return static_cast<const T *>(this)->get_prev_sibling(); }

    XMLAttribute first_attribute(const std::string &name) const {
        return static_cast<const T *>(this)->get_first_attribute(name);
    }

    XMLAttribute first_attribute() const {
        return static_cast<const T *>(this)->get_first_attribute();
    }

    std::string name() const { return static_cast<const T *>(this)->node_name(); }

    std::string value() const { return static_cast<const T *>(this)->node_value(); }

    ChildIterator child_begin() const { return ChildIterator{first_node()}; }

    ChildIterator child_end() const { return ChildIterator{T{nullptr}}; }

    RecursiveIterator recur_begin() const { return RecursiveIterator{first_node()}; }

    RecursiveIterator recur_end() const { return RecursiveIterator{T{nullptr}}; }

    using Attribute = typename XMLParserTraits<T>::child;
    using AttributeIterator = typename Attribute::Iterator;

    AttributeIterator attr_begin() const { return AttributeIterator{first_attribute()}; }

    AttributeIterator attr_end() const { return AttributeIterator{Attribute{nullptr}}; }

    bool is_valid() const { return static_cast<const T *>(this)->is_valid_node(); }

    friend bool operator==(const XMLParserNode &lhs, const XMLParserNode &rhs) {
        const T *lhs_ptr = static_cast<const T *>(&lhs);
        const T *rhs_ptr = static_cast<const T *>(&rhs);

        return (*lhs_ptr == *rhs_ptr) || (!lhs.is_valid() && !rhs.is_valid());
    }

    friend bool operator!=(const XMLParserNode &lhs, const XMLParserNode &rhs) {
        return !(lhs == rhs);
    }
};

template <class T>
class XMLParserDocument {
 public:
    using XMLNode = typename XMLParserTraits<T>::child;
    using NodeChildIterator = typename XMLNode::ChildIterator;
    using NodeRecursiveIterator = typename XMLNode::RecursiveIterator;

    bool parse(const std::filesystem::path &path) {
        return static_cast<T *>(this)->parse_document(path);
    }

    bool parse(std::vector<char> &filecontent) {
        return static_cast<T *>(this)->parse_document(filecontent);
    }

    XMLNode first_node(const std::string &name) const {
        return static_cast<const T *>(this)->find_first_node(name);
    }

    XMLNode first_node() const { return static_cast<const T *>(this)->find_first_node(); }

    NodeChildIterator begin() const { return NodeChildIterator{first_node()}; }

    NodeChildIterator end() const { return NodeChildIterator{XMLNode{nullptr}}; }

    NodeRecursiveIterator recur_begin() const { return NodeRecursiveIterator{first_node()}; }

    NodeRecursiveIterator recur_end() const { return NodeRecursiveIterator{XMLNode{nullptr}}; }

    void set_filename(const std::string &filename) { filename_ = filename; }

    const std::string& filename() const { return filename_; }

 protected:
    std::string filename_;
};

template <class T>
std::ostream &operator<<(std::ostream &out, const XMLParserAttribute<T> &xml_attr) {
    if (xml_attr.is_valid()) {
        std::string name = xml_attr.name();
        std::string value = xml_attr.value();
        out << name << "=\"" << value << "\"";
    }
    return out;
}

template <class T>
std::ostream &operator<<(std::ostream &out, const XMLParserNode<T> &xml_node) {
    using pos_type = typename std::ostream::pos_type;
    if (xml_node.is_valid()) {
        pos_type cur_pos;

        std::string name = xml_node.name();
        assert(!name.empty());
        std::ostringstream ss;

        ss << "<" << name << " ";
        for (auto attr_it = xml_node.attr_begin(); attr_it != xml_node.attr_end(); ++attr_it) {
            ss << *attr_it << " ";
        }
        cur_pos = ss.tellp();
        assert(cur_pos != pos_type(-1));
        ss.seekp(cur_pos - pos_type(1));
        ss << ">";
        auto child_it = xml_node.child_begin();
        auto child_end = xml_node.child_end();
        bool has_children = child_it != child_end;
        if (has_children) {
            for (; child_it != child_end; ++child_it) {
                ss << "\n" << *child_it;
            }
        }
        ss << "</" << name << ">";
        out << ss.str();
    }
    return out;
}

template <class T>
std::ostream &operator<<(std::ostream &out, const XMLParserDocument<T> &xml_doc) {
    out << xml_doc.first_node();
    return out;
}
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H_
