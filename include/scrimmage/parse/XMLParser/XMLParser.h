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
#include <iterator>
#include <ostream>
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
      /*
       * Iterates over only the siblings of a node
       */
      class ChildIterator {
        public:
          using iterator_category = std::forward_iterator_tag;
          using difference_type   = std::ptrdiff_t;
          using value_type        = T;
          using pointer           = T*;
          using reference         = T&;

          ChildIterator(T node): node_{node} {}
          ChildIterator(const ChildIterator& other): node_{other.node_} {}
          ~ChildIterator() {}

          reference operator*() { return node_; }
          pointer operator->() { return &node_; }

          ChildIterator& operator++() {
            node_ = node_.next_sibling();
            if(!node_.is_valid()) {
              node_ = T{nullptr};
            }
            return *this;
          }

          ChildIterator& operator++(int) {
            ChildIterator& old{*this};
            ++(*this);
            return old;
          }

          friend bool operator==(const ChildIterator& lhs, const ChildIterator& rhs) {
            return (lhs.node_ == rhs.node_) || (!lhs.node_.is_valid() && !rhs.node_.is_valid()); 
          }

          friend bool operator!=(const ChildIterator& lhs, const ChildIterator& rhs) {
            return (lhs.node_ != rhs.node_) && (lhs.node_.is_valid() || rhs.node_.is_valid()); 
          }

        private: 
          T node_;
      };

      /*
       * Iterates over every Node that has this node as a parent. 
       * Will perform an in-order walk of the tree with this parents root
       */
      class RecursiveIterator  {
        public:
          using iterator_category = std::forward_iterator_tag;
          using difference_type   = std::ptrdiff_t;
          using value_type        = T;
          using pointer           = T*;
          using reference         = T&;

          RecursiveIterator(T node):
            node_{node}
          {}

          RecursiveIterator(const RecursiveIterator& other): 
            node_{other.node_}, 
            ancestors_{other.ancestors_}
          {}
          ~RecursiveIterator() {};

          reference operator*() { return node_; }
          pointer operator->() { return &node_; }

          RecursiveIterator& operator++() {
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
              while(!ancestors_.empty() && (!node_.is_valid())) {
                T parent = ancestors_.back();
                ancestors_.pop_back();
                node_ = parent.get_next_sibling();   
              }
            }
            if (!node_.is_valid()) { node_ = T{nullptr}; }
            return *this;
          }

          RecursiveIterator& operator++(int) {
            RecursiveIterator& old{*this};
            ++(*this);
            return old;
          }

          friend bool operator==(const RecursiveIterator& lhs, const RecursiveIterator& rhs) {
            return (lhs.node_ == rhs.node_) || (!lhs.node_.is_valid() && !rhs.node_.is_valid()); 
          }

          friend bool operator!=(const RecursiveIterator& lhs, const RecursiveIterator& rhs) {
            return !(lhs == rhs);
          }


        private: 
          bool is_valid() {
            return node_ != nullptr && node_->is_valid();
          }

          T node_;
          std::vector<T> ancestors_;
      };

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

      ChildIterator begin() {
        return ChildIterator{first_node()};
      }

      ChildIterator end() {
        return ChildIterator{nullptr};
      }

      RecursiveIterator recur_begin() {
        return RecursiveIterator{first_node()};
      }

      RecursiveIterator recur_end() {
        return RecursiveIterator{nullptr};
      }

      bool is_valid() const {
        return static_cast<const T*>(this)->is_valid_node();
      }

      friend bool operator==(const XMLParserNode& lhs, const XMLParserNode& rhs) {
        const T* lhs_ptr = static_cast<const T*>(&lhs);
        const T* rhs_ptr = static_cast<const T*>(&rhs);

        return (*lhs_ptr == *rhs_ptr) || (!lhs.is_valid() && !rhs.is_valid());
      }

      friend bool operator!=(const XMLParserNode& lhs, const XMLParserNode& rhs) {
        return !(lhs == rhs);
      }
    };

  template<class T>
    class XMLParserDocument {
      public:
        using XMLNode = typename XMLParserTraits<T>::child;
        using NodeChildIterator = typename XMLNode::ChildIterator;
        using NodeRecursiveIterator = typename XMLNode::RecursiveIterator;

        bool parse(const std::filesystem::path& path) {
          return static_cast<T*>(this)->parse_document(path);   
        }

        bool parse(std::vector<char>& filecontent) {
          return static_cast<T*>(this)->parse_document(filecontent);   
        }

        XMLNode first_node(const std::string& name) {
          return static_cast<T*>(this)->find_first_node(name);
        }

        XMLNode first_node() {
          return static_cast<T*>(this)->find_first_node();
        }

        NodeChildIterator begin() {
          return NodeChildIterator{first_node()};
        }

        NodeChildIterator end() {
          return NodeChildIterator{nullptr};
        }

        NodeRecursiveIterator recur_begin() {
          return NodeRecursiveIterator{first_node()};
        }

        NodeRecursiveIterator recur_end() {
          return NodeRecursiveIterator{nullptr};
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

  template<class T>
    std::ostream& operator<<(std::ostream& out, XMLParserDocument<T> xml_doc);
} // namespace scrimmage

#endif //INCLUDE_SCRIMMAGE_PARSE_XMLPARSER_XMLPARSER_H 
