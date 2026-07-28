#ifndef RAPIDXML_ITERATORS_HPP_INCLUDED
#define RAPIDXML_ITERATORS_HPP_INCLUDED

// Copyright (C) 2006, 2009 Marcin Kalicinski
// Version 1.13
// Revision $DateTime: 2009/05/15 23:02:39 $
//! \file rapidxml_iterators.hpp This file contains rapidxml iterators

#include "rapidxml.hpp"

namespace rapidxml
{
    const unsigned int iterate_check_name = 1 << 0;
    const unsigned int iterate_case_sensitive = 1 << 1;

    //! Iterator of child nodes of xml_node
    template <class Ch = char>
    class node_iterator
    {
    public:
        typedef xml_node<Ch> *value_type;
        typedef const value_type& reference;
        typedef xml_node<Ch> *pointer;
        typedef std::ptrdiff_t difference_type;
        typedef std::bidirectional_iterator_tag iterator_category;

        node_iterator()
            : m_cur(0)
            , m_prev(0)
            , m_flags(0)
        {
        }

        node_iterator(xml_node<Ch>* node)
            : m_cur(node)
            , m_prev(0)
            , m_flags(0)
        {
        }

        node_iterator(xml_node<Ch>* node, xml_node<Ch>* prev,
                      unsigned char flags)
            : m_cur(node)
            , m_prev(prev)
            , m_flags(flags)
        {
        }

        reference operator*() const
        {
            assert(m_cur);
            return m_cur;
        }

        pointer operator->() const
        {
            assert(m_cur);
            return m_cur;
        }

        node_iterator& operator++()
        {
            increment();
            return *this;
        }

        node_iterator operator++(int)
        {
            node_iterator tmp = *this;
            increment();
            return tmp;
        }

        node_iterator& operator--()
        {
            decrement();
            return *this;
        }

        node_iterator operator--(int)
        {
            node_iterator tmp = *this;
            decrement();
            return tmp;
        }

        bool operator==(const node_iterator<Ch> &rhs) const
        {
            return m_cur == rhs.m_cur;
        }

        bool operator!=(const node_iterator<Ch> &rhs) const
        {
            return m_cur != rhs.m_cur;
        }

    private:
        void increment()
        {
            assert(m_cur && "Attempted to increment end iterator");
            m_prev = m_cur;

            if (m_flags & iterate_check_name)
                m_cur = m_cur->next_sibling(
                    m_cur->name(), m_cur->name_size(),
                    !!(m_flags & iterate_case_sensitive));
            else
                m_cur = m_cur->next_sibling();
        }

        void decrement()
        {
            assert(m_prev && "Attempted to decrement begin iterator");
            m_cur = m_prev;

            if (m_flags & iterate_check_name)
                m_prev = m_prev->previous_sibling(
                    m_prev->name(), m_prev->name_size(),
                    !!(m_flags & iterate_case_sensitive));
            else
                m_prev = m_prev->previous_sibling();
        }

        xml_node<Ch> *m_cur;
        xml_node<Ch> *m_prev;
        unsigned char m_flags;
    };

    //! Iterator of child attributes of xml_node
    template <class Ch = char>
    class attribute_iterator
    {
    public:
        typedef xml_attribute<Ch> *value_type;
        typedef const value_type& reference;
        typedef xml_attribute<Ch> *pointer;
        typedef std::ptrdiff_t difference_type;
        typedef std::bidirectional_iterator_tag iterator_category;

        attribute_iterator()
            : m_cur(0)
            , m_prev(0)
            , m_flags(0)
        {
        }

        attribute_iterator(xml_attribute<Ch>* attr)
            : m_cur(attr)
            , m_prev(0)
            , m_flags(0)
        {
        }

        attribute_iterator(xml_attribute<Ch>* attr, xml_attribute<Ch>* prev,
                           unsigned char flags)
            : m_cur(attr)
            , m_prev(prev)
            , m_flags(flags)
        {
        }

        reference operator*() const
        {
            assert(m_cur);
            return m_cur;
        }

        pointer operator->() const
        {
            assert(m_cur);
            return m_cur;
        }

        attribute_iterator& operator++()
        {
            increment();
            return *this;
        }

        attribute_iterator operator++(int)
        {
            attribute_iterator tmp = *this;
            increment();
            return tmp;
        }

        attribute_iterator& operator--()
        {
            decrement();
            return *this;
        }

        attribute_iterator operator--(int)
        {
            attribute_iterator tmp = *this;
            decrement();
            return tmp;
        }

        bool operator==(const attribute_iterator<Ch> &rhs) const
        {
            return m_cur == rhs.m_cur;
        }

        bool operator!=(const attribute_iterator<Ch> &rhs) const
        {
            return m_cur != rhs.m_cur;
        }

    private:
        void increment()
        {
            assert(m_cur && "Attempted to increment end iterator");
            m_prev = m_cur;

            if (m_flags & iterate_check_name)
                m_cur = m_cur->next_attribute(
                    m_cur->name(), m_cur->name_size(),
                    !!(m_flags & iterate_case_sensitive));
            else
                m_cur = m_cur->next_attribute();
        }

        void decrement()
        {
            assert(m_prev && "Attempted to decrement begin iterator");
            m_cur = m_prev;

            if (m_flags & iterate_check_name)
                m_prev = m_prev->previous_attribute(
                    m_prev->name(), m_prev->name_size(),
                    !!(m_flags & iterate_case_sensitive));
            else
                m_prev = m_prev->previous_attribute();
        }

        xml_attribute<Ch>* m_cur;
        xml_attribute<Ch>* m_prev;
        unsigned char m_flags;
    };

    // Range-based for loop support
    template <typename Iterator>
    class iterator_range
    {
    public:
        typedef Iterator const_iterator;
        typedef Iterator iterator;

        iterator_range(Iterator first, Iterator last)
            : m_first(first)
            , m_last(last)
        {
        }

        Iterator begin() const { return m_first; }
        Iterator end() const { return m_last; }

    private:
        Iterator m_first;
        Iterator m_last;
    };

    template <class Ch>
    iterator_range<node_iterator<Ch>> nodes(const xml_node<Ch>* node,
                                            const Ch* name = 0,
                                            std::size_t name_size = 0,
                                            bool case_sensitive = true)
    {
        unsigned char flags = 0;
        if (name)
            flags |= iterate_check_name;
        if (case_sensitive)
            flags |= iterate_case_sensitive;

        xml_node<Ch>* first =
            node->first_node(name, name_size, case_sensitive);
        xml_node<Ch>* last = first ?
            node->last_node(name, name_size, case_sensitive) : nullptr;

        node_iterator<Ch> begin(first, 0, flags);
        node_iterator<Ch> end(0, last, flags);
        return iterator_range<node_iterator<Ch>>(begin, end);
    }

    template <class Ch>
    iterator_range<attribute_iterator<Ch>> attributes(const xml_node<Ch>* node,
                                                      const Ch *name = 0,
                                                      std::size_t name_size = 0,
                                                      bool case_sensitive = true)
    {
        unsigned char flags = 0;
        if (name)
            flags |= iterate_check_name;
        if (case_sensitive)
            flags |= iterate_case_sensitive;

        xml_attribute<Ch>* first =
            node->first_attribute(name, name_size, case_sensitive);
        xml_attribute<Ch>* last =
            node->last_attribute(name, name_size, case_sensitive);

        attribute_iterator<Ch> begin(first, 0, flags);
        attribute_iterator<Ch> end(0, last, flags);
        return iterator_range<attribute_iterator<Ch>>(begin, end);
    }
}

#endif
