#ifndef RAPIDXML_UTILS_HPP_INCLUDED
#define RAPIDXML_UTILS_HPP_INCLUDED

// Copyright (C) 2006, 2009 Marcin Kalicinski
// Version 1.13
// Revision $DateTime: 2009/05/15 23:02:39 $
//! \file rapidxml_utils.hpp This file contains high-level rapidxml utilities that can be useful
//! in certain simple scenarios. They should probably not be used if maximizing performance is the main objective.

#include "rapidxml.hpp"
#include <stdexcept>

namespace rapidxml
{
    //! Counts children of node. Time complexity is O(n).
    //! \return Number of children of node
    template<class Ch>
    inline std::size_t count_children(const xml_node<Ch> *node,
                                      const Ch* name = 0,
                                      std::size_t name_size = 0)
    {
        if (name && name_size == 0)
            name_size = internal::measure(name);

        xml_node<Ch> *child = node->first_node(name, name_size);
        std::size_t count = 0;
        while (child)
        {
            ++count;
            child = child->next_sibling(name, name_size);
        }
        return count;
    }

    //! Counts attributes of node. Time complexity is O(n).
    //! \return Number of attributes of node
    template<class Ch>
    inline std::size_t count_attributes(const xml_node<Ch> *node,
                                        const Ch* name = 0,
                                        std::size_t name_size = 0)
    {
        if (name && name_size == 0)
            name_size = internal::measure(name);

        xml_attribute<Ch> *attr = node->first_attribute(name, name_size);
        std::size_t count = 0;
        while (attr)
        {
            ++count;
            attr = attr->next_attribute(name, name_size);
        }
        return count;
    }
}

#endif
