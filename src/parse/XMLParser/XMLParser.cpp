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

//#include <scrimmage/parse/XMLParser/XMLParser.h>
//
//#include <algorithm>
//
//
//// Some helper functions to trim whitespace:
//namespace scrimmage {
//  namespace XMLUtilities {
//
//    template<typename S>
//    S left_trim(S&& s){
//      s.erase(s.begin(), 
//          std::find_if(s.cbegin(), s.cend(), [](unsigned char c) {
//            return !std::isspace(c);
//            })
//          );
//      return s;
//    }
//
//    template<typename S>
//    S right_trim(S&& s) {
//      auto start_it = std::find_if(s.crbegin(), s.crend(), [](unsigned char c){
//          return !std::isspace(c);
//          }).base() + 1;
//      s.erase(start_it, s.end());
//      return s;
//    }
//
//    template<typename S>
//    S trim(S&& s) {
//      left_trim(s);
//      right_trim(s);
//      return s;
//    }
//  }
//}
