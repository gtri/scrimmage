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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_JOYSTICKCONTROLLER_AXISSCALE_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_JOYSTICKCONTROLLER_AXISSCALE_H_

#include <scrimmage/common/Utilities.h>

#include <string>

namespace scrimmage {
namespace controller {

class AxisScale {
 public:
    AxisScale(int axis_index, double input_min, double input_max,
              double output_min, double output_max, double coeff,
              int vector_index) : axis_index_(axis_index),
        input_min_(input_min), input_max_(input_max),
        output_min_(output_min), output_max_(output_max), coeff_(coeff),
        vector_index_(vector_index) { }

    int axis_index() { return axis_index_; }
    int vector_index() { return vector_index_; }

    double scale(double input) {
        return coeff_ * scrimmage::scale<double>(input, input_min_, input_max_,
                                                 output_min_, output_max_);
    }

 protected:
    int axis_index_ = 0;
    double input_min_ = 0;
    double input_max_ = 0;
    double output_min_ = 0;
    double output_max_ = 0;
    double coeff_ = 1;
    int vector_index_ = 0;
};

} // namespace controller
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_JOYSTICKCONTROLLER_AXISSCALE_H_
