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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_JOYSTICKCONTROLLER_JOYSTICK_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_JOYSTICKCONTROLLER_JOYSTICK_H_

#ifndef __APPLE__

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <scrimmage/plugins/controller/JoystickController/AxisScale.h>
#include <scrimmage/common/VariableIO.h>
#include <scrimmage/pubsub/PubSub.h>
#include <scrimmage/pubsub/Publisher.h>

#include <string>
#include <list>
#include <vector>
#include <map>

namespace scrimmage {
namespace controller {

class Joystick {
 public:
    Joystick();
    ~Joystick();

    void init(std::map<std::string, std::string> &params, VariableIO &vars,
              PluginPtr plugin);
    bool step(double t, double dt, VariableIO &vars);

 protected:
    int joy_fd_ = -1;
    int *axis_ = NULL;
    int num_of_axis_ = 0;
    int num_of_buttons_ = 0;
	char *button_ = NULL;
	struct js_event js_;

    int min_value = -32767;
    int max_value = +32767;

    bool print_js_values_ = false;

    std::list<AxisScale> axis_tfs_;

    bool publish_button_state_ = false;

    scrimmage::PublisherPtr pub_buttons_;
    std::vector<bool> prev_button_state_;
};

} // namespace controller
} // namespace scrimmage

#endif  // __APPLE__

#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_JOYSTICKCONTROLLER_JOYSTICK_H_
