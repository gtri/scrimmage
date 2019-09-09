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

#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/controller/JoystickController/Joystick.h>
#include <scrimmage/entity/EntityPlugin.h>

#include <iostream>
#include <algorithm>

using std::cout;
using std::endl;

namespace sc = scrimmage;

namespace scrimmage {
namespace controller {

Joystick::Joystick() {
}

Joystick::~Joystick() {
    free(axis_);
    free(button_);
}

void Joystick::init(std::map<std::string, std::string> &params,
                    VariableIO &vars, EntityPluginPtr plugin) {
    print_js_values_ = sc::get<bool>("print_raw_joystick_values", params, false);

    std::string dev = sc::get<std::string>("device", params, "/dev/input/js0");
    if ((joy_fd_ = open(dev.c_str(), O_RDONLY)) == -1) {
		cout << "couldn't open joystick: " << dev << endl;
	}

    char name_of_joystick[80];

    ioctl(joy_fd_, JSIOCGAXES, &num_of_axis_);
	ioctl(joy_fd_, JSIOCGBUTTONS, &num_of_buttons_);
	ioctl(joy_fd_, JSIOCGNAME(80), &name_of_joystick);

	axis_ = reinterpret_cast<int*>(calloc(num_of_axis_, sizeof(int)));
	button_ = reinterpret_cast<char*>(calloc(num_of_buttons_, sizeof(char)));

    prev_button_state_.resize(num_of_buttons_);

    if (print_js_values_) {
        cout << "Joystick detected:" << *name_of_joystick << endl;
        cout << "\t " << num_of_axis_ << " axis" << endl;
        cout << "\t " << num_of_buttons_ << " buttons" << endl;
    }

	fcntl(joy_fd_, F_SETFL, O_NONBLOCK); // use non-blocking mode

    std::string axis_map = sc::get<std::string>("axis_map", params, "");
    std::vector<std::vector<std::string>> vecs;
    if (!sc::get_vec_of_vecs(axis_map, vecs)) {
        cout << "Failed to parse axis map:" << axis_map << endl;
    } else {
        for (std::vector<std::string> vec : vecs) {
            if (vec.size() != 7) {
                cout << "Invalid joystick axis mapping: " << endl;
                for (std::string s : vec) {
                    cout << s << " ";
                }
                continue;
            }

            int axis = std::stod(vec[1]);
            if (axis >= num_of_axis_) {
                cout << "Warning: axis_map contains out-of-range axis index"
                     << endl;
            } else {
                AxisScale at(axis, std::stod(vec[2]),
                             std::stod(vec[3]), std::stod(vec[4]),
                             std::stod(vec[5]), std::stod(vec[6]),
                             vars.declare(vec[0], VariableIO::Direction::Out));
                axis_tfs_.push_back(at);
            }
        }
    }

    publish_button_state_ = sc::get<bool>("publish_button_state", params, false);
    std::string button_topic = sc::get<std::string>("button_topic", params, "joystick_buttons");
    std::string button_network_name = sc::get<std::string>("button_network_name", params, "LocalNetwork");
    if (publish_button_state_) {
        pub_buttons_ = plugin->advertise(button_network_name, button_topic);
    }
}

bool Joystick::step(double t, double dt, VariableIO &vars) {
    int bytes = read(joy_fd_, &js_, sizeof(struct js_event));
    if (bytes == -1) {
        // nop, avoid unused variable warning
    }

    // see what to do with the event
    bool button_changed = false;
    switch (js_.type & ~JS_EVENT_INIT) {
    case JS_EVENT_AXIS:
        axis_[js_.number] = js_.value;
        break;
    case JS_EVENT_BUTTON:
        button_[js_.number] = js_.value;
        button_changed = true;
        break;
    }

    if (print_js_values_) {
        for (int x = 0; x < num_of_axis_; x++) {
            printf("%d: %6d  ", x, axis_[x] );
        }

        for (int x = 0; x < num_of_buttons_; x++) {
            printf("B%d: %d  ", x, button_[x]);
        }
        printf("  \r");
        fflush(stdout);
    }

    for (AxisScale axis_tf : axis_tfs_) {
        vars.output(axis_tf.vector_index(),
                    axis_tf.scale(axis_[axis_tf.axis_index()]));
    }

    if (publish_button_state_) {
        // If the button state has changed, publish a new button message
        if (button_changed) {
            auto msg = std::make_shared<sc::Message<std::vector<bool>>>();
            msg->data.resize(num_of_buttons_);

            for (int i = 0; i < num_of_buttons_; i++) {
                msg->data[i] = button_[i];
            }

            auto it = std::mismatch(msg->data.begin(), msg->data.end(),
                              prev_button_state_.begin());
            if (it.first != msg->data.end()) {
                pub_buttons_->publish(msg);
                prev_button_state_ = msg->data;
            }
        }
    }

    return true;
}

} // namespace controller
} // namespace scrimmage
