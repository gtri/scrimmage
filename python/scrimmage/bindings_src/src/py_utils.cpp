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


#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/State.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/log/Log.h>
#include <py_utils.h>

#include <string>

namespace py = pybind11;
namespace sc = scrimmage;

class MyException : public std::exception {
public:
    explicit MyException(const char * m) : message{m} {}
    virtual const char * what() const noexcept override {return message.c_str();}
private:
    std::string message = "";
};

pybind11::object frames2pandas(std::string &fname) {

    // TODO: not sure if this is working after log refactor
    scrimmage::Log log;
    if (!log.parse(fname, scrimmage::Log::FRAMES)) {
        throw MyException((fname + "does not exist").c_str());
    }

    pybind11::list data;
    for (scrimmage::Frame &frame : log.scrimmage_frames()) {
        for (auto &kv : *(frame.contacts_)) {
            unsigned int id = kv.first;
            scrimmage::Contact &c = kv.second;

            Eigen::Vector3d &p = c.state()->pos();
            Eigen::Vector3d &v = c.state()->vel();
            scrimmage::Quaternion &q = c.state()->quat();

            pybind11::list row;
            row.append(pybind11::float_(frame.time_));
            row.append(pybind11::int_(id));
            row.append(pybind11::int_(c.id().team_id()));
            row.append(pybind11::int_(c.id().sub_swarm_id()));
            int type = static_cast<int>(c.type());
            row.append(pybind11::int_(type));
            row.append(pybind11::float_(p(0)));
            row.append(pybind11::float_(p(1)));
            row.append(pybind11::float_(p(2)));
            row.append(pybind11::float_(v(0)));
            row.append(pybind11::float_(v(1)));
            row.append(pybind11::float_(v(2)));
            row.append(pybind11::float_(q.yaw()));
            row.append(pybind11::float_(q.pitch()));
            row.append(pybind11::float_(q.roll()));

            data.append(row);
        }
    }

    pybind11::list cols;
    cols.append(pybind11::str("time"));
    cols.append(pybind11::str("id"));
    cols.append(pybind11::str("team_id"));
    cols.append(pybind11::str("swarm_id"));
    cols.append(pybind11::str("veh_type"));
    cols.append(pybind11::str("x"));
    cols.append(pybind11::str("y"));
    cols.append(pybind11::str("z"));
    cols.append(pybind11::str("vx"));
    cols.append(pybind11::str("vy"));
    cols.append(pybind11::str("vz"));
    cols.append(pybind11::str("yaw"));
    cols.append(pybind11::str("pitch"));
    cols.append(pybind11::str("roll"));

    pybind11::object df_class_obj = 
        pybind11::module::import("pandas").attr("DataFrame");
    pybind11::object df = df_class_obj(data, pybind11::none(), cols);
    return df;
}
