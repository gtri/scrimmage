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

#ifndef INCLUDE_SCRIMMAGE_AUTONOMY_AUTONOMY_H_
#define INCLUDE_SCRIMMAGE_AUTONOMY_AUTONOMY_H_

#include <scrimmage/fwd_decl.h>
#include <scrimmage/entity/EntityPlugin.h>

#include <list>
#include <map>
#include <memory>
#include <string>

namespace scrimmage {

class Autonomy : public EntityPlugin {
 public:
    Autonomy();

    std::string type() override;
    virtual bool step_autonomy(double t, double dt);
    virtual bool posthumous(double t);
    virtual void init();
    bool ready() override { return true; }
    virtual void init(std::map<std::string, std::string> &params);
    void close(double t) override;
    bool need_reset();

    // getters/setters
    StatePtr &desired_state();
    void set_desired_state(StatePtr desired_state);

    ContactMapPtr &get_contacts();
    ContactMap &get_contacts_raw();
    virtual void set_contacts(ContactMapPtr &contacts);
    virtual void set_contacts_from_plugin(AutonomyPtr &ptr);
    virtual void set_projection(std::shared_ptr<GeographicLib::LocalCartesian> &proj);

    scrimmage::RTreePtr &rtree();
    void set_rtree(scrimmage::RTreePtr &rtree);

    StatePtr &state();
    virtual void set_state(StatePtr &state);

    std::string &logging_msg();

    bool get_is_controlling();
    void set_is_controlling(bool is_controlling);

 protected:
    std::shared_ptr<GeographicLib::LocalCartesian> proj_;

    StatePtr state_;
    StatePtr desired_state_;
    ContactMapPtr contacts_;
    scrimmage::RTreePtr rtree_;

    bool need_reset_;
    std::string logging_msg_;

    bool is_controlling_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_AUTONOMY_AUTONOMY_H_
