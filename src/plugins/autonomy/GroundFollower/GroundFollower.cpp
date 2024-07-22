/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2024 by the Georgia Tech Research Institute (GTRI)
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
 * @author Wesley Ford <wesley.ford@gatech.edu>
 * @date 31 Jan 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/CSV.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/autonomy/GroundFollower/GroundFollower.h>
#include <scrimmage/plugins/interaction/Boundary/Boundary.h>
#include <scrimmage/plugins/interaction/Boundary/BoundaryBase.h>
#include <scrimmage/plugins/interaction/Boundary/Cuboid.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/sensor/Sensor.h>

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace sc = scrimmage;
namespace sp = scrimmage_proto;
namespace sci = scrimmage::interaction;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::GroundFollower, GroundFollower_plugin)

namespace scrimmage {
namespace autonomy {
void GroundFollower::init(std::map<std::string, std::string> &params) {
  speed_ = scrimmage::get("speed", params, 0.0);
  target_height_ = sc::get<double>("target_height", params, 50);

  // Project goal in front...
  Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1e6;
  Eigen::Vector3d unit_vector = rel_pos.normalized();
  unit_vector = state_->quat().rotate(unit_vector);
  goal_ = state_->pos() + unit_vector * rel_pos.norm();
  goal_(2) = state_->pos()(2);

  // Set the desired_z to our initial position.
  // desired_z_ = state_->pos()(2);

  // Register the desired_z parameter with the parameter server
  auto param_cb = [&](const double &desired_z) {
    std::cout << "desired_z param changed at: " << time_->t() << ", with value: " << desired_z
              << endl;
  };
  register_param<double>("desired_z", goal_(2), param_cb);

  interpolate_terrain_ = get<bool>("interpolate_terrain_", params, false);

  auto bd_cb = [&](const auto &msg) { boundary_ = sci::Boundary::make_boundary(msg->data); };
  subscribe<sp::Shape>("GlobalNetwork", "Boundary", bd_cb);

  auto state_cb = [&](const auto &msg) {
    noisy_state_set_ = true;
    noisy_state_ = msg->data;
  };
  subscribe<StateWithCovariance>("LocalNetwork", "StateWithCovariance", state_cb);

  auto cnt_cb = [&](const scrimmage::MessagePtr<ContactMap> &msg) {
    noisy_contacts_ = msg->data;  // Save map of noisy contacts
  };
  subscribe<ContactMap>("LocalNetwork", "ContactsWithCovariances", cnt_cb);

  desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
  desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
  desired_heading_idx_ =
      vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

  std::string logging_filename = parent_->mp()->log_dir() + +"/" + "ground_follower_" +
                                 std::to_string(parent_->id().id()) + ".csv";
  if (!csv_.open_output(logging_filename)) {
    std::cout << "Could not create output file " << logging_filename << "\n";
  } else {
    csv_.set_column_headers("t, lon, lat,altitude, elevation");
  }
}

bool GroundFollower::step_autonomy(double t, double dt) {
  if (boundary_ != nullptr && enable_boundary_control_) {
    if (!boundary_->contains(noisy_state_.pos())) {
      // Project goal through center of boundary
      Eigen::Vector3d center = boundary_->center();
      center(2) = noisy_state_.pos()(2);  // maintain altitude
      Eigen::Vector3d diff = center - noisy_state_.pos();
      goal_ = noisy_state_.pos() + diff.normalized() * 1e6;
    }
  }
  Eigen::Vector3d diff = goal_ - noisy_state_.pos();
  Eigen::Vector3d v = speed_ * diff.normalized();

  if (terrain_map_ != nullptr) {
    double lat, lon, alt;
    double x_pos = state_->pos()(0);
    double y_pos = state_->pos()(1);
    double z_pos = state_->pos()(2);

    parent_->projection()->Reverse(x_pos, y_pos, z_pos, lat, lon, alt);

    double elevation = terrain_map_->QueryLongLat(lon, lat, interpolate_terrain_);
    // std::optional<double> elevation_opt = terrain_map_->RaycastLongLat(
    //     lon,
    //     lat,
    //     alt,
    //     0.0,
    //     -M_PI,
    //     1000.0);

    // if(elevation_opt) {
    //   double elevation = elevation_opt.value();
    //   desired_alt_ = elevation + target_height_;

    //  csv_.append(scrimmage::CSV::Pairs{
    //      {"t", t},
    //      {"lon", lon},
    //      {"lat", lat},
    //      {"altitude", alt},
    //      {"elevation", elevation}});
    //}

    desired_alt_ = elevation + target_height_;
  }

  ///////////////////////////////////////////////////////////////////////////
  // Convert desired velocity to desired speed, heading, and pitch controls
  ///////////////////////////////////////////////////////////////////////////
  double heading = Angles::angle_2pi(atan2(v(1), v(0)));
  vars_.output(desired_alt_idx_, desired_alt_);
  vars_.output(desired_speed_idx_, v.norm());
  vars_.output(desired_heading_idx_, heading);

  noisy_state_set_ = false;
  return true;
}

bool GroundFollower::posthumous(double t) {
  csv_.close_output();
  return true;
}
}  // namespace autonomy
}  // namespace scrimmage
