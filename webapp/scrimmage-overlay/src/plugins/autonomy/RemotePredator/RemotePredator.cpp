#include "c2overlay/plugins/autonomy/RemotePredator/RemotePredator.h"

#include <iostream>
#include <limits>

#include "scrimmage/entity/Entity.h"
#include "scrimmage/math/Angles.h"
#include "scrimmage/math/State.h"
#include "scrimmage/msgs/Capture.pb.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"
#include "scrimmage/pubsub/Message.h"
#include "scrimmage/pubsub/Publisher.h"

#include "Commands.pb.h"

namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Autonomy, c2overlay::autonomy::RemotePredator, RemotePredator_plugin)

namespace c2overlay {
namespace autonomy {

void RemotePredator::init(std::map<std::string, std::string>& params) {
  max_speed_ = scrimmage::get<double>("max_speed", params, 21);
  capture_range_ = scrimmage::get<double>("capture_range", params, 5);
  prey_team_id_ = scrimmage::get<int>("prey_team_id", params, 1);

  allow_prey_switching_ = scrimmage::get<bool>("allow_prey_switching", params, false);

  capture_ent_pub_ = advertise("GlobalNetwork", "CaptureEntity");

  follow_id_ = -1;

  speed_idx_ = vars_.declare(scrimmage::VariableIO::Type::speed, scrimmage::VariableIO::Direction::Out);
  turn_rate_idx_ = vars_.declare(scrimmage::VariableIO::Type::turn_rate, scrimmage::VariableIO::Direction::Out);
  pitch_rate_idx_ = vars_.declare(scrimmage::VariableIO::Type::pitch_rate, scrimmage::VariableIO::Direction::Out);

  desired_heading_idx_ =
      vars_.declare(scrimmage::VariableIO::Type::desired_heading, scrimmage::VariableIO::Direction::Out);
  desired_speed_idx_ = vars_.declare(scrimmage::VariableIO::Type::desired_speed, scrimmage::VariableIO::Direction::Out);

  // --- NEW: subscribe to operator commands. ---
  auto cb = [this](scrimmage::MessagePtr<c2overlay_msgs::TargetAssignment> msg) {
    std::cerr << "[RemotePredator id=" << parent_->id().id()
              << "] received TargetAssignment{predator_id=" << msg->data.predator_id()
              << ", target_id=" << msg->data.target_id() << "}\n";
    if (msg->data.predator_id() != parent_->id().id()) {
      std::cerr << "[RemotePredator id=" << parent_->id().id() << "] ignored (not for me)\n";
      return;  // not for me
    }
    if (msg->data.target_id() == 0) {
      std::cerr << "[RemotePredator id=" << parent_->id().id() << "] cleared assignment\n";
      assigned_target_id_ = -1;  // clear
    } else {
      assigned_target_id_ = msg->data.target_id();
      std::cerr << "[RemotePredator id=" << parent_->id().id()
                << "] locked target_id=" << assigned_target_id_ << "\n";
    }
  };
  subscribe<c2overlay_msgs::TargetAssignment>("GlobalNetwork", "Commands/TargetAssignment", cb);
  std::cerr << "[RemotePredator id=" << parent_->id().id()
            << "] subscribed to GlobalNetwork/Commands/TargetAssignment\n";
}

bool RemotePredator::step_autonomy(double t, double dt) {
  // --- NEW: hard-lock assigned target if valid; else fall through to upstream auto-pick. ---
  bool assignment_active = false;
  if (assigned_target_id_ > 0) {
    if (contacts_->count(assigned_target_id_) > 0) {
      follow_id_ = assigned_target_id_;
      assignment_active = true;
    } else {
      // Assigned target is gone — clear and let auto-pick resume.
      assigned_target_id_ = -1;
    }
  }

  // --- AUTO-PICK BLOCK (upstream behavior) ---
  // Only runs when no operator assignment is active, OR if follow_id_ went stale.
  if (!assignment_active) {
    if (contacts_->count(follow_id_) == 0) {
      follow_id_ = -1;
    }

    if (follow_id_ < 0 || allow_prey_switching_) {
      double min_dist = std::numeric_limits<double>::infinity();
      for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
        if (it->second.id().team_id() != prey_team_id_)
          continue;
        double dist = (it->second.state()->pos() - state_->pos()).norm();
        if (dist < min_dist) {
          min_dist = dist;
          follow_id_ = it->first;
        }
      }
    }
  }

  // --- CAPTURE PUBLISHING (upstream behavior, unchanged) ---
  for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
    if (it->second.id().team_id() == parent_->id().team_id())
      continue;
    double dist = (it->second.state()->pos() - state_->pos()).norm();
    if (dist < capture_range_) {
      auto msg = std::make_shared<scrimmage::Message<sm::CaptureEntity>>();
      msg->data.set_source_id(parent_->id().id());
      msg->data.set_target_id(it->second.id().id());
      capture_ent_pub_->publish(msg);
    }
  }

  // --- CHASE LOGIC (upstream behavior, unchanged) ---
  if (contacts_->count(follow_id_) > 0) {
    scrimmage::StatePtr ent_state = contacts_->at(follow_id_).state();
    Eigen::Vector3d v = (ent_state->pos() - state_->pos()).normalized() * max_speed_;
    double desired_heading = std::atan2(v(1), v(0));
    double desired_pitch = std::atan2(v(2), v.head<2>().norm());

    vars_.output(speed_idx_, max_speed_);
    vars_.output(turn_rate_idx_, scrimmage::Angles::angle_pi(desired_heading - state_->quat().yaw()));
    vars_.output(pitch_rate_idx_, scrimmage::Angles::angle_pi(desired_pitch + state_->quat().pitch()));

    vars_.output(desired_heading_idx_, desired_heading);
    vars_.output(desired_speed_idx_, max_speed_);
  }

  return true;
}

}  // namespace autonomy
}  // namespace c2overlay
