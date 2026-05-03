#ifndef C2OVERLAY_PLUGINS_AUTONOMY_REMOTEPREDATOR_REMOTEPREDATOR_H_
#define C2OVERLAY_PLUGINS_AUTONOMY_REMOTEPREDATOR_REMOTEPREDATOR_H_

#include <map>
#include <string>

#include "scrimmage/autonomy/Autonomy.h"

namespace c2overlay {
namespace autonomy {

class RemotePredator : public scrimmage::Autonomy {
 public:
  void init(std::map<std::string, std::string>& params) override;
  bool step_autonomy(double t, double dt) override;

 protected:
  // --- copied verbatim from upstream Predator.h ---
  int follow_id_;
  int prey_team_id_;
  double max_speed_;
  double capture_range_;
  bool allow_prey_switching_;
  scrimmage::PublisherPtr capture_ent_pub_;

  int speed_idx_ = 0;
  int turn_rate_idx_ = 0;
  int pitch_rate_idx_ = 0;

  int desired_heading_idx_ = 0;
  int desired_speed_idx_ = 0;

  // --- new for RemotePredator ---
  // -1 = no operator assignment, > 0 = locked target id.
  int assigned_target_id_ = -1;
};

}  // namespace autonomy
}  // namespace c2overlay

#endif  // C2OVERLAY_PLUGINS_AUTONOMY_REMOTEPREDATOR_REMOTEPREDATOR_H_
