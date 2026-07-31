/*! 
 * @file
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_RUNTIMEOVERRIDESPAWNER_RUNTIMEOVERRIDESPAWNER_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_RUNTIMEOVERRIDESPAWNER_RUNTIMEOVERRIDESPAWNER_H_

#include <Eigen/Dense>

#include <map>
#include <string>

#include "scrimmage/autonomy/Autonomy.h"

namespace scrimmage {
namespace autonomy {

class RuntimeOverrideSpawner : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string>& params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    void publish_spawn(int index, int row, int col, const Eigen::Vector3d& anchor);

    PublisherPtr pub_gen_ents_;

    // Entity template and naming
    std::string entity_tag_ = "runtime_override_agent";
    std::string name_prefix_ = "runtime_override_agent";

    // Layout configuration
    int spawn_count_ = 120;
    int columns_ = 12;
    double x_spacing_ = 20.0;
    double y_spacing_ = 20.0;
    double z_spacing_ = 0.0;
    double x_offset_ = 0.0;
    double y_offset_ = 0.0;
    double z_offset_ = 0.0;
    double spawn_time_ = 0.1;
    bool spawned_ = false;

   int desired_altitude_idx_ = 0;
   int desired_heading_idx_ = 0;
   int desired_speed_idx_ = 0;
};

}  // namespace autonomy
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_RUNTIMEOVERRIDESPAWNER_RUNTIMEOVERRIDESPAWNER_H_