#include "scrimmage/plugins/autonomy/RuntimeOverrideSpawner/RuntimeOverrideSpawner.h"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

#include "scrimmage/log/Logger.h"
#include "scrimmage/math/State.h"
#include "scrimmage/msgs/Event.pb.h"
#include "scrimmage/parse/ParseUtils.h"
#include "scrimmage/plugin_manager/RegisterPlugin.h"
#include "scrimmage/proto/ProtoConversions.h"
#include "scrimmage/pubsub/Message.h"
#include "scrimmage/pubsub/Publisher.h"

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(
    scrimmage::Autonomy,
    scrimmage::autonomy::RuntimeOverrideSpawner,
    RuntimeOverrideSpawner_plugin)

namespace scrimmage {
namespace autonomy {

namespace {

std::string zero_pad_index(int index, int width) {
    std::ostringstream stream;
    stream << std::setw(width) << std::setfill('0') << index;
    return stream.str();
}

}  // namespace

void RuntimeOverrideSpawner::init(std::map<std::string, std::string>& params) {
    // This plugin demonstrates how to publish GenerateEntity messages with
    // runtime plugin parameter overrides. Copy and adapt this pattern for
    // custom spawning logic.
    pub_gen_ents_ = advertise("GlobalNetwork", "GenerateEntity");

    // Layout/spawning configuration - kept configurable via XML
    entity_tag_ = sc::get<std::string>("entity_tag", params, entity_tag_);
    name_prefix_ = sc::get<std::string>("name_prefix", params, name_prefix_);
    spawn_count_ = std::max(0, sc::get<int>("spawn_count", params, spawn_count_));
    columns_ = std::max(1, sc::get<int>("columns", params, columns_));
    x_spacing_ = sc::get<double>("x_spacing", params, x_spacing_);
    y_spacing_ = sc::get<double>("y_spacing", params, y_spacing_);
    z_spacing_ = sc::get<double>("z_spacing", params, z_spacing_);
    x_offset_ = sc::get<double>("x_offset", params, x_offset_);
    y_offset_ = sc::get<double>("y_offset", params, y_offset_);
    z_offset_ = sc::get<double>("z_offset", params, z_offset_);
    spawn_time_ = sc::get<double>("spawn_time", params, spawn_time_);

    desired_altitude_idx_ =
        vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
    desired_heading_idx_ =
        vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
    desired_speed_idx_ =
        vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
}

bool RuntimeOverrideSpawner::step_autonomy(double t, double dt) {
    static_cast<void>(dt);

    // Keep the host entity valid for UnicyclePID even though it is only acting
    // as a publisher for runtime-generated children.
    vars_.output(desired_altitude_idx_, state_->pos()(2));
    vars_.output(desired_heading_idx_, state_->quat().yaw());
    vars_.output(desired_speed_idx_, 0.0);

    if (spawned_ || t < spawn_time_ || spawn_count_ <= 0) {
        return true;
    }

    // Publish the whole batch once. The selected entity_tag points at a
    // zero-count template in the mission file, and each message applies
    // runtime plugin parameter overrides to that template before spawn.
    spawned_ = true;
    const Eigen::Vector3d anchor = state_->pos();

    for (int index = 0; index < spawn_count_; ++index) {
        const int row = index / columns_;
        const int col = index % columns_;
        publish_spawn(index, row, col, anchor);
    }

    LOG_INFO("RuntimeOverrideSpawner: published " << spawn_count_
             << " GenerateEntity messages for tag '" << entity_tag_ << "'.");
    return true;
}

void RuntimeOverrideSpawner::publish_spawn(
    int index,
    int row,
    int col,
    const Eigen::Vector3d& anchor) {
    const int index_width =
        std::max<int>(1, std::to_string(std::max(0, spawn_count_ - 1)).size());
    const std::string formatted_index = zero_pad_index(index, index_width);

    State spawned_state(*state_);
    spawned_state.pos() = anchor + Eigen::Vector3d(
        x_offset_ + col * x_spacing_,
        y_offset_ + row * y_spacing_,
        z_offset_ + row * z_spacing_);

    // --- Example: Build a GenerateEntity message with plugin overrides ---
    auto msg = std::make_shared<Message<sm::GenerateEntity>>();

    // Set the entity template to spawn from (must exist in mission XML with count=0)
    msg->data.set_entity_tag(entity_tag_);

    // Set the spawn position/orientation
    sc::set(msg->data.mutable_state(), spawned_state);

    // --- Plugin parameter overrides ---
    // Target a plugin by type and name (preferred) or type and index
    auto* plugin_override = msg->data.add_plugin_override();
    plugin_override->set_plugin_type("autonomy");
    plugin_override->set_plugin_name("APITester");  // Target by name, not index

    // Add parameter overrides for the targeted plugin
    auto* int_param = plugin_override->add_params();
    int_param->set_key("my_test_int");
    int_param->set_value(std::to_string(1000 + index));

    auto* csv_param = plugin_override->add_params();
    csv_param->set_key("csv_file_name");
    csv_param->set_value("runtime_override_stress_" + formatted_index + ".csv");

    // Publish the message — SimControl will spawn the entity with overrides applied
    pub_gen_ents_->publish(msg);
}

}  // namespace autonomy
}  // namespace scrimmage