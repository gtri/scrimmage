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

#ifndef INCLUDE_SCRIMMAGE_SIMCONTROL_SIMCONTROL_H_
#define INCLUDE_SCRIMMAGE_SIMCONTROL_SIMCONTROL_H_
#include <scrimmage/fwd_decl.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/common/Timer.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/Visual.pb.h>

#include <future> // NOLINT
#include <memory>
#include <deque>
#include <vector>
#include <string>
#include <thread> // NOLINT
#include <map>
#include <list>
#include <mutex> // NOLINT

namespace scrimmage {

typedef std::shared_ptr<scrimmage_proto::Shape> ShapePtr;

typedef std::shared_ptr<scrimmage_proto::ContactVisual> ContactVisualPtr;

enum class EndConditionFlags {
    TIME          = 1 << 0,
    ONE_TEAM      = 1 << 1,
    NONE          = 1 << 2,
    ALL_DEAD      = 1 << 3
};

inline EndConditionFlags operator|(EndConditionFlags a, EndConditionFlags b) {
    return static_cast<EndConditionFlags>(static_cast<int>(a) |
                                            static_cast<int>(b));
}

class SimControl {
 public:
    SimControl();
    bool init();
    void start();
    void display_progress(bool enable);
    void run();
    void cleanup();
    bool wait_for_ready();
    void force_exit();
    bool external_exit();
    void join();

    bool finished();
    void set_finished(bool finished);

    void get_contacts(std::unordered_map<int, Contact> &contacts);
    void set_contacts(ContactMapPtr &contacts);

    void get_contact_visuals(std::map<int, ContactVisualPtr> &contact_visuals);
    void set_contact_visuals(std::map<int, ContactVisualPtr> &contact_visuals);

    bool generate_entities(double t);

    void set_mission_parse(MissionParsePtr mp);
    MissionParsePtr mp();
    void set_log(std::shared_ptr<Log> &log);

    bool enable_gui();

    void set_time(double t);
    double t();

    void setup_timer(double rate, double time_warp);
    void start_overall_timer();
    void start_loop_timer();
    void loop_wait();
    void inc_warp();
    void dec_warp();
    void pause(bool pause);
    bool paused();
    double time_warp();
    double actual_time_warp();

    void single_step(bool value);
    bool single_step();

    bool end_condition_reached(double t, double dt);

    Timer &timer();

    std::list<MetricsPtr> & metrics();
    PluginManagerPtr &plugin_manager();
    FileSearch &file_search();

    struct Task {
        EntityPtr ent;
        std::promise<bool> prom;
    };

    bool take_step();

    void step_taken();

    void set_incoming_interface(InterfacePtr &incoming_interface);

    void set_outgoing_interface(InterfacePtr &outgoing_interface);

 protected:
    // Key: Entity ID
    // Value: Team ID
    std::shared_ptr<std::unordered_map<int, int> > team_lookup_;

    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;

    std::thread network_thread_;

    MissionParsePtr mp_;

    std::list<EntityPtr> ents_;

    ContactMapPtr contacts_;

    std::map<int, std::list<ShapePtr> > shapes_;

    std::map<int, ContactVisualPtr> contact_visuals_;

    std::thread thread_;
    bool display_progress_ = false;

    std::string jsbsim_root_;

    double t0_ = 0;
    double tend_ = 0;
    double dt_ = 0;
    double t_ = 0;
    bool paused_;
    bool single_step_;

    bool take_step_ = false;

    Timer timer_;

    bool finished_ = false;
    bool exit_ = false;

    std::mutex finished_mutex_;
    std::mutex contacts_mutex_;
    std::mutex exit_mutex_;
    std::mutex timer_mutex_;
    std::mutex paused_mutex_;
    std::mutex single_step_mutex_;
    std::mutex take_step_mutex_;
    std::mutex time_mutex_;
    std::mutex time_warp_mutex_;
    std::mutex entity_pool_mutex_;

    bool use_entity_threads_ = false;
    int num_entity_threads_ = 0;
    bool entity_pool_stop_ = false;
    std::deque<std::shared_ptr<Task>> entity_pool_queue_;
    std::condition_variable_any entity_pool_condition_var_;
    std::vector<std::thread> entity_worker_threads_;
    void worker();
    void run_entities();

    std::shared_ptr<Log> log_;

    EndConditionFlags end_conditions_ = EndConditionFlags::NONE;

    RandomPtr random_;

    PluginManagerPtr plugin_manager_;

    bool collision_exists(Eigen::Vector3d &p);

    std::shared_ptr<GeographicLib::LocalCartesian> proj_;

    std::list<EntityInteractionPtr> ent_inters_;
    std::list<MetricsPtr> metrics_;

    NetworkPtr network_;

    int next_id_ = 1;
    FileSearch file_search_;
    RTreePtr rtree_;

    void create_rtree();
    void run_autonomy();
    void set_autonomy_contacts();
    void run_dynamics();
    bool run_interaction_detection();
    void run_logging();
    void run_remove_inactive();
    void run_check_network_msgs();
    void run_send_shapes();
    void run_send_contact_visuals();

    bool send_shutdown_msg_ = true;

    PluginPtr pubsub_;
    PublisherPtr pub_end_time_;
    PublisherPtr pub_ent_gen_;
    PublisherPtr pub_ent_rm_;
    PublisherPtr pub_ent_pres_end_;
    PublisherPtr pub_ent_int_exit_;
    PublisherPtr pub_no_teams_;
    PublisherPtr pub_one_team_;

    std::list<EntityPtr> not_ready_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_SIMCONTROL_SIMCONTROL_H_
