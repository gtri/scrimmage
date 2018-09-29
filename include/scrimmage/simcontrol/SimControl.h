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

#include <Eigen/Dense>

#include <scrimmage/fwd_decl.h>

#include <scrimmage/common/Timer.h>
#include <scrimmage/common/DelayedTask.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/Visual.pb.h>

#include <future> // NOLINT
#include <memory>
#include <deque>
#include <vector>
#include <set>
#include <string>
#include <thread> // NOLINT
#include <map>
#include <list>
#include <mutex> // NOLINT
#include <condition_variable> // NOLINT

namespace scrimmage {

typedef std::shared_ptr<scrimmage_proto::ContactVisual> ContactVisualPtr;

enum class EndConditionFlags {TIME = 1, ONE_TEAM = 2, NONE = 3, ALL_DEAD = 4};

class SimControl {
 public:
    SimControl();
    bool init();
    void start();
    void display_progress(bool enable);
    void run();
    bool run_single_step(int loop_number);
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

    bool output_summary();
    bool output_runtime();
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
    void close();

    void single_step(bool value);
    bool single_step();

    bool end_condition_reached();

    Timer &timer();

    std::list<MetricsPtr> & metrics();
    PluginManagerPtr &plugin_manager();
    FileSearchPtr &file_search();

    struct Task {
        // FIXME: this will be much simpler once there is a
        // step function in Plugin.h
        // In particular, we can get rid of Task::Type and
        // more easily do entity_interaction/network plugins in multiple threads.
        enum class Type {AUTONOMY, CONTROLLER, MOTION, SENSOR};

        Type type;
        double t;
        double dt;
        EntityPtr ent;
        std::promise<bool> prom;
    };

    bool take_step();

    void step_taken();

    void set_incoming_interface(InterfacePtr &incoming_interface);
    InterfacePtr incoming_interface();

    void set_outgoing_interface(InterfacePtr &outgoing_interface);
    InterfacePtr outgoing_interface();

    void set_limited_verbosity(bool limited_verbosity);
    std::list<EntityPtr> &ents();

 protected:
    // Key: Entity ID
    // Value: Team ID
    std::shared_ptr<std::unordered_map<int, int>> id_to_team_map_;
    std::shared_ptr<std::unordered_map<int, EntityPtr>> id_to_ent_map_;

    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;

    std::thread network_thread_;

    MissionParsePtr mp_;

    std::list<EntityPtr> ents_;

    ContactMapPtr contacts_;

    std::map<int, std::list<scrimmage_proto::ShapePtr>> shapes_;

    std::map<int, ContactVisualPtr> contact_visuals_;

    std::thread thread_;
    bool display_progress_ = false;

    std::string jsbsim_root_;

    TimePtr time_;
    ParameterServerPtr param_server_;

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

    std::set<Task::Type> entity_thread_types_;
    int num_entity_threads_ = 0;
    bool entity_pool_stop_ = false;
    std::deque<std::shared_ptr<Task>> entity_pool_queue_;
    std::condition_variable_any entity_pool_condition_var_;
    std::vector<std::thread> entity_worker_threads_;
    void worker();
    bool run_entities();

    bool add_tasks(Task::Type type, double t, double dt);

    bool run_sensors();
    bool run_motion(EntityPtr &ent, double t, double dt);
    bool reset_autonomies();

    std::shared_ptr<Log> log_;

    std::set<EndConditionFlags> end_conditions_ = {EndConditionFlags::NONE};

    RandomPtr random_;

    PluginManagerPtr plugin_manager_;

    bool collision_exists(Eigen::Vector3d &p);

    std::shared_ptr<GeographicLib::LocalCartesian> proj_;

    std::list<EntityInteractionPtr> ent_inters_;
    std::list<MetricsPtr> metrics_;
    NetworkMapPtr networks_;
    PubSubPtr pubsub_;

    int next_id_ = 1;
    FileSearchPtr file_search_;
    RTreePtr rtree_;

    void request_screenshot();
    void create_rtree();
    void run_autonomy();
    void set_autonomy_contacts();
    void run_dynamics();
    bool run_interaction_detection();
    bool run_logging();
    bool run_metrics();
    void run_remove_inactive();
    void run_check_network_msgs();
    void run_send_shapes();
    void run_send_contact_visuals();
    bool run_networks();

    bool send_shutdown_msg_ = true;

    PluginPtr sim_plugin_;
    PublisherPtr pub_end_time_;
    PublisherPtr pub_ent_gen_;
    PublisherPtr pub_ent_rm_;
    PublisherPtr pub_ent_pres_end_;
    PublisherPtr pub_ent_int_exit_;
    PublisherPtr pub_no_teams_;
    PublisherPtr pub_one_team_;
    PublisherPtr pub_world_point_clicked_;
    PublisherPtr pub_custom_key_;

    std::list<EntityPtr> not_ready_;
    DelayedTask screenshot_task_;
    bool prev_paused_;

    DelayedTask reseed_task_;
    bool limited_verbosity_;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_SIMCONTROL_SIMCONTROL_H_
