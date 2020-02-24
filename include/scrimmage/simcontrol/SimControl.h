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
#include <unordered_map>

namespace scrimmage {

typedef std::shared_ptr<scrimmage_proto::ContactVisual> ContactVisualPtr;

enum class EndConditionFlags {TIME = 1, ONE_TEAM = 2, NONE = 3, ALL_DEAD = 4};

class SimControl {
 public:
    /// @brief SimControl default constructor
    SimControl();

    /**
     * @brief Initialize a scrimmage simulation.
     *
     * @param [in] mission_file The scrimmage mission file for the
     * simulation. The mission_file can be a relative path, absolute path, or a
     * scrimmage mission file that is already on the SCRIMMAGE_MISSION_PATH.
     *
     * @param [in] init_python If true, will call Py_Initialize() and
     * initialize the Python interface. Py_Initialize() should only be called
     * once per process.
     *
     * When using SimControl to run a scrimmage mission, this init() function
     * must always be called with a mission_file to properly initialize the
     * simulation. After calling init(), the API supports running scrimmage in
     * its own thread (e.g., using the run_threaded() function) or the
     * controlling program can step the simulation manually (e.g., using the
     * run_single_step() function).
     */
    bool init(const std::string& mission_file, const bool& init_python = true);

    /**
     * @brief Starts the mission by generating entities and setting up logging.
     *
     * Initializes entities, sets up logging, initializes the time.
     */
    bool start();

    /**
     * @brief Runs the scrimmage simulation in the current thread.
     *
     * Example API usage:
     *
     *  \code{.cpp}
     *  SimControl simcontrol;
     *  simcontrol.init("./missions/straight.xml");
     *  simcontrol.run();
     *  simcontrol.shutdown();
     *  \endcode
     */
    bool run();

    /**
     * @brief Runs the scrimmage simulation in a separate thread
     *
     * Example API usage:
     *
     *  \code{.cpp}
     *  SimControl simcontrol;
     *  simcontrol.init("./missions/straight.xml");
     *  simcontrol.run_threaded();
     *
     *  ... Do other work ...
     *
     *  simcontrol.force_exit();
     *  simcontrol.join();
     *  simcontrol.shutdown();
     *  \endcode
     */
    void run_threaded();

    /**
     * @brief Runs the scrimmage simulation by a single time step
     *
     * Example API usage:
     *
     *  \code{.cpp}
     *  SimControl simcontrol;
     *  simcontrol.init("./missions/straight.xml");
     *  simcontrol.start();
     *
     *  for (int i = 0; i < 100; i++) {
     *      simcontrol.run_single_step(i);
     *  }
     *  simcontrol.shutdown();
     *  \endcode
     */
    bool run_single_step(const int& loop_number);

    /**
     * @brief Finalizes the simulation, closes logs, closes plugins.
     *
     * @param [in] shutdown_python If true, will call Py_Finalize() and
     * shutdown the Python interface. Py_Finalize() should only be called once
     * per process and after Py_Initialize() was called.
     *
     * The shutdown() function should be called after the simulation is
     * complete.
     */
    bool shutdown(const bool& shutdown_python = true);

    /**
     * @brief Force a threaded simulation to exit.
     *
     * When running the simulation in a separate thread, this function sends a
     * signal to the main simulation loop to exit.
     */
    void force_exit();

    /// @brief Waits for the threaded simulation to exit.
    void join();

    /*
    * @brief Provides access to the SimControl's entity plugin.
    *
    * After SimControl has been initialized with the init() function, this
    * function provides access to the entity plugin owned by SimControl. This
    * entity plugin can be used to access the publish/subscribe bus used during
    * the simulation. For example, when used during a test, the test can
    * publish and subscribe to messages used by plugins under test.
    */
    EntityPluginPtr plugin();

    /**
     * @brief Provides access to the simulated contacts.
     *
     * The contact map is indexed by contact ID.
     */
    void get_contacts(std::unordered_map<int, Contact> &contacts);

    /**
     * @brief Provides access to the simulated contacts visualization
     * information.
     *
     * The contact visuals map is indexed by contact ID.
     */
    void get_contact_visuals(std::map<int, ContactVisualPtr> &contact_visuals);

    /**
     * @brief Generate entities based on the current time.
     *
     * @param [in] t Current simulation time.
     *
     * Determines the entities that should be generated at the given time and
     * generates the entities.
     */
    bool generate_entities(const double& t);

    /// @brief Generate an entity given the entity description ID.
    bool generate_entity(const int &ent_desc_id);

    /**
     * @brief Generate an entity given the entity description ID and
     * parameters.
     */
    bool generate_entity(const int &ent_desc_id,
                         std::map<std::string, std::string> &params);

    /// @brief Get the pointer to the MissionParser instance.
    MissionParsePtr mp();

    /**
     * @brief Set the simulation time
     *
     * @param [in] t The simulation time
     */
    void set_time(const double &t);

    /// @brief Get the current simulation time.
    double t();

    /// @brief Pause (true) or unpause (false) the simulation.
    void pause(const bool& pause);

    /**
     * @brief Get the paused (true) or unpaused (false) state of the
     * simulation.
     */
    bool paused();

    /// @brief Get the desired time warp of the simulation.
    double time_warp();

    /**
     * @brief Get the actual time warp of the simulation.
     *
     * The simulator will attempt to achieve the desired time warp provided by
     * the time_warp() function. This function returns the actual time warp
     * being used. (WARNING: Not implemented yet. For future use).
     */
    double actual_time_warp();

    /// @brief Returns true if mission file requests a GUI.
    bool enable_gui();

    /// @brief Returns true if a simulation end condition has been met.
    bool end_condition_reached();

    /// @brief Access the simulation timer instance.
    Timer &timer();

    /// @brief Access the metrics plugins.
    std::list<MetricsPtr> & metrics();

    /// @brief Access the PluginManager instance.
    PluginManagerPtr &plugin_manager();

    /// @brief Access the FileSearch instance.
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

    /**
     * @brief Set the incoming interface for communication from external
     * visualizers.
     */
    void set_incoming_interface(InterfacePtr &incoming_interface);

    /// @brief Get the incoming interface.
    InterfacePtr incoming_interface();

    /**
     * @brief Set the outgoing interface for communication to external
     * visualizers.
     */
    void set_outgoing_interface(InterfacePtr &outgoing_interface);

    /// @brief Get the outgoing interface.
    InterfacePtr outgoing_interface();

    /// @brief Access the entities in the simulation.
    std::list<EntityPtr> &ents();

    /// @brief Sends terrain to visualizers and log files.
    void send_terrain();

    /// @brief Sends simulation shapes to visualizers and log files.
    void run_send_shapes();

    /// @brief Enables/disable displaying the current progress in the terminal.
    void display_progress(const bool& enable);

    /*
     * @brief Provides access to the simulated entities.
     *
     * Provies access to the simulated entities, where the map is indexed by
     * the entity.
     */
    std::shared_ptr<std::unordered_map<int, EntityPtr>> id_to_entity_map();

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
    GlobalServicePtr global_services_;

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

    std::set<int> ids_used_ = {0};
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
    void run_send_contact_visuals();
    bool run_networks();

    bool send_shutdown_msg_ = true;

    EntityPluginPtr sim_plugin_;
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

 private:
    bool take_step();
    void single_step(const bool& value);
    bool single_step();
    void set_finished(bool finished);
    bool output_summary();
    bool output_runtime();
    bool output_git_summary();
    void setup_timer(double rate, double time_warp);
    void start_overall_timer();
    void start_loop_timer();
    void loop_wait();
    void inc_warp();
    void dec_warp();

    bool wait_for_ready();
    bool check_output(const std::string& output_type,
                      const std::string& desired_output);
    bool setup_logging();
    bool logging_logic(const std::string &s);
    void end_of_simulation();
    void cleanup();
    bool finalize();
    bool reset_pointers();
    int find_available_id(const std::map<std::string, std::string>& params);

    bool finalized_called_ = false;
    bool running_in_thread_ = false;

    bool python_enabled_ = false;
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_SIMCONTROL_SIMCONTROL_H_
