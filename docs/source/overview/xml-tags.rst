.. _scrimmage_xml:

Mission XML Tag Definitions
===========================

Either the ``latitude``, ``longitude``, and ``altitude`` tags must be used
together or the ``x``, ``y``, and ``z`` tags must be used together. Do not mix
and match between the two paradigms for a single entity.

- ``run`` : Defines timing parameters for simulation

  attributes:

  - ``start``: The start time, should usually be set to 0.0.
  - ``end``: Final time of simulation, in seconds.
  - ``dt`` : Change in simulation time for each time step, in seconds.
  - ``time_warp`` : Specifies how much slower or faster than real-time that the
    simulation should run. If set to 1, the simulation runs in real-time. If
    set to 0, the simulation runs as fast as possible. If set to x, it runs x
    times faster than real-time. For example, if set to 10, the simulation will
    run 10 times as fast as real-time. If set to 0.5, the simulation will run
    half as fast as real-time. It should be noted that this parameter only
    defines a "best effort." The autonomy logic can and will slow down the
    simulation, but the simulation is guaranteed to run deterministically with
    respect to the ``seed`` XML tag, even if it runs slower than real-time.
  - ``enable_gui`` : If set to ``true``, the SCRIMMAGE gui will run in parallel
    with the SCRIMMAGE simulation. If ``false``, the gui will not run during
    the simulation.
  - ``network_gui`` : If set to ``true``, the SCRIMMAGE simulation controller
    will attempt to connect to a network-based (GRPC) SCRIMMAGE viewer. If
    ``network_gui`` is set to ``true``, ``enable_gui`` should be set to
    ``false``, so that the shared memory version of the SCRIMMAGE viewer isn't
    initialized.
  - ``start_paused`` : If ``true``, the simulation will not start until the
    user types ``b`` into the gui. This allows the user to ensure that the
    simulation's initial conditions are correct before running simulation since
    the gui can launch after the actual simulation begins execution.
  - ``motion_multiplier`` : set to an integer greater than or equal to 1.
      If not 1, scrimmage will run controller and motion plugins
      multiple times for each timestep.
  - ``full_screen`` : If ``true``, display the SCRIMMAGE viewer in full screen.
  - ``window_width`` : If ``full_screen`` is false, sets the SCRIMMAGE
    viewer width. If not specified, defaults to 800.
  - ``window_height`` : If ``full_screen`` is false, sets the SCRIMMAGE
    viewer height. If not specified, defaults to 600.

- ``stream_port`` : When ``network_gui`` is enabled, GRPC messages of the
  positions and orientations of SCRIMMAGE entities will be streamed to this
  port.

- ``stream_ip`` : When ``network_gui`` is enabled, GRPC messages of the
  positions and orientations of SCRIMMAGE entities will be streamed to this
  network address (e.g., localhost, 192.168.1.49, etc.)

- ``seed`` : Used to seed SCRIMMAGE's random number generator. If not specified
  or commented out, the current computer time will be used to seed the
  simulation. In some cases a user will want the scenario to begin deterministically
  and then proceed randomly. This can be achieved by setting a seed but
  adding an attribute ``reseed_time`` for the time in the simulation
  when the seed should be set to something random. Additionally,
  if you want the reseed to be deterministic you can set it with the ``reseed``
  tag.

- ``end_condition`` : Specifies the conditions for ending the simulation. The
  possible end conditions are ``time``, ``one_team``, ``all_dead``, and
  ``none``. If ``time`` is set, the simulation will automatically end when the
  time reaches the ``end`` attribute defined in the ``run`` tag. If
  ``one_team`` is set, the simulation will end when all entities remaining
  belong to the same team ID. If ``all_dead`` is set, the simulation will end
  when there are no more remaining entities. If ``none`` is set, the simulation
  will only end when the user closes the gui or if the user "kills" the
  simulation by typing ``CTRL + c`` in the terminal that launched
  scrimmage. The end conditions can be OR-ed together by separating the end
  conditions with commas. For example, if the ``end_condition`` is set to
  "time, one_team" the simulation will end if the ``end`` time is reached or if
  only a single team is remaining.

- ``terrain`` : Defines the name of the folder that holds the terrain polygon
  and texture data. This is typically located at:
  ``$SCRIMMAGE_DATA_PATH/gui/terrain``.

- ``grid_spacing`` : Defines the distance in meters between grid lines when not
  using the ``terrain`` tag.

- ``grid_size`` : Defines the length in meters of the square grid when not
  using the ``terrain`` tag.

- ``background_color`` : The visualization background color defined in RED,
  GREEN, BLUE order.

- ``gui_update_period`` : The gui update period in milliseconds. Providing a
  small number increases the gui update rate, but can slow the simulation due
  to system resources.

- ``output_type`` : Specifies what artifacts are generated by a simulation
  run. If set to ``frames`` only the frames.bin file will be created. If set to
  ``summary``, a CSV file of the mission metrics will be created. If set to
  ``git_commits``, a hash of the current git project will be created. If set to
  ``all``, all the possible artifacts will be created. If set to nothing no
  output will be created.

- ``show_plugins`` : If ``true``, SCRIMMAGE will print the plugins that were
  discovered on the SCRIMMAGE_PLUGIN_PATH at simulation start-up. The plugins
  will be grouped into similar plugin types (e.g. autonomy plugins, motion
  plugins, interaction plugins, sensors, controllers, etc.)

- ``log_dir`` : The log directory is the folder where scrimmage will save all
  log information from the simulation. This includes entity trajectory data,
  event data, and the random seed that was used during the simulation.

- ``create_latest_dir`` : If ``true``, a symbolically linked directory named
  ``latest`` will be created that links to the most recent ``log_dir``. This
  tag is ``true`` by default. When executing parallel runs, the user should set
  this to ``false``, so that multiple instances of SCRIMMAGE do not try to
  create the same ``latest`` directory.

- ``latitude_origin`` : This is the latitude (decimal degrees) at which the
  simulation's cartesian coordinate system's origin is centered. (e.g.,
  35.721025)

- ``longitude_origin`` : This is the longitude (decimal degrees) at which the
  simulation's cartesian coordinate system's origin is centered. (e.g.,
  -120.767925)

- ``altitude_origin`` : This is the altitude in meters above sea level at which
  the simulation's cartesian coordinate system's origin is centered.

- ``show_origin`` : If set to ``true``, a three axis coordinate system will be
  drawn at the origin of the simulation. This is help for debugging the
  latitude, longitude, and altitude origin values.

- ``display_progress`` : whether to show a progress bar on the command line

- ``origin_length`` : Specifies the length of the origin axes in meters when
  ``show_origin`` is enabled.

- ``metrics`` : This tag loads metrics plugins and executes them during the
  mission. Metrics plugins are executed in the order in which they appear in
  the mission file. A metrics plugin counts "events" in the simulation and
  outputs the counts to the summary CSV file.

- ``entity_interaction`` : This tag loads a plugin that computes interactions
  between entities during the simulation. For example, the ``SimpleCollision``
  entity interaction plugin computes the distance between all entities and if
  the distance between any two entities is below a given threshold, the
  entities are removed from the simulation. The ``GroundCollision`` entity
  interaction plugin removes entities from the simulation that are within a
  given distance from the ground. Entity interaction plugins are executed in
  the order in which they appear in the mission file.

- ``network`` : This tag loads a plugin that simulates network
  communications. This can be used to simulate dropped packets. By default,
  every SCRIMMAGE simulation initializes a "GlobalNetwork," which is used to
  transfer simulation data, such as "EntityGenerated" messages. The
  "LocalNetwork" only allows transmission of messages between plugins that are
  attached to the same entity. The "SphereNetwork" is a probabilistic
  transmission model that is parameterized by communication range. (perfect
  comms within range or probabilistic comms within a range). Multiple network
  tags can be specified in a single mission file.

- ``entity`` : The entity tag is used to initialize a single entity or a swarm
  of homogeneous entities. The following XML tags should be used within the
  ``entity`` tag:

  - ``name`` : A human-readable name for this entity block. This name can be
    used to reference this block later. When running SCRIMMAGE within another
    system, such as ROS or MOOS, this name is used to reference this entity
    block.
  - ``id`` : An optional tag that manually specifies the entity's ID or the
    first ID of an entity block that generates more than a single agent. If two
    entity blocks specify the same ``id``, the entity that is generated first
    is assigned the ID. If the ID is already in use by another agent, the ID is
    incremented until an available entity ID is found.
  - ``team_id`` : The team identification number for this entity group. If
    multiple entity tags are used to initialize a team of heterogeneous
    entities, as long as the ``team_id`` is the same, the entities will be on
    the same team.
  - ``color`` : This is a team color that affects vehicle trail colors and home
    base colors.
  - ``count`` : The number of entities to create.
  - ``generate_rate`` : The rate (in Hz) at which new entities will be
    generated. For example "1 / 2" will generate new entities every two
    seconds.
  - ``generate_count`` : The number of entities that are generated at each time
    interval. If set to ``2``, two entities will be generated every generation
    interval.
  - ``generate_start_time`` : The simulation time at which new entities should
    start to be generated.
  - ``generate_time_variance`` : This tag is used to add "noise" to the
    generation interval. The time between launches is modeled to be distributed
    according to the Gaussian Distribution. All four generate tags have to be
    defined, otherwise, the generate feature will not work properly. See the
    sasc.xml mission for an example configuration.
  - ``autonomy`` : This tag assigns an autonomy plugin for each entity in the
    entity group. Scrimmage searches the SCRIMMAGE_PLUGIN_PATH for an XML file
    with the autonomy tag's name. Multiple autonomy plugins can run in serial
    on a single entity. Autonomy plugins are executed in the order in which
    they appear in the mission file.

    - ``loop_rate`` : This tag sets the autonomy plugin's loop_rate in Hz.
      Scrimmage will attempt to step the plugin at this rate, but cannot
      step a plugin faster than the simulation itself. If the loop_rate is set
      faster than the sim, or is set to 0 or a negative number, it will step
      every time the simulator steps.

  - ``health`` : The initial health points for this entity. The entity
    interaction plugin can affect the health points of each entity depending on
    entity messages and the world state. When the health points decrement to
    zero, the entity is usually removed by the entity interaction plugin.
  - ``variance_x`` : The first entity in an entity group is, by default,
    initialized at the position determined by the the ``x``, ``y``, and
    ``altitude`` tags. Since it can be tedious to specifically initialize large
    numbers of entities, scrimmage has the ability to randomly initialize an
    entity group's starting positions. The ``variance_x``, ``variance_y``, and
    ``variance_z`` tags bound the possible starting locations to a cube around
    the ``x``, ``y``, and ``altitude`` tag position.
  - ``variance_y`` : The y-distance around the entity's initial position for
    generating entities.
  - ``variance_z`` : The z-distance around the entity's initial position for
    generating entities.
  - ``use_variance_all_ents`` : Defaults to ``false``. This tag, if set to
    ``true``, causes the first entity in an entity group to be placed randomly
    according to the same variances and about the same ``x``, ``y``,
    ``altitude`` point as the rest of the entity group.
  - ``x`` : The entity's initial x-position. The first entity in the entity
    group is initialized at this x-position, but other entities in the same
    group are randomly placed around this starting position.
  - ``y`` : The entity's initial y-position.
  - ``z`` : The entity's initial z-position.
  - ``latitude`` : The entity's initial latitude.
  - ``longitude`` : The entity's initial longitude.
  - ``altitude`` : The entity's initial altitude.
  - ``heading`` : The entity's initial heading. Zero-degrees is in the x-axis
    direction. The heading angle increases in a counter-clockwise
    direction. This is different from GPS heading, which increases in a
    clockwise heading. Scrimmage has utilities for easily translating between
    GPS and cartesian coordinate systems.
  - ``motion_model`` : The motion model or "physics" model that describes the
    entity's movement. Scrimmage ships with the following motion models:

    - ``SimpleAircraft`` : An efficient aircraft model that does not allow
      inverted flight.
    - ``SimpleQuadrotor`` : An efficient quadrotor model that does not allow
      inverted flight.
    - ``JSBSimModel`` : A high-fidelity aircraft model that allows inverted
      flight. This plugin ships with an autopilot defined in JSBSim that
      attempts to prevent inverted flight.
    - ``JSBSimControl`` : A high-fidelity aircraft model that allows direct
      control of the actuators on the aircraft. This is the model that allows
      for experimenting with the control of inverted aircraft with realistic
      physics.
    - ``SimpleCar`` : A simple car model
    - ``SingleIntegrator`` : A single integrator model for experimenting with
      motion planning.

  - ``visual_model`` : Loads an XML file that specifies the appearance of the
    entity. Examples: zephyr-blue, zephyr-red, iris, sea-angler, volkswagen.

  - ``controller`` : Loads a low-level controller plugin. Each entity can only
    have a single controller.

    - ``loop_rate`` : This tag sets the controller plugin's loop_rate in Hz.
      See autonomy tag for details.

  - ``sensor`` : Loads a sensor plugin. Multiple sensor plugins can run in
    serial on a single entity.

    - ``loop_rate`` : This tag sets the sensor plugin's loop_rate in Hz.
      See autonomy tag for details.

  - ``base`` : Used to define a "home base" for the entity. Only one home base
    per team should be specified. Entity groups that share a team ID will share
    a home base defined in only one entity group. The ``base`` tag has the
    following tags:

    - ``x`` : The base's x-position
    - ``y`` : The base's y-position
    - ``z`` : The base's z-position
    - ``latitude`` : The base's latitude
    - ``longitude`` : The base's longitude
    - ``altitude`` : The base's altitude
    - ``radius`` : The base's radius

- ``entity_common`` : this is a way to share parameters across entity blocks.
  An ``entity_common`` block must have a ``name`` attribute
  and can have any entries that an ``entity`` block can have.
  For an ``entity`` block to use what is in the ``entity_common`` block,
  the entity must have an attribute ``entity_common="name"`` where ``name``
  matches what is in the attribute for the ``entity_common`` block.

- ``param_common`` : plugins have their own xml files but it can be
  useful to override their attributes in the main mission file.
  For instance, one can override the parameter ``foo`` in the main mission file with
  ``<autonomy foo="bar">MyAutonomyPlugin</autonomy>``. Regardless of what
  is in ``MyAutonomyPlugin.xml``, ``foo`` will have a value of ``bar`` when the
  plugin is initialized. To abstract this further, plugins can share overriden
  values with a ``param_common`` block. A ``param_common`` block needs to have
  a ``name`` attribute (similar to ``entity_common`` blocks). The common block
  can be referenced as
  ``<autonomy param_common="vehicle">MyAutonomyPlugin</autonomy>``
  where ``vehicle`` is the value given to the attribute ``name`` in the ``param_common`` block.

- ``camera`` : defines camera parameters

  attributes:

  - ``mode``: ``follow``, ``free``, or ``offset``
  - ``follow_id``: the entity id for the camera to follow
  - ``pos``: 3 entry comma separated list of where to put the camera
    (applicable to ``free`` mode only)
  - ``focal_point``: 3 entry comma separated list of where to point the camera
    (applicable to ``free`` mode only)
  - ``show_fps``: whether to show frames per second on the gui

- ``enable_screenshots``: if the tag is set to true, scrimmage will save
  screenshots at regular intervals.  This will slow down performance as the
  simulation loop will wait for the gui to save the screenshot before
  proceeding. Note that ``enable_gui`` must be true for this setting to be
  effective. The ``min_period``, ``start``, and ``end`` attributes are
  specified in seconds.

  - ``min_period`` : how often to save screenshots
  - ``start``: when to start taking screenshots
  - ``end``: when to stop taking screenshots.

- ``multi_threaded``: allows scrimmage to run in multiple threads if the tag is set to true (default=``false``).
  The default is for scrimmage to run in a single thread. The attributes are:

  - ``num_threads`` : how many threads to use (``default = 1``)
  - ``autonomy`` : whether to enable or disable running autonomy plugins in threads (``default = true``)
  - ``controller`` : whether to enable or disable running controller plugins in threads (``default = true``)
  - ``motion`` : whether to enable or disable running motion plugins in threads (``default = true``)
  - ``sensor`` : whether to enable or disable running sensor plugins in threads (``default = true``)
