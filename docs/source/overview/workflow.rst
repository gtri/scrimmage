.. _scrimmage_workflow:

SCRIMMAGE Workflow
------------------

SCRIMMAGE ships with a number of tools to help you simulate, playback, and
analyze simulation results. The following is a description of SCRIMMAGE's
tools and how they may be used in a typical workflow:

.. Note::
   
   If scrimmage is not installed on your system, please navigate to
   :ref:`install_scrimmage`. If you don't know if scrimmage is installed, enter
   the following command into a terminal: ::

     $ scrimmage
     
   If you receive the message, ``scrimmage: command not found``, you will have
   to install scrimmage.

- **scrimmage** : The main work horse of scrimmage. Simulates scrimmage
  missions.  Example usage: ::

    $ scrimmage ~/scrimmage/scrimmage/missions/straight.xml

- **scrimmage-playback** : Used to replay scrimmage simulations from saved
  Protocol Buffer binary files. Every time a simulation is executed, a
  time-stamped directory is created in the ``log_dir`` location (defined in the
  mission file. This is ``~/.scrimmage/logs`` by default). The time-stamped
  directory contains a .bin file, which contains the trajectories of all
  entities, a log.txt file, which contains the number that was used to seed
  scrimmage's random number generator, and a mission.xml file, which contains
  an exact copy of the mission used to run the simulation. Usage: ::
   
    $ scrimmage-playback ~/.scrimmage/logs/2016-11-01_14-53-27

- **./scripts/plot_3d_fr.py** : Used to plot the recorded trajectories of the
  entities. The resulting plot can be difficult to visualize if the simulation
  contained a large number of aircraft. This plot is typically used for reports
  and testing low-level controllers. Example usage: ::

    $ cd /path/to/scrimmage/scripts
    $ ./plot_3d_fr.py ~/swarm-log/2017-07-20_15-37-25/frames.bin
    $ ./plot_3d_fr.py ~/swarm-log/2017-07-20_15-37-25/frames.bin --2d  # 2D mode

- **aggregate-runs** : The ``aggregate-runs`` tool can be executed on a
  directory containing many runs in order to calculate the number of wins for
  each team. The ``aggregate-runs`` program creates text
  files at ``~/.scrimmage/logs/aggregate/wins`` that specify the scenarios in
  which each team won. Example usage: ::
   
    $ aggregate-runs -d ~/.scrimmage/logs

- **filter-runs** : Reads the generated text files and allows for easy playback
   of each type of scenario: ::

     $ filter-runs ~/.scrimmage/logs

   Running ``filter-runs`` will print output similar to the following: ::

     ====================================================
     Choose an outcome number: 
     ----------------------------------------------------
     Number		Name		Win Count
     ----------------------------------------------------
     [0]		draw		1
     [1]		team_1		7
     [2]		team_2		3     
     >> 

   To view the simulation runs where ``team_1`` was the winner, type the number
   ``1``, hit enter, and follow the remaing prompts to step through the
   playback of each simulation.
   
- **scrimmage-plugin** : Display information about a plugin. This program can
  help you debug the location of SCRIMMAGE plugins. Provide this program with a
  plugin name. For example: ::
  
    $ scrimmage-plugin Straight

  will produce something like the following: ::

    ==========================================
    Plugin found.
    Name: Straight
    File: /home/myuser/scrimmage/scrimmage/include/scrimmage/plugins/autonomy/Straight/Straight.xml
    Library: Straight_plugin
    -------------------------
    Params: XML_DIR=/home/myuser/scrimmage/scrimmage/include/scrimmage/plugins/autonomy/Straight/
    library=Straight_plugin
    save_camera_images=false
    show_camera_images=true
    speed=21

