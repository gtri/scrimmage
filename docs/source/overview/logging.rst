SCRIMMAGE Logging
---------------------

SCRIMMAGE provides several tools for logging and playback of data from a
simulation. A user can specify a SCRIMMAGE root logging directory in a
SCRIMMAGE mission file with the ``log_dir`` XML tag. By default, the root
logging directory is set to ~/.scrimmage/logs. Every time a new simulation is
run, a new directory is created under the root logging directory based on the
current time stamp (e.g., YYYY-MM-DD_HH-MM-SS). A SCRIMMAGE plugin can gain
access to the current simulation's logging directory through the MissionParse
class' ``log_dir()`` method and access to the root log directory though the
``root_log_dir()`` method. For example, from within a plugin, the log
directories are accessed through the following statements:

.. code-block:: c++
   :linenos:

   //Need this include
   #include <scrimmage/parse/MissionParse.h>

   // Get the current simulation's log directory (time-stamped directory)
   std::string mission_log_dir = parent_->mp()->log_dir();

   // Get the root log directory
   std::string root_log_dir = parent_->mp()->root_log_dir();

The types of files written to the current simulation's logging directory
depends on the ``output_type`` XML tags defined in :doc:`xml-tags`. Generally,
this consists of frames.bin (full state history of all entities in simulation),
contact_visual.bin (entity visualization information: meshes, colors, etc.),
utm_terrain.bin (which terrain was loaded), shapes.bin (which shapes were drawn
during the simulation), summary.csv (output from metrics plugins), log.txt
(contains mission pseudorandom seed value), mission.xml (the mission file that
was executed), and runtime_seconds.txt (the runtime for the simulation).


Protocol Buffer Logging / Visualization
=======================================

The ``plot_3d_fr.py`` python script can be used to plot 2D and 3D trajectories
on entities from simulations. Plot the 3D trajectories by pointing to a
directory containing a frames.bin file.

.. code-block:: bash
   :linenos:

   $ ./plot_3d_fr.py ~/.scrimmage/logs/latest

The 2D trajectories can be plotted with the ``--2d`` flag.

.. code-block:: bash
   :linenos:

   $ ./plot_3d_fr.py ~/.scrimmage/logs/latest --2d


.. _csv_logging:

CSV File Logging
================================

The ``CSV`` class can be used in any plugin to log custom data in a comma
separated format. The ``CSV`` class can also be used to read data from a comma
separated file. For a full example of using the ``CSV`` class, see the
``TrajectoryRecordPlayback`` autonomy plugin. The following code examples
briefly describe the ``CSV`` classes API.

CSV File Writing
~~~~~~~~~~~~~~~~

Include the ``CSV`` class:

.. code-block:: c++
   :linenos:

   #include <scrimmage/common/CSV.h>


Create a file to write comma separated data:

.. code-block:: c++
   :linenos:

   scrimmage::CSV csv;

   // Write the CSV file to the root log directory
   std::string filename = parent_->mp()->root_log_dir() + "/my.csv";

   // Create the file
   if (!csv.open_output(filename)) {
      std::cout << "Couldn't create output file" << endl;
   }

Specify the names for the column headers (t, x, y, z):

.. code-block:: c++
   :linenos:

   csv.set_column_headers("t, x, y, z")

Write data to the csv file:

.. code-block:: c++
   :linenos:

   csv.append(sc::CSV::Pairs{
      {"t", t},
      {"x", state_->pos()(0)},
      {"y", state_->pos()(1)},
      {"z", state_->pos()(2)}});


Close the output file when you are done writing data:

.. code-block:: c++
   :linenos:

   csv.close_output();

CSV File Reading
~~~~~~~~~~~~~~~~

When reading in a CSV file, the ``CSV`` class will use the header column names
for indexing columns. If the CSV file doesn't have header columns, the CSV file
can be indexed with integers.

First, read the CSV file:

.. code-block:: c++
   :linenos:

   if (!csv.read_csv(filename)) {
      cout << "Failed to read CSV file: " << filename
           << endl;
   }

Print out the data from the CSV file:

.. code-block:: c++
   :linenos:

   for (int r = 0; r < csv.rows(); r++) {
      cout << "t: " << csv.at(r, "t") << endl;
      cout << "x: " << csv.at(r, "x") << endl;
      cout << "y: " << csv.at(r, "y") << endl;
      cout << "z: " << csv.at(r, "z") << endl;
   }

Real-time Plotting from CSV
~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can plot data from CSV files with the ``csv-plot`` script in the
``scrimmage/scripts`` directory. ``csv-plot`` can plot previously created CSV
files or it can create real-time plots while SCRIMMAGE is running. For example,
to plot the pitch, pitch_rate, yaw, and yaw_rate columns from a CSV file called
``my.csv``, run the following command:

.. code-block:: bash
   :linenos:

   $ ./csv-plot.py -c my.csv -y pitch pitch_rate

The column headers can be assigned to different subplots by running the following
command:

.. code-block:: bash
   :linenos:

   $ ./csv-plot.py -c my.csv -y pitch pitch_rate:1

The ``:X`` after the column header name specifies which subplot to use.
If no subplot is given, the data is plotted on the first subplot.

The y-axis label of each subplot can be set by prefacing the column header name
with the name of the subplot and an equal sign. For example to add a name to our
example plots we would use:

.. code-block:: bash
   :linenos:

   $ ./csv-plot.py -c my.csv -y "pitch (deg)"=pitch "pitch rate (deg/s)"=pitch_rate:1

If no y-axis label is provided for a subplot, the default will be a list of the column
headers used to generate that subplot.

Additionally, python expressions can be used to perform operations on the
data before plotting. For example, to plot the error between measured pitch
and actual pitch, run the following command:

.. code-block:: bash
   :linenos:

   $ ./csv-plot.py -c my.csv -y {measured_pitch - pitch}

To create a 3D plot, use the ``-x``, ``-y``, and ``-z`` flags:

.. code-block:: bash
   :linenos:

   $ ./csv-plot.py -c my.csv -x x -y y -z z -e

The ``-e`` flag tells the plotter to ensure that the x, y, and z axes have
equal scales.

By default, the script looks for the CSV file located in the
~/.scrimmage/logs/latest directory, but the ``-l`` flag can be used to change
the directory. The ``csv-plot.py`` script can also detect when SCRIMMAGE links
the ``latest`` directory to the newest log directory and it will automatically
start plotting data from the newest CSV file. Thus, during development, the
plugin developer can leave the ``csv-plot.py`` script constantly running, while
restarting SCRIMMAGE.

The script can be passed a ``-s`` or ``--static`` flag to disable automatic
plotting of new data. 

Tips for Naming CSV Files
=========================

Since your simulation may consist of multiple entities running the same
plugins, which in turn, could be writing CSV files, it's important to ensure
that each entity writes to a uniquely named CSV file. It is advised the plugin
developer includes the entity's ID in the filename. For example, to create a
CSV file that contains the entity's ID and is written to the current
simulation's log directory, use the following code:

.. code-block:: c++
   :linenos:

   std::string csv_filename = parent_->mp()->log_dir() + "/"
                              + std::to_string(parent_->id().id())
                              + "-my.csv"
