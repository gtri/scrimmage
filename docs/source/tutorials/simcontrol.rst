.. _simcontrol_api:

SimControl API
=========================

The ``scrimmage`` executable is a light-weight wrapper around the primary C++
class used to manage the simulation engine: ``SimControl``. The ``SimControl``
class can be used in three contexts: executing ``SimControl`` in its own
thread, executing ``SimControl`` in the current thread, and executing
``SimControl`` in the current thread with manual stepping of the simulation.

.. Note:: The return values of the ``SimControl`` API call should be checked in
          production code (often just a boolean check). The checks are left out
          of the following code samples to improve readability.

Threaded Execution of SimControl
--------------------------------

The following code sample demonstrates how to use the ``SimControl`` class in
non-blocking threaded execution. This is how the simulation is executed when
running with the scrimmage visualization engine because the VTK window needs to
be executed in the primary thread.

.. code-block:: c++
   :linenos:

     // Create an instance of the SimControl class.
     SimControl simcontrol;

     // Initialize the simulation with the straight.xml mission file.
     // SimControl will search on the SCRIMMAGE_MISSION_PATH environment
     // variable for the mission file.
     simcontrol.init("straight");

     // Run the simulation in its own thread. The run_threaded() function  is
     // non-blocking.
     simcontrol.run_threaded();

     // ... Do other work in the current thread ...

     // Tell the simulation to end execution.
     simcontrol.force_exit();

     // Wait for the simulation to stop running internal threads.
     simcontrol.join();

     // Clean up any resources allocated during the simulation.
     simcontrol.shutdown();

Non-Threaded Execution of SimControl
------------------------------------

The following code sample demonstrates how to use the ``SimControl`` class in
blocking non-threaded execution. This is how the simulation is executed when
the scrimmage GUI is not enabled.

.. code-block:: c++
   :linenos:

     // Create an instance of the SimControl class.
     SimControl simcontrol;

     // Initialize the simulation with the straight.xml mission file.
     simcontrol.init("straight");

     // Run the simulation in the current thread. The run() function is
     // blocking.
     simcontrol.run();

     // Clean up any resources allocated during the simulation.
     simcontrol.shutdown();

Manual Stepping of SimControl
-----------------------------

The following code sample demonstrates how to use the ``SimControl`` class in a
manual simulation stepping context. This context is helpful for writing unit
tests that need to check the state of the simulation at each time step.

.. code-block:: c++
   :linenos:

     // Create an instance of the SimControl class.
     SimControl simcontrol;

     // Initialize the simulation with the straight.xml mission file.
     simcontrol.init("straight");

     // Start the simulation timers, setup log directories, etc. The start()
     // function has to be manually called in this case. In the previous
     // examples, the run() function automatically called the start()
     // function.
     simcontrol.start();

     // Step the simulation 100 times.
     for (int i = 0; i < 100; i++) {
         simcontrol.run_single_step(i);
     }

     // Clean up any resources allocated during the simulation.
     simcontrol.shutdown();
