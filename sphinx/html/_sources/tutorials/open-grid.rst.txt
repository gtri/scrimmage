.. _open_grid_engine:

Multiple Runs with Open Grid Engine
------------------------------------

.. Warning:: This tutorial is old and hasn't been updated in a while. While the
   general process is accurate, the program names and arguments may have
   changed. It is recommended that you understand how to run multiple scenarios
   on your local computer first. Please refer to :doc:`multiple-local-runs`
   first.

Running single simulations is helpful for directly observing autonomous
behaviors. However, if you want to statistically compare various algorithms,
you will need to generate hundreds or thousands of runs. Luckily, Scrimmage
ships with scripts that enable it to run on a cluster or a single computer
running Open Grid Engine.

1. Install dependencies on Ubuntu::

     $ sudo apt-get install python-pip python-scipy python-pandas

   Install Python Pandas::

     $ sudo pip install pyDOE

2. Now you need to configure Open Grid Engine (OGE) to run on your local
   machine. Follow the instructions at the following website to configure Open
   Grid Engine for a single computer:
   https://scidom.wordpress.com/2012/01/18/sge-on-single-pc/ . Start at "Part
   1: Installation of SGE" and you can finish before "Part 3: Elementary Test
   Runs on SGE."

   .. Note:: 
      When you run ``sudo qmon`` the GUIs icons may not appear. To fix
      this problem, execute the follow commands in a terminal: ::

        $ mkdir -p $HOME/icons/Qmon
        $ cd $HOME/icons/Qmon
        $ for f in /usr/share/gridengine/pixmaps/*; do ln -sf $f $(basename ${f%.xpm}); done

3. Submit 10 simulations to Open Grid Engine::

     $ cd ~/scrimmage
     
     $ python scrimmage/scripts/generate_scenarios.py \
        --ranges scrimmage/ranges/test-1.xml \
        --num_runs=10 \
        scrimmage-sasc-plugins/missions/capture_the_flag-cpp.xml \
        doe_cache_dir

This will create 10 variations of the ``capture_the_flag-cpp`` mission by
varying parameters found in the `test-1.xml` file. The mission files and
text output from each mission will be saved in ``doe_cache_dir``. The ranges
file is optional.

4. You can observe the status of the submitted jobs::

     $ qstat

   You can delete a running job::

     $ qdel <job-ID>

   Also, you can use the ``sudo qmon`` GUI to monitor and delete jobs in the
   "Job Control" section.

5. The simulation results are stored in the ``log_dir`` location. After the
   simulation is complete, you can use a number of tools to process the
   simulation results. Run the ``aggregate-runs`` program to count the number
   of victories between the teams in the simulations based on the score weights
   defined in an XML file::

     $ aggregate-runs -d ~/swarm-log \
     -w ~/scrimmage/scrimmage-sasc-plugins/config/weights/sasc.xml    

   The ``-w`` flag specifies the XML score weights file to use. The
   ``aggregate-runs`` program creates text files at
   ``~/swarm-log/aggregate/wins`` that specify the scenarios in which each team
   won.

6. The ``filter-runs`` program reads the generated text files and allows for
   easy playback of each type of scenario.::

     $ filter-runs ~/swarm-log

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

The ``generate_scenarios.sh`` generates multiple scenarios from a single base
scenario through a sampling process called "Latin Hypercube Sampling"
(https://en.wikipedia.org/wiki/Latin_hypercube_sampling). While, a base
scenario file is provided, the LHS algorithm needs to know how many and which
parameters can be varied in the base scenario. The "ranges" file defines the
parameters that should be varied in the base mission file and defines minimum
and maximum values for each parameter. An example can be helpful. If we look at
the test-1.xml ranges file located at ``~/scrimmage/scrimmage/ranges/``, we see
several XML tags that have related values in a base scenario file. For example,
we see::

  <x low="0" high="2000" type="float"/>  

This line tells the LHS algorithm that the ``x`` tag in the base mission file
can be varied from 0 to 2000 and that the ``x`` tag is of type float. Most
parameters in the base mission file can be varied in a similar way.
