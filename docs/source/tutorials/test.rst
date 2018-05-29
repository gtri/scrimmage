.. _scrimmage_test:

Create a SCRIMMAGE Test
=======================

Unit testing is often added to a project to ensure that new code works and
doesn't break existing functionality. A common library for adding tests is
`googletest <https://github.com/google/googletest>`_ where one can test
individual functions or classes. However, it is often only when all of these
functions and classes are combined together in a simulation that edge cases
are found. Thus, SCRIMMAGE provides a high level testing framework to test
mission level functionality. A sample can be found in
``scrimmage/test/test_collisions.cpp``:

.. code-block:: cpp
    :linenos: 

    #include <gtest/gtest.h>

    #include <scrimmage/common/CSV.h>
    #include <scrimmage/simcontrol/SimUtils.h>

    #include <boost/optional.hpp>

    namespace sc = scrimmage;

    TEST(test_angles, rotation) {
        const std::string mission = "straight";
        auto log_dir = sc::run_test(mission);

        bool success = log_dir ? true : false;
        EXPECT_TRUE(success);
        if (!log_dir) return;

        sc::CSV csv;
        bool summary_found = csv.read_csv(*log_dir + "/summary.csv");
        EXPECT_TRUE(summary_found);
        if (!summary_found) return;

        const int row = csv.rows() - 1;
        double collisions = csv.at(row, "team_coll");
        EXPECT_GT(collisions, 0); // expect collisions
    }

Here are links to `gtest <https://github.com/google/googletest>`_, scrimmage
:ref:`csv_logging`, and `boost optional
<https://www.fluentcpp.com/2016/11/24/clearer-interfaces-with-optionalt>`_. We
now walk through what including ``SimUtils.h`` does. The following line
specifies what mission file to run (it uses ``find_mission`` to find the file
at ``scrimmage/missions/straight.xml``) and then runs the mission.  The output
is a boost::optional that can be dereferenced to the output log directory if
the mission was successful:

.. code-block:: cpp

        const std::string mission = "straight";
        auto log_dir = sc::run_test(mission);
        bool success = log_dir ? true : false;
        EXPECT_TRUE(success);
        if (!log_dir) return;

We now need to verify that the simulation passed our test. Here we are checking
that there was at least one collision in the simulation by reading the
``summary.csv`` that is output by a metrics plugin (see :ref:`metrics_plugin`):

.. code-block:: cpp

        sc::CSV csv;
        bool summary_found = csv.read_csv(*log_dir + "/summary.csv");
        EXPECT_TRUE(summary_found);
        if (!summary_found) return;

        const int row = csv.rows() - 1;
        double collisions = csv.at(row, "team_coll");
        EXPECT_GT(collisions, 0); // expect collisions
