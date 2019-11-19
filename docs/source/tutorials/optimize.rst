.. _optimize:

Optimizing Parameters in SCRIMMAGE
==================================

Mission Setup
-------------

Here we investigate a predator-prey scenario and discuss how to use SCRIMMAGE
to optimize model parameters. There are many ways to do this but here 
we focus on optimization using Gaussian Processes. 
For background on Gaussian Processes see [Rasmussen]_. For a similar
discussion on surrogate modeling with an additional discussion of sampling plans
see [Forrester]_. When the parameter space is larger than the 
current application it is recommended to start
with a latin hypercube sampling technique. We do not discuss this here
but note that it is available elsewhere in SCRIMMAGE (see :ref:`multiple_local_runs`
for details).

First let's run the scenario::

    scrimmage missions/predator_prey_boids.xml

The code for the optimization can be found in ``scripts/optimization_tutorial.py``.
To run it you will need to install the python dependences::

    sudo apt install python3-sklearn python3-pandas python-numpy
    sudo pip3 install bayesian-optimization==0.6.0

You should see about 50 blue vehicles (the prey) and 1 red vehicle (the
predator). The blue vehicles are running a version of the Reynolds-Boids model
[Boids]_. The predators attempt to get close enough to the prey to capture
them and in the process get points. There are a couple of parameters 
that affect the efficacy of a predator, namely speed and capture radius.
Both factors generally yield better performance when they are larger.
To create a more interesting scenario we assume that the two factors are 
coupled. In particular, we assume that as the speed is increased
there is a proportionate decrease in capture radius. We want to know
what the optimal speed and capture radius are under this constraint.

To do this note that the ``predator_prey_boids.xml`` file uses metrics plugin
called ``SimpleCaptureMetrics`` that is responsible for writing to 
a ``summary.csv`` file in the log directory. See :ref:`metrics_plugin`
for details of how metrics work. We will be changing the ``max_speed`` 
of the predator autonomy model in order to maximize this output.

To make the problem somewhat more interesting we will introduce noise into the
simulation by randomizing the starting locations (see the ``variance_x``,
``variance_y``, and ``variance_z`` tags). This means that we need 
to repeat simulations to get an estimate of the expected value
of the output. Gaussian Processes can interpolate with noise
by adding a constant to the correlation matrix. We could go with 
this approach since we can estimate the variance of an output
given the repeated runs. However, it is simpler to just let
the library handle this for us. This is the purpose of the WhiteKernel below::

    def main():
        repeats = 16
        cores = 8
        mission = find_mission('predator_prey_boids.xml')
        nominal_capture_range = 5
        nominal_speed = 35
        kappa = 5       # higher is more exploration, less exploitation

        num_samples = 20
        low_speed = 10
        high_speed = 200
        num_explore_points = 10

        def _run(max_speed):
            num = len(glob.glob('.optimize*'))
            return run(repeats, cores, mission, num,
                       nominal_capture_range, nominal_speed, max_speed)

        pbounds = {'max_speed': (10, 200)}
        bo = bayes_opt.BayesianOptimization(_run, pbounds)

        init_explore_points = \
            {'max_speed':  np.linspace(low_speed, high_speed, num_explore_points)}
        bo.explore(init_explore_points)

        gp_params = {'kernel': 1.0 * RBF() + WhiteKernel()}
        bo.maximize(init_points=1, n_iter=num_samples - num_explore_points,
                    kappa=kappa, **gp_params)


    if __name__ == '__main__':
        main()

This code essentially provides the function to optimize ``_run`` to 
the Gaussian Process. ``_run`` captures some variables so that it is
only a function of the optimization variable and calls ``run``::

    def run(repeats, cores, mission, num,
            nominal_capture_range, nominal_speed, max_speed):
        """Runs the missions and aggragate the data from each summary.csv"""
        out_dir, out_mission = \
            create_mission(mission, num, nominal_capture_range,
                           nominal_speed, max_speed)

        parallel(repeats, out_mission, cores)

        # Finds and aggragates the score
        files = [os.path.expanduser(os.path.join(out_dir, d, 'summary.csv'))
                 for d in os.listdir(os.path.expanduser(out_dir))]

        scores = []
        for f in files:
            try:
                if not os.path.exists(f):
                    print("{} does not exists".format(f))
                scores.append(pd.read_csv(f)['score'].sum())
            except (OSError, IndexError):
                scores.append(0)
        score = np.array(scores).mean()

        return score

This function adjusts the ``predator_prey_boids.xml`` file so that 
the constraint is satisfied. In particular, it sets the ``max_speed``
for the predator and makes the corresponding adjustment to ``capture_radius``.
It then calls a helper function ``parallel`` that runs 
SCRIMMAGE instances locally (see [Parallel]_ and :ref:`multiple_local_runs` for
more details). For larger problems where grid engine is available for a cluster,
batch jobs can instead call the ``scrimmage.qsub`` and ``scrimmage.wait_for_job``
functions. Here is the ``create_mission`` function::

    def create_node(tag, text):
        """Create an xml node."""
        el = ET.Element(tag)
        el.text = "{:.2f}".format(text) if isinstance(text, float) else str(text)
        return el


    def create_mission(mission, num, nominal_capture_range, nominal_speed, max_speed):
        """Modify the mission xml with custom parameters"""
        tree = ET.parse(mission)
        root = tree.getroot()

        # Removes the seed for each run
        seed_node = root.find('seed')
        if seed_node != None:
            root.remove(seed_node)

        run_node = root.find('run')
        run_node.attrib['enable_gui'] = "false"
        run_node.attrib['time_warp'] = "0"

        log_dir_node = root.find('log_dir')
        out_dir = os.path.join(log_dir_node.text, 'optimize' + str(num))
        log_dir_node.text = out_dir

        ratio = nominal_speed / max_speed
        capture_range = nominal_capture_range * ratio

        # Applies the max_speed and capture_range attributes to the Predator and SimpleCapture
        for entity_node in root.findall('entity'):
            autonomy_node = entity_node.find('autonomy')
            if autonomy_node.text == 'Predator':
                autonomy_node.attrib['max_speed'] = str(max_speed)
                autonomy_node.attrib['capture_range'] = str(capture_range)

        for interaction_node in root.findall('entity_interaction'):
            if interaction_node.text == 'SimpleCapture':
                interaction_node.attrib['capture_range'] = str(capture_range)

        out_mission = '.optimize' + str(num) + '.xml'
        tree.write(out_mission)

        return out_dir, out_mission

We can now run this file and get the following::

    Initialization
    -------------------------------------------
     Step |   Time |      Value |   max_speed | 
        1 | 05m05s |    0.52941 |     10.0000 | 
        2 | 04m15s |    1.50000 |     31.1111 | 
        3 | 04m04s |    6.88235 |     52.2222 | 
        4 | 04m09s |    6.20000 |     73.3333 | 
        5 | 04m06s |    6.11765 |     94.4444 | 
        6 | 04m11s |    5.52941 |    115.5556 | 
        7 | 04m08s |    6.29412 |    136.6667 | 
        8 | 04m06s |    5.11765 |    157.7778 | 
        9 | 04m06s |    6.05882 |    178.8889 | 
       10 | 04m10s |    4.17647 |    200.0000 | 
       11 | 04m07s |    5.88235 |     36.7489 | 
    Bayesian Optimization
    -------------------------------------------
     Step |   Time |      Value |   max_speed | 
       12 | 04m08s |    6.62500 |    114.1056 | 
       13 | 04m09s |    5.81250 |    111.5516 | 
       14 | 04m08s |    6.64706 |    110.5069 | 
       15 | 04m06s |    7.76471 |     82.7220 | 
       16 | 04m08s |    7.35294 |     79.4004 | 
       17 | 04m09s |    6.41176 |     78.4301 | 
       18 | 04m07s |    6.11765 |     79.6248 | 
       19 | 04m08s |    7.05882 |     81.1437 | 
       20 | 04m06s |    6.58824 |     80.4494 | 
       21 | 04m09s |    6.35294 |     80.4652 |

The best speed found so far is 65.6019 (note that we could have had more
exploration by setting ``kappa`` to something higher). We can either continue
the search process with more or use a function minimizer to minimize the
Gaussian Process to get a final estimate of optimal value.

.. [Boids] Reynolds, Craig W. "Flocks, herds and schools: A distributed behavioral model." ACM SIGGRAPH computer graphics. Vol. 21. No. 4. ACM, 1987.
.. [Rasmussen] Rasmussen, Carl Edward. "Gaussian processes in machine learning." Advanced lectures on machine learning. Springer, Berlin, Heidelberg, 2004. 63-71.
.. [Forrester] Forrester, Alexander, and Andy Keane. Engineering design via surrogate modelling: a practical guide. John Wiley & Sons, 2008.
.. [Parallel] Tange, Ole. "Gnu parallel-the command-line power tool." The USENIX Magazine 36.1 (2011): 42-47.
