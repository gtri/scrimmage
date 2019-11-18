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
        repeats = 100
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

        out_dir, out_mission = \
            create_mission(mission, num, nominal_capture_range,
                           nominal_speed, max_speed)

        parallel(repeats, out_mission, cores)

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

        tree = ET.parse(mission)
        root = tree.getroot()

        seed_node = root.find('seed')
        if seed_node:
            root.remove(seed_node)

        run_node = root.find('run')
        run_node.attrib['enable_gui'] = "false"
        run_node.attrib['time_warp'] = "0"

        log_dir_node = root.find('log_dir')
        out_dir = os.path.join(log_dir_node.text, 'optimize' + str(num))
        log_dir_node.text = out_dir

        capture_range = nominal_capture_range * ratio / 5.0

        for entity_node in root.findall('entity'):
            autonomy_node = entity_node.find('autonomy')
            if autonomy_node.text == 'Predator':
                ratio = nominal_speed / max_speed

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
        1 | 00m34s |    0.00000 |     10.0000 | 
        2 | 00m34s |    1.68317 |     31.1111 | 
        3 | 00m28s |    7.65347 |     52.2222 | 
        4 | 00m32s |    7.67327 |     73.3333 | 
        5 | 00m31s |    6.08911 |     94.4444 | 
        6 | 00m31s |    4.94059 |    115.5556 | 
        7 | 00m29s |    3.59406 |    136.6667 | 
        8 | 00m31s |    2.56436 |    157.7778 | 
        9 | 00m32s |    2.29703 |    178.8889 | 
       10 | 00m35s |    2.07921 |    200.0000 | 
       11 | 00m32s |    0.00000 |     18.4934 | 
    Bayesian Optimization
    -------------------------------------------
     Step |   Time |      Value |   max_speed | 
       12 | 00m32s |    7.55446 |     62.8933 | 
       13 | 00m32s |    7.61386 |     68.0475 | 
       14 | 00m35s |    8.17822 |     65.6019 | 
       15 | 00m35s |    7.87129 |     64.1568 | 
       16 | 00m33s |    7.79208 |     63.3237 | 
       17 | 00m35s |    8.00000 |     63.1199 | 
       18 | 00m33s |    7.99010 |     62.5929 | 
       19 | 00m34s |    7.47525 |     62.2671 | 
       20 | 00m34s |    7.87129 |     63.8412 |

The best speed found so far is 65.6019 (note that we could have had more
exploration by setting ``kappa`` to something higher). We can either continue
the search process with more or use a function minimizer to minimize the
Gaussian Process to get a final estimate of optimal value.

.. [Boids] Reynolds, Craig W. "Flocks, herds and schools: A distributed behavioral model." ACM SIGGRAPH computer graphics. Vol. 21. No. 4. ACM, 1987.
.. [Rasmussen] Rasmussen, Carl Edward. "Gaussian processes in machine learning." Advanced lectures on machine learning. Springer, Berlin, Heidelberg, 2004. 63-71.
.. [Forrester] Forrester, Alexander, and Andy Keane. Engineering design via surrogate modelling: a practical guide. John Wiley & Sons, 2008.
.. [Parallel] Tange, Ole. "Gnu parallel-the command-line power tool." The USENIX Magazine 36.1 (2011): 42-47.
