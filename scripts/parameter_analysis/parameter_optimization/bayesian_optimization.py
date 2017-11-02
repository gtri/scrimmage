from bayes_opt import BayesianOptimization  # Install with 'sudo pip install bayesian-optimization'
from bayes_opt import helpers
import numpy as np
from collections import OrderedDict
import matplotlib.pyplot as plt
from matplotlib import gridspec

# priorInputs is a list of prior values
# ranges is the range of each value e.g. {'x': (-2, 10)}
# acq can be 'ucb' (Upper Confidence Bound) or 'ei' (Expected Improvement)
# kappa is exploration vs exploitation. 10 -> much exploration, 1 -> extreme exploitation
# Returns (dict of known argmax, expectedValue, dict of potential argmax i.e. the next argmax to try)
def BayesianOptimizeArgmax(priorInputs, priorOutputs, ranges, acq='ucb', kappa=6):
    bo = BayesianOptimization(None, ranges)

    # BO requires a single dict with the keys being the parameters and 'target' which is the output from the function
    # Combine Inputs and Outputs into one dict.. { 'target': [...], 'x': [...], 'y': [...]}
    inputsOutputs = {}
    for sampleIter in range(len(priorInputs)):
        for paramIter, key in enumerate(ranges.keys()):
            if key in inputsOutputs:
                inputsOutputs[key].append(priorInputs[sampleIter][paramIter])
            else:
                inputsOutputs[key] = [priorInputs[sampleIter][paramIter]]
    inputsOutputs['target'] = priorOutputs

    bo.initialize(inputsOutputs)

    # Maximize the function approximation to find the best parameters (0 iterations since we dont have an explicit function to sample)
    bo.maximize(init_points=0, n_iter=0, acq=acq, kappa=kappa)

    # Argmax, return the optimal parameters to try next (based on Explore vs Exploit)
    knownYmax = bo.Y.max()
    knownArgmax = bo.X[bo.Y.argmax()]

    nextArgmax = helpers.acq_max(ac=bo.util.utility, gp=bo.gp, y_max=knownYmax, bounds=bo.bounds)
    return OrderedDict(zip(ranges.keys(), knownArgmax)), \
            knownYmax, \
            OrderedDict(zip(ranges.keys(), nextArgmax))


def posterior(bo, x, xmin=-2, xmax=10):
    xmin, xmax = -2, 10
    bo.gp.fit(bo.X, bo.Y)
    mu, sigma = bo.gp.predict(x, return_std=True)
    return mu, sigma


def plot_gp(bo, x, y):
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        'Gaussian Process and Utility Function After {} Steps'.format(
            len(bo.X)),
        fontdict={'size': 30})

    gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
    axis = plt.subplot(gs[0])
    acq = plt.subplot(gs[1])

    mu, sigma = posterior(bo, x)
    axis.plot(x, y, linewidth=3, label='Target')
    axis.plot(
        bo.X.flatten(),
        bo.Y,
        'D',
        markersize=8,
        label=u'Observations',
        color='r')
    axis.plot(x, mu, '--', color='k', label='Prediction')

    axis.fill(
        np.concatenate([x, x[::-1]]),
        np.concatenate([mu - 1.9600 * sigma, (mu + 1.9600 * sigma)[::-1]]),
        alpha=.6,
        fc='c',
        ec='None',
        label='95% confidence interval')

    axis.set_xlim((-2, 10))
    axis.set_ylim((None, None))
    axis.set_ylabel('f(x)', fontdict={'size': 20})
    axis.set_xlabel('x', fontdict={'size': 20})

    utility = bo.util.utility(x, bo.gp, 0)
    acq.plot(x, utility, label='Utility Function', color='purple')
    acq.plot(
        x[np.argmax(utility)],
        np.max(utility),
        '*',
        markersize=15,
        label=u'Next Best Guess',
        markerfacecolor='gold',
        markeredgecolor='k',
        markeredgewidth=1)
    acq.set_xlim((-2, 10))
    acq.set_ylim((0, np.max(utility) + 0.5))
    acq.set_ylabel('Utility', fontdict={'size': 20})
    acq.set_xlabel('x', fontdict={'size': 20})

    axis.legend(loc=2, bbox_to_anchor=(1.01, 1), borderaxespad=0.)
    acq.legend(loc=2, bbox_to_anchor=(1.01, 1), borderaxespad=0.)
    plt.show()



if __name__ == '__main__':

    if True:
        ranges = OrderedDict({
            'w_pk': (0, 2),
            'w_pr': (0, 2),
            'w_dist': (0, 2),
            'w_dist_decay': (700, 1300)})
        testIn = [[0.76114678841435457, 0.818033407918765, 1.7549922964055682, 979.05896407346995]]
        testOut = [1.1]
        print(BayesianOptimizeArgmax(testIn, testOut, ranges))

    # 1D black box function example
    if False:
        ranges = OrderedDict({'x':(-2, 10)})
        testIn = [[-2, 2.6812, 1.6509, 10]]
        testOut = [0.20166, 1.08328, 1.30455, 0.21180]
        print(BayesianOptimizeArgmax(testIn, testOut, ranges))

    # 2D black box function example
    if False:
        ranges = OrderedDict({'x':(0, 6), 'y': (0, 8)})
        testIn = [[1.2295,2.0411], [1.6756,2.1539], [.57302,.8216], [.22063,2.4784],[.37829,5.7002],[6.0,6.0],[6.0,3.1094],[0.0,6.0],[0.3793,5.4137],[1.5898, 3.6339]]
        testOut = [.05501, .16466, .57302, .22063, .37829, .73542, .14238, .00002, .16007, 1.41949]
        print(BayesianOptimizeArgmax(testIn, testOut, ranges))

    # 3D black box function example
    if False:
        # ranges = {'x': (0, 6), 'y': (0, 8), 'z': (0, 1)}
        # testIn = {
        #     'x': [
        #         1.2295, 1.6756, .57302, .22063, .37829, 6.0, 6.0, 0.0, 0.3793,
        #         1.5898
        #     ],
        #     'y': [
        #         2.0411, 2.1539, .8216, 2.4784, 5.7002, 6.0, 3.1094, 6.0,
        #         5.4137, 3.6339
        #     ],
        #     'z': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        # }

        ranges = OrderedDict({'x': (0, 3), 'y': (3, 5), 'z': (5, 10)})

        testIn = [[2.0,3.0,10.0], [2.0,3.0,10.0], [2.0, 3.0,10.0], [2.0,3.0,10.0], [2.0,3.0,10.0], [2.0, 3.0,10.0], [2.0,3.0,10.0], [2.0,3.0,10.0],[2.0,3.0,10.0],[2.0,3.0,10.0]]
        testOut = [
                .05501, .16466, .57302, .22063, .37829, .73542, .14238, .00002,
                .16007, 1.41949
            ]
        print(BayesianOptimizeArgmax(testIn, testOut, ranges))

    # 2D example
    if False:
        def target(x, y):
            a = np.exp(-( (x - 2)**2/0.7 + (y - 4)**2/1.2) + (x - 2)*(y - 4)/1.6 )
            b = np.exp(-( (x - 4)**2/3 + (y - 2)**2/2.) )
            c = np.exp(-( (x - 4)**2/0.5 + (y - 4)**2/0.5) + (x - 4)*(y - 4)/0.5 )
            d = np.sin(3.1415 * x)
            e = np.exp(-( (x - 5.5)**2/0.5 + (y - 5.5)**2/.5) )
            return 2*a + b - c + 0.17 * d + 2*e

        x = np.linspace(0, 6, 300)
        y = np.linspace(0, 8, 300)
        inputs = [x,y]
        X, Y = np.meshgrid(*inputs)
        x = X.ravel()
        y = Y.ravel()
        X = np.vstack([x, y]).T[:, [1, 0]]
        z = target(x, y)

        bo = BayesianOptimization(target, {'x': (0, 6), 'y': (0, 6)})

        bo.maximize(init_points=5, n_iter=4, acq='ucb', kappa=5)

        # The output values can be accessed with self.res
        print('Best param/output so far:', bo.res['max'])

        utility = bo.util.utility(X, bo.gp, bo.Y.max())
        print('Best param to test next:', X[np.argmax(utility)])

    # 1D graphical example
    if False:
        def target(x):
            return np.exp(-(x - 2)**2) + np.exp(-(x - 6)**2 / 10) + 1 / (x**2 + 1)


        x = np.linspace(-2, 10, 10000).reshape(-1, 1)
        y = target(x)

        bo = BayesianOptimization(target, {'x': (-2, 10)})

        # Additionally, if we have any prior knowledge of the behaviour of
        # the target function (even if not totally accurate) we can also
        # tell that to the optimizer.
        # Here we pass a dictionary with 'target' and parameter names as keys and a
        # list of corresponding values
        bo.initialize({
            'target': [0.20166, 1.08328, 1.30455, 0.21180],
            'x': [-2, 2.6812, 1.6509, 10]
        })

        # utility = Upper Confidence Bound
        # alternative acq = 'ei' (Expected Improvement)
        # kappa = exploration vs exploitation. 10 -> much exploration, 1 -> only tight exploitation
        bo.maximize(init_points=5, n_iter=5, acq='ucb', kappa=5)

        # The output values can be accessed with self.res
        print('Best param/output so far:', bo.res['max'])

        utility = bo.util.utility(x, bo.gp, 0)
        print('Best param to test next:', x[np.argmax(utility)])

        y_max = bo.Y.max()
        x_max = helpers.acq_max(ac=bo.util.utility, gp=bo.gp, y_max=y_max, bounds=bo.bounds)
        print('which should equal', x_max)

        # # Making changes to the gaussian process can impact the algorithm
        # # dramatically.
        # gp_params = {'kernel': None, 'alpha': 1e-5}

        # # Run it again with different acquisition function
        # bo.maximize(n_iter=5, acq='ei', **gp_params)



        plot_gp(bo, x, y)
