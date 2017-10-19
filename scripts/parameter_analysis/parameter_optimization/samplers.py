import pyDOE
import numpy as np

# Uses Latin hypercube sampling to create a list of uniformly and randomly selected samples.
def lhsSampler(ranges, numSamples):
    if numSamples == 0:
        return []

    paramSamples = pyDOE.lhs(len(ranges), samples=numSamples)

    # Vectorize to make faster
    normalizedSamples = []
    for iter, sample in enumerate(paramSamples):
        normalizedParams = []
        for range, param in zip(ranges.values(), sample):
            normalizedParam = float(range[1] - range[0]) * param + range[0]
            normalizedParams.append(normalizedParam)
        normalizedSamples.append(normalizedParams)
    return normalizedSamples

# Returns all possible combinations of parameter values given the ranges and number of samples.
# e.g. ranges [0,2] and [2,4] for 3 samples would output [[0,2],[0,3],[0,4],[1,2],[1,3],[1,4],[2,2],[2,3],[2,4]]
def gridSearch(ranges, numSamples):
    if numSamples == 0:
        return []

    linspaces = []
    for paramRange in ranges.values():
        linspaces.append(np.linspace(paramRange[0], paramRange[1],numSamples))

    # flatten for easier iterating
    grid = [x.flatten() for x in np.meshgrid(*linspaces)]

    numPermutations = grid[0].shape[0]
    numParams = len(ranges)
    samples = [[] for x in range(numPermutations)]
    for permIter in range(numPermutations):
        for paramIter in range(numParams):
            samples[permIter].append(grid[paramIter][permIter])

    return samples
