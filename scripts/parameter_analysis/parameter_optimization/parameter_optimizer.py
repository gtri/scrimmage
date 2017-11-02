from mako.template import Template
from mako.runtime import Context
try:
    from StringIO import StringIO
except ImportError:
    import io
import os
from settings_parser import *
import subprocess
import numpy as np
from collections import OrderedDict
from bayesian_optimization import BayesianOptimizeArgmax
import json
import os.path
import logging
import threading
import time
import os
import signal
import sys
import xml.etree.ElementTree as ET

try:
    import Queue
except ImportError:
    import queue

from concurrent import futures



def generateMissionFile(templateFullFilename, parameterLabels, parameterValues, logPath, missionIteration):
    print(parameterLabels, parameterValues)

    if len(parameterLabels) != len(parameterValues):
        raise ValueError('Parameter Labels and Values are mismatched. (Labels=', len(parameterLabels), 'values=', len(parameterValues))
        quit()

    # Load the mission file
    myTemplate = Template(filename=templateFullFilename)

    try:
        # Add log path
        parameters = dict(zip(parameterLabels + ['log_path'], parameterValues + [logPath + 'iter-' + str(missionIteration)]))

        # Parse in the params
        missionData = myTemplate.render(**parameters)

        # Save the file
        folder = os.path.dirname(os.path.abspath(templateFullFilename))
        filename = os.path.splitext(os.path.basename(templateFullFilename))[0]
        fullFilePath = folder + '/' + filename + str(missionIteration) + '.xml'
        file = open(fullFilePath, 'w')
        file.write(missionData)
        file.close()

        return fullFilePath
    except NameError:
        print('Detected incorrect or missing parameters in mission xml.')
        quit()

# Return xx and yy parsed from file
def parseSamples(file):
    xx = []
    yy = []
    if os.path.isfile(file):
        with open(file) as f:
            f.readline() # skip the header

            for iter, line in enumerate(f):
                values = line.strip().split('=')
                print(values)
                xx.append([float(i) for i in values[0].split(',')])
                yy.append(float(values[1]))
    return xx, yy

def saveSamples(file, xx, yy, header):
    # Write header if first time writing to file (it's a new file)
    if not os.path.isfile(file):
        with open(file, "a") as myfile:
            myfile.write(",".join((str(x) for x in header)) + '\n')

    # Write the known data points
    with open(file, "a") as myfile:
        myfile.write(",".join((str(x) for x in xx)) + '=' + str(yy) + '\n')


# postScrimmageAnalysis gets called on a directory of mission files, not a specific mission, returning an output value
# ranges is a dict of tuple ranges keyed by param name
def optimize(templateFilename, ranges, stateSpaceSampler,
             postScrimmageAnalysis, functionApproximator, logPath,
             numExploreSamples=0, numIterationsPerSample=1, numExploitSamples=0):
    folder = os.path.dirname(os.path.abspath(templateFilename))
    logName = os.path.splitext(os.path.basename(templateFilename))[0]
    samplesFile = folder + '/' + logName + '_samples.log'
    logFile = folder + '/' + logName + '.log'
    logging.basicConfig(filename=logFile, filemode='w', level=logging.DEBUG)

    xx, yy = parseSamples(samplesFile)
    if len(xx) > 0 and len(yy) > 0:
        logging.info('Samples from file'+samplesFile)
        logging.info('Initial xx: ' + ','.join((str(x) for x in xx)))
        logging.info('Initial yy: ' + ','.join((str(x) for x in yy)))

    # Initial new exploration parameters
    logging.info('Sampling State Space')
    new_xx = stateSpaceSampler(ranges, numExploreSamples)
    simulationIter = len(xx)

    for loopIter in range(numExploitSamples+1):
        newBatchStartingIter = simulationIter
        for params in new_xx:
            # Parse sample into template mission
            logging.info('Generating Mission File with params: ' + ','.join((str(x) for x in zip(ranges.keys(),params))))
            missionFile = generateMissionFile(templateFilename, list(ranges.keys()), params, logPath, simulationIter)

            xx.append(params)

            # Run the same params multiple times (in order to get a better average result)
            scrimmageProcesses = []
            for paramScrimmageIter in range(numIterationsPerSample):
                # run scrimmage
                logging.info('Executing Mission File')
                scrimmageProcesses.append(subprocess.Popen(["scrimmage", missionFile]))

            # Wait for all processes to finish
            for process in scrimmageProcesses:
                process.wait()

            os.remove(missionFile)
            logging.info('Completed Scrimmage Simulations')

            simulationIter+=1

        # analysis for all mission results
        logging.info('Parsing Results')
        for iter in range(newBatchStartingIter,simulationIter):
            yy.append(postScrimmageAnalysis(logPath+'iter-'+str(iter)))

            # Append the new params and output
            saveSamples(samplesFile, xx[iter], yy[iter], ranges.keys())

        # Use f_hat to guess some optimal params
        logging.info('Approximating function')
        knownArgmax, expectedValue, nextArgmax = functionApproximator(xx, yy, ranges)
        logging.debug('knownArgmax: ' + json.dumps(knownArgmax))
        logging.debug('expectedValue: ' + json.dumps(expectedValue))
        logging.debug('nextArgmax: ' + json.dumps(nextArgmax))

        new_xx = [nextArgmax.values()]
    logging.info('Optimization complete.')
    return knownArgmax, expectedValue

if __name__ == "__main__":
    # Parse the configuration from the settings file
    parser = SettingsParser('settings.json')
    missionFile = parser.getMissionFile()
    logPath = parser.getLogPath()
    sampler = parser.getStateSpaceSampler()
    postMissionAnalyzer = parser.getPostMissionAnalyzer()
    fApprox = parser.getFunctionApproximator()
    numExploreSamples = parser.getNumExploreSamples()
    numIterationsPerSample = parser.getNumIterationsPerSample()
    numExploitSamples = parser.getNumExploitSamples()
    ranges = parser.getRanges()

    knownArgmax, expectedValue = optimize(
        missionFile,
        ranges,
        sampler,
        postMissionAnalyzer,
        fApprox,
        logPath,
        numExploreSamples=numExploreSamples,
        numIterationsPerSample=numIterationsPerSample,
        numExploitSamples=numExploitSamples)

    print('Known Argmax:', knownArgmax)
    print('Expected Value:', expectedValue)
