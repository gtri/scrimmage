import os
import subprocess
import glob
import xml.etree.ElementTree as ET
import multiprocessing as mp

import bayes_opt
import pandas as pd
import numpy as np
import scrimmage
from sklearn.gaussian_process.kernels import RBF, WhiteKernel

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

    for entity_node in root.findall('entity'):
        autonomy_node = entity_node.find('autonomy')
        if autonomy_node.text == 'Predator':
            ratio = nominal_speed / max_speed

            autonomy_node.attrib['max_speed'] = str(max_speed)
            autonomy_node.attrib['capture_range'] = \
                str(nominal_capture_range * ratio / 5.0)

    out_mission = '.optimize' + str(num) + '.xml'
    tree.write(out_mission)

    return out_dir, out_mission


def run(repeats, cores, mission, num,
        nominal_capture_range, nominal_speed, max_speed):

    out_dir, out_mission = \
        create_mission(mission, num, nominal_capture_range,
                       nominal_speed, max_speed)

    scrimmage.parallel(repeats, out_mission, cores)

    files = [os.path.join(out_dir, d, 'summary.csv')
             for d in os.listdir(os.path.expanduser(out_dir))]

    scores = []
    for f in files:
        try:
            scores.append(pd.read_csv(f)['score'].iloc[0])
        except (OSError, IndexError):
            scores.append(0)
    score = np.array(scores).mean()

    return score


def main():
    repeats = 100
    cores = 8
    mission = scrimmage.find_mission('predator_prey_boids.xml')
    nominal_capture_range = 5
    nominal_speed = 35
    kappa = 5       # higher is more exploration, less exploitation

    num_samples = 50
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
