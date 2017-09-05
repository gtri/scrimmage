"""Provide OpenAI interface for SCRIMMAGE."""
from __future__ import print_function
import threading
import subprocess
import time
import os
import signal
import sys
import xml.etree.ElementTree as ET

import numpy as np
import queue
import grpc
from concurrent import futures

import gym
import gym.spaces
from gym.utils import seeding

from .proto import ExternalControl_pb2, ExternalControl_pb2_grpc


class ServerThread(threading.Thread):
    """Start SCRIMMAGE ExternalControl GRPC Service."""

    def __init__(self, queues, max_workers=10, address="localhost:50051"):
        """Initialize variables."""
        super(ServerThread, self).__init__()
        self.queues = queues
        self.address = address
        self.max_workers = max_workers
        self.stop = False

    def run(self):
        """Start SCRIMMAGE ExternalControl GRPC Service."""
        server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers))

        ExternalControl_pb2_grpc.add_ExternalControlServicer_to_server(
            ExternalControl(self.queues), server)
        server.add_insecure_port(self.address)
        server.start()

        try:
            while not self.stop:
                time.sleep(1)
            server.stop(0)
            self.queues['stop'].put(True)
        except KeyboardInterrupt:
            server.stop(0)


class ScrimmageEnv(gym.Env):
    """OpenAI implementation for SCRIMMAGE."""

    def __init__(self, enable_gui, mission_file):
        """Create queues for multi-threading."""
        self.enable_gui = enable_gui
        self.mission_file = mission_file

        self.rng = None
        self.seed = self._seed(None)

        self.queues = {s: queue.Queue()
                       for s in ['env', 'action', 'action_response', 'stop']}
        self.server_thread = ServerThread(self.queues)
        self.server_thread.start()

        # startup headless version of scrimmage to get the environment
        self.scrimmage_process = self._start_scrimmage(False, True)
        environment = self.queues['env'].get()
        self._terminate_scrimmage()

        try:
            self.action_space = \
                _create_tuple_space(environment.action_spaces)
            self.observation_space = \
                _create_tuple_space(environment.observation_spaces)
        except AssertionError:
            print('calling terminate from __init__ due to env problem')
            self.close()
            raise

        self.reward_range = (environment.min_reward, environment.max_reward)

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Exit cleanly <ctrl-c> (i.e., kill the subprocesses)."""
        print("exiting scrimmage from sigint...")
        self.close()
        sys.exit(0)

    def _reset(self):
        """Restart scrimmage and return result."""
        self._terminate_scrimmage()

        _clear_queue(self.queues['action'])
        _clear_queue(self.queues['action_response'])
        _clear_queue(self.queues['env'])

        self.scrimmage_process = \
            self._start_scrimmage(self.enable_gui, False)

        return self._return_action_result()

    def _step(self, action):
        """Send action to SCRIMMAGE and return result."""
        action_pb = ExternalControl_pb2.Action(
            discrete=action[0], continuous=action[1], done=False)
        self.queues['action'].put(action_pb)
        return self._return_action_result()

    def _seed(self, seed=None):
        self.rng, seed = seeding.np_random(seed)
        return [seed]

    def _start_scrimmage(self, enable_gui, disable_output):
        tree = ET.parse(self.mission_file)
        root = tree.getroot()

        # set the seed using this class' random number generator
        seed_node = root.find('seed')
        if not seed_node:
            root.append(ET.Element("foo"))
            seed_node = root.find('seed')
        seed_node.text = str(self.rng.randint(0, 2**32 - 1))

        run_node = root.find('run')
        run_node.attrib['enable_gui'] = str(enable_gui)

        if disable_output:
            output_node = root.find("output_type")
            output_node.text = ""

        temp_mission_file = "." + os.path.basename(self.mission_file)
        print('temp mission file is ' + temp_mission_file)
        tree.write(temp_mission_file)
        return subprocess.Popen(["scrimmage", temp_mission_file])

    def _close(self):
        """Cleanup spawned threads and subprocesses.

        The thread manages a GRPC server to communicate with scrimmage.  The
        subprocess is the actual scrimmage instance.  This method needs to be
        called in order to make sure a python instance exits cleanly.
        """
        self._terminate_scrimmage()
        self.server_thread.stop = True

    def _return_action_result(self):
        res = self.queues['action_response'].get()
        size_discrete = self.observation_space.spaces[0].num_discrete_space
        discrete_obs = np.array(res.observations.value[:size_discrete])
        continuous_obs = np.array(res.observations.value[size_discrete:])
        obs = tuple((discrete_obs, continuous_obs))
        info = {}
        return obs, res.reward, res.done, info

    def _terminate_scrimmage(self):
        """Terminates scrimmage instance held by the class.

        given how sigints are handled by scrimmage, we need to
        shutdown the network to the autonomy in addition to sending a
        sigint.
        """
        if self.scrimmage_process:
            temp_mission_file = "." + os.path.basename(self.mission_file)
            try:
                os.remove(temp_mission_file)
            except OSError:
                pass

            self.queues['action'].put(ExternalControl_pb2.Action(done=False))
            self.scrimmage_process.terminate()
            self.scrimmage_process.wait()


class ExternalControl(ExternalControl_pb2_grpc.ExternalControlServicer):
    """GRPC Service to communicate with SCRIMMAGE Autonomy."""

    def __init__(self, queues):
        """Receive queues for multi-threading."""
        print('starting ExternalControl')
        self.queues = queues

    def SendEnvironment(self, env, context):
        """Receive Environment proto and send back an action."""
        self.queues['env'].put(env)
        return ExternalControl_pb2.Empty_pb2()

    def SendActionResult(self, action_result, context):
        """Receive ActionResult proto and send back an action."""
        self.queues['action_response'].put(action_result)
        return self.queues['action'].get()


def _create_tuple_space(space_params):
    discrete_extrema = []
    continuous_extrema = []

    def _append(param, dst_lst):
        if param.num_dims != 1 and len(param.maximum) == 1:
            dst_lst += param.num_dims * [param.minimum, param.maximum]
        else:
            assert len(param.minimum) == len(param.maximum)
            dst_lst += zip(list(param.minimum), list(param.maximum))

    for param in space_params.params:
        if param.discrete:
            _append(param, discrete_extrema)
        else:
            _append(param, continuous_extrema)

    discrete_space = gym.spaces.MultiDiscrete(discrete_extrema)

    low, high = zip(*continuous_extrema)
    continuous_space = gym.spaces.Box(np.array(low), np.array(high))
    return gym.spaces.Tuple((discrete_space, continuous_space))


def _clear_queue(q):
    try:
        while True:
            q.get(False)
    except queue.Empty:
        pass
