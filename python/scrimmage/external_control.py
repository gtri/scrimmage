"""Provide OpenAI interface for SCRIMMAGE."""
from __future__ import print_function
import threading
import subprocess
import collections
import time
import os
import signal
import sys
import xml.etree.ElementTree as ET
from concurrent import futures

import numpy as np
import grpc

import gym
import gym.spaces
from gym.utils import seeding

from .proto import ExternalControl_pb2, ExternalControl_pb2_grpc


if sys.version[0] == '2':
    import Queue as queue
else:
    import queue as queue


class ServerThread(threading.Thread):
    """Start SCRIMMAGE ExternalControl GRPC Service."""

    def __init__(self, queues, address, max_workers=10):
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
        print('starting python on ', self.address)
        server.add_insecure_port(self.address)
        server.start()

        try:
            while not self.stop:
                time.sleep(1)
            server.stop(0)
        except KeyboardInterrupt:
            server.stop(0)


class ScrimmageEnv(gym.Env):
    """OpenAI implementation for SCRIMMAGE."""

    metadata = {'render.modes': ['human']}

    def __init__(self, enable_gui, mission_file,
                 address="localhost:50051", gdb_args=""):
        """Create queues for multi-threading."""
        self.enable_gui = enable_gui
        self.mission_file = mission_file
        self.address = address
        self.gdb_args = gdb_args

        self.rng = None
        self.seed = self._seed(None)

        queue_names = ['env', 'action', 'action_response']
        self.queues = {s: queue.Queue() for s in queue_names}
        self.server_thread = ServerThread(self.queues, self.address)
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
        _clear_queue(self.queues['action'])
        _clear_queue(self.queues['action_response'])
        _clear_queue(self.queues['env'])

        self._terminate_scrimmage()

        _clear_queue(self.queues['action'])
        _clear_queue(self.queues['action_response'])
        _clear_queue(self.queues['env'])

        self.scrimmage_process = \
            self._start_scrimmage(self.enable_gui, False)

        return self._return_action_result()[0]

    def _step(self, action):
        """Send action to SCRIMMAGE and return result."""
        if isinstance(self.action_space, gym.spaces.Tuple):
            action_pb = ExternalControl_pb2.Action(
                discrete=action[0], continuous=action[1], done=False)
        elif isinstance(self.action_space, gym.spaces.Box):
            action_pb = ExternalControl_pb2.Action(
                continuous=action, done=False)
        else:
            if isinstance(action, collections.Iterable):
                action_pb = ExternalControl_pb2.Action(
                    discrete=action, done=False)
            else:
                action_pb = ExternalControl_pb2.Action(
                    discrete=[action], done=False)
        self.queues['action'].put(action_pb)
        return self._return_action_result()

    def render(self, mode='human', close=False):
        """Ignores a render call but avoids an exception.

        If a user wants the environment rendered then the user should
        pass enable_gui=True as a kwarg
        """
        pass

    def _seed(self, seed=None):
        self.rng, seed = seeding.np_random(seed)
        return [seed]

    def _start_scrimmage(self, enable_gui, disable_output):
        _clear_queue(self.queues['action'])
        _clear_queue(self.queues['action_response'])
        _clear_queue(self.queues['env'])

        port = self.address.split(":")[-1]
        tree = ET.parse(self.mission_file)
        root = tree.getroot()

        # set the seed using this class' random number generator
        seed_node = root.find('seed')
        if not seed_node:
            root.append(ET.Element("foo"))
            seed_node = root.find('seed')
        seed_node.text = str(self.rng.randint(0, 2**32 - 1))

        # enable gui
        run_node = root.find('run')
        run_node.attrib['enable_gui'] = str(enable_gui)
        if not bool(enable_gui):
            run_node.attrib['time_warp'] = "0"
        
        # logging
        log_node = root.find('log_dir')
        try:
            log_node.text = os.path.join(log_node.text, 'external', port)
        except AttributeError:
            log_node = ET.Element("log_node")
            log_node.text = os.path.join('~/.scrimmage/external/logs', port)
            root.append(log_node)

        # disable output
        if disable_output:
            output_node = root.find("output_type")
            output_node.text = ""

        # set port
        for entity_node in root.findall('entity'):
            for autonomy_node in entity_node.findall('autonomy'):
                autonomy_node.attrib['server_address'] = self.address

        self.temp_mission_file = "." + port + os.path.basename(self.mission_file)
        print('temp mission file is ' + self.temp_mission_file)
        tree.write(self.temp_mission_file)
        if self.gdb_args:
            cmd = self.gdb_args.split(" ") + ["scrimmage", self.temp_mission_file]
        else:
            cmd = ["scrimmage", self.temp_mission_file]
        print(cmd)
        return subprocess.Popen(cmd)

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
        if isinstance(self.observation_space, gym.spaces.Tuple):
            size_discrete = self.observation_space.spaces[0].num_discrete_space
            discrete_obs = np.array(res.observations.value[:size_discrete])
            continuous_obs = np.array(res.observations.value[size_discrete:])
            obs = tuple((discrete_obs, continuous_obs))

        else:
            obs = np.array(res.observations.value)

        info = {}
        return obs, res.reward, res.done, info

    def _terminate_scrimmage(self):
        """Terminates scrimmage instance held by the class.

        given how sigints are handled by scrimmage, we need to
        shutdown the network to the autonomy in addition to sending a
        sigint.
        """
        if self.scrimmage_process.returncode is None:
            try:
                os.remove(self.temp_mission_file)
            except OSError:
                pass

            self.queues['action'].put(ExternalControl_pb2.Action(done=True))
            try:
                self.scrimmage_process.kill()
                self.scrimmage_process.poll()
                while self.scrimmage_process.returncode is None:
                    self.scrimmage_process.poll()
                    time.sleep(0.1)
            except OSError:
                print('could not terminate existing scrimmage process. '
                      'It may have already shutdown.')


class ExternalControl(ExternalControl_pb2_grpc.ExternalControlServicer):
    """GRPC Service to communicate with SCRIMMAGE Autonomy."""

    def __init__(self, queues):
        """Receive queues for multi-threading."""
        print('starting ExternalControl')
        self.queues = queues

    def SendEnvironment(self, env, context):
        """Receive Environment proto and send back an action."""
        self.queues['env'].put(env)
        return ExternalControl_pb2.Empty()

    def SendActionResult(self, action_result, context):
        """Receive ActionResult proto and send back an action."""
        self.queues['action_response'].put(action_result)
        try:
            action = self.queues['action'].get(timeout=3.0)
        except queue.Empty:
            res = ExternalControl_pb2.ActionResponse(done=True)
            self.queues['action_response'].put(res)
        return action


def _create_tuple_space(space_params):
    discrete_extrema = []
    continuous_extrema = []

    def _append(param, dst_lst):
        if param.num_dims != 1 and len(param.maximum) == 1:
            # use same min/max for all dims
            dst_lst += param.num_dims * [param.minimum, param.maximum]
        else:
            # each min/max is specified individually
            assert len(param.minimum) == len(param.maximum)
            dst_lst += zip(list(param.minimum), list(param.maximum))

    for param in space_params.params:
        if param.discrete:
            _append(param, discrete_extrema)
        else:
            _append(param, continuous_extrema)

    # make sure that discrete entries are ints
    discrete_extrema = [(int(mn), int(mx)) for mn, mx in discrete_extrema]

    if len(discrete_extrema) == 1 and discrete_extrema[0][0] == 0:
        discrete_space = gym.spaces.Discrete(discrete_extrema[0][1] + 1)
    else:
        discrete_space = gym.spaces.MultiDiscrete(discrete_extrema)

    if continuous_extrema:
        low, high = zip(*continuous_extrema)
        continuous_space = gym.spaces.Box(np.array(low), np.array(high))

    if discrete_extrema and continuous_extrema:
        return gym.spaces.Tuple((discrete_space, continuous_space))
    elif discrete_extrema:
        return discrete_space
    elif continuous_extrema:
        return continuous_space
    else:
        raise NotImplementedError(
            "received a space with no discrete or continuous components")


def _clear_queue(q):
    try:
        while True:
            q.get(False)
    except queue.Empty:
        pass
