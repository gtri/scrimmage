"""Provide OpenAI interface for SCRIMMAGE."""
from __future__ import print_function
import threading
import warnings
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
                 num_actors=1, port_offset=1,
                 address="localhost:50051", gdb_args=""):
        """Create queues for multi-threading."""
        self.enable_gui = enable_gui
        self.mission_file = mission_file
        self.address = address
        self.gdb_args = gdb_args
        self.num_actors = num_actors
        self.port_offset = port_offset

        self.rng = None
        self.seed = self._seed(None)

        queue_names = ['env', 'action', 'action_response']

        ip, port = address.split(":")
        self.queues = []
        self.server_threads = []
        for i in range(num_actors):
            port = int(port) + i * port_offset
            address = ip + ":" + str(port)
            self.queues.append({s: queue.Queue() for s in queue_names})
            self.server_threads.append(
                ServerThread(self.queues[-1], address))
            self.server_threads[-1].start()

        # startup headless version of scrimmage to get the environment
        for queues in self.queues:
            queues['action'].put(ExternalControl_pb2.Action(done=True))
        self.scrimmage_process = self._start_scrimmage(False, True)

        environments = \
            [self.queues[i]['env'].get() for i in range(num_actors)]
        self._terminate_scrimmage()

        if len(environments) == 1:
            self.action_space, self.observation_space, self.reward_range = \
                self._create_spaces(environments[0])
        else:
            spaces = [self._create_spaces(e) for e in environments]
            action_space, observation_space, reward_range = zip(*spaces)
            self.action_space = gym.spaces.Tuple(action_space)
            self.observation_space = gym.spaces.Tuple(observation_space)
            min_rewards, max_rewards = zip(*reward_range)
            self.reward_range = (min(min_rewards), max(max_rewards))

        signal.signal(signal.SIGINT, self._signal_handler)

    def _create_spaces(self, environment):
        try:
            action_space = \
                _create_tuple_space(environment.action_spaces)
            observation_space = \
                _create_tuple_space(environment.observation_spaces)
            reward_range = (environment.min_reward, environment.max_reward)
            return action_space, observation_space, reward_range
        except AssertionError:
            print(('scrimmage external_control.py: calling terminate '
                   'from __init__ due to env problem'))
            self.close()
            raise

    def _signal_handler(self, signum, frame):
        """Exit cleanly <ctrl-c> (i.e., kill the subprocesses)."""
        print("scrimmage external_control.py: exiting scrimmage from sigint")
        self.close()
        sys.exit(0)

    def _reset(self):
        """Restart scrimmage and return result."""
        self._clear_queues()
        self._terminate_scrimmage()
        self._clear_queues()

        self.scrimmage_process = \
            self._start_scrimmage(self.enable_gui, False)

        if self.num_actors == 1:
            return self._return_action_result(0)[0]
        else:
            ret = []
            for i in range(self.num_actors):
                ret.append(self._return_action_result(i)[0])
            return ret

    def _step(self, action):
        """Send action to SCRIMMAGE and return result."""
        def _step_single(i, space, a):
            if not isinstance(a, collections.Iterable):
                a = [a]

            if isinstance(self.action_space, gym.spaces.Box):
                action_pb =\
                    ExternalControl_pb2.Action(continuous=a, done=False)
            else:
                action_pb =\
                    ExternalControl_pb2.Action(discrete=a, done=False)

            self.queues[i]['action'].put(action_pb)

        if self.num_actors == 1:
            _step_single(0, self.action_space, action)
            return self._return_action_result(0)
        else:
            for i, a in enumerate(action):
                _step_single(i, self.action_space.spaces[i], a)

            data = [self._return_action_result(i) for i in range(len(action))]
            states, rewards, terminals, infos = zip(*data)
            terminal = any(terminals)
            reward = sum(rewards)

            return states, reward, terminal, infos

    def render(self, mode='human', close=False):
        """Ignores a render call but avoids an exception.

        If a user wants the environment rendered then the user should
        pass enable_gui=True as a kwarg
        """
        pass

    def _seed(self, seed=None):
        self.rng, seed = seeding.np_random(seed)
        return [seed]

    def _clear_queues(self):
        for queues in self.queues:
            for q in queues.values():
                _clear_queue(q)

    def _start_scrimmage(self, enable_gui, disable_output):

        ip, port = self.address.split(":")
        tree = ET.parse(self.mission_file)
        root = tree.getroot()

        # set the seed using this class' random number generator
        seed_node = root.find('seed')
        if seed_node is None:
            root.append(ET.Element("seed"))
            seed_node = root.find('seed')
        seed_node.text = str(self.rng.randint(0, 2**32 - 1))

        # enable gui
        run_node = root.find('run')
        run_node.attrib['enable_gui'] = str(enable_gui)
        if not bool(enable_gui):
            run_node.attrib['time_warp'] = "0"

        if self.num_actors > 1:
            multithreaded_node = root.find("multi_threaded")
            if multithreaded_node is None:
                warnings.warn(
                    ("num_actors > 1 requires multithreading. "
                     "Adding to mission file."))
                root.append(ET.Element("multi_threaded"))
                multithreaded_node = root.find("multi_threaded")

            if ("num_threads" not in multithreaded_node.attrib or
               int(multithreaded_node.attrib["num_threads"]) == 1):
                warnings.warn(
                    "num_actors > 1 requires num_actors threads.")
                multithreaded_node.attrib["num_threads"] = str(self.num_actors)

            if multithreaded_node.text != "true":
                warnings.warn(
                    "num_actors > 1 requires multithreading set to true.")
                multithreaded_node.text = "true"

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
        idx = 0
        for entity_node in root.findall('entity'):
            for autonomy_node in entity_node.findall('autonomy'):
                if 'server_address' in autonomy_node.attrib:
                    autonomy_node.attrib['server_address'] = \
                        "{}:{}".format(ip, int(port) + idx * self.port_offset)
                    idx += 1

        if self.num_actors != 1 and idx != self.num_actors:
            raise RuntimeError(
                'num_actors does not match the number of autonomies found ' +
                'in ' + self.mission_file)

        self.temp_mission_file = \
            "." + port + os.path.basename(self.mission_file)
        # print('temp mission file is ' + self.temp_mission_file)
        tree.write(self.temp_mission_file)
        if self.gdb_args:
            cmd = self.gdb_args.split(" ") + \
                ["scrimmage", self.temp_mission_file]
        else:
            cmd = ["scrimmage", self.temp_mission_file]
        # print(cmd)
        return subprocess.Popen(cmd)

    def _close(self):
        """Cleanup spawned threads and subprocesses.

        The thread manages a GRPC server to communicate with scrimmage.  The
        subprocess is the actual scrimmage instance.  This method needs to be
        called in order to make sure a python instance exits cleanly.
        """
        self._terminate_scrimmage()
        for server_thread in self.server_threads:
            server_thread.stop = True

    def _return_action_result(self, index):
        info = {}
        try:
            res = self.queues[index]['action_response'].get(timeout=3)
        except queue.Empty:
            print('Scrimmage Environment: error getting action result')
            res = ExternalControl_pb2.ActionResult(done=True)
            info['scrimmage_err'] = ""

        obs = np.array(res.observations.value)
        return obs, res.reward, res.done, {}

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

            for queues in self.queues:
                queues['action'].put(ExternalControl_pb2.Action(done=True))

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
        self.queues = queues

    def SendEnvironment(self, env, context):
        """Receive Environment proto and send back an action."""
        self.queues['env'].put(env)
        return ExternalControl_pb2.Empty()

    def SendActionResult(self, action_result, context):
        """Receive ActionResult proto and send back an action."""
        self.queues['action_response'].put(action_result)
        if not action_result.done:
            try:
                action = self.queues['action'].get(timeout=10000.0)
            except queue.Empty:
                action = ExternalControl_pb2.Action(done=True)
                res = ExternalControl_pb2.ActionResult(done=True)
                self.queues['action_response'].put(res)
        else:
            action = ExternalControl_pb2.Action(done=True)
            res = ExternalControl_pb2.ActionResult(done=True)
            self.queues['action_response'].put(res)
        return action


def _create_tuple_space(space_params):
    discrete_extrema = []
    continuous_extrema = []

    def _append(param, dst_lst):
        if param.num_dims != 1 and len(param.maximum) == 1:
            # use same min/max for all dims
            dst_lst += param.num_dims * [[param.minimum[0], param.maximum[0]]]
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
