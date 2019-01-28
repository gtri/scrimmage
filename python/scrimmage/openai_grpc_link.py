#!/usr/bin/env python
import argparse
import collections
import grpc
import gym
import importlib
import numpy as np
import threading
import time
import signal
import sys

from concurrent import futures
from scrimmage.proto import OpenAI_pb2, OpenAI_pb2_grpc

class ServerThread(threading.Thread):
    """Start SCRIMMAGE OpenAI GRPC Service."""

    def __init__(self, address, actor_str, max_workers=10):
        """Initialize variables."""
        super(ServerThread, self).__init__()
        self.address = address
        self.max_workers = max_workers
        self.stop = False
        self.actor_str = actor_str

    def run(self):
        """Start SCRIMMAGE OpenAI GRPC Service."""
        start = time.time()
        server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers))

        OpenAI_pb2_grpc.add_OpenAIServicer_to_server(
            OpenAIControl(self.actor_str), server)
        server.add_insecure_port(self.address)
        server.start()
        end = time.time()
        print("PYTHON: GRPC Server Started")
        while not self.stop:
            time.sleep(0.5)
        server.stop(0)
        exit()

    def msg_stop(self, signum, stack):
        """Callback function to stop server"""
        self.stop = True

def create_space(space):
    """Converts space params into a Gym space """
    # Get discrete components
    discrete_extrema = []
    for size in space.discrete:
        discrete_extrema.append(size)
    # Get continuous components
    continuous_extrema = []
    for limit in space.continuous:
        continuous_extrema += zip([limit.min], [limit.max])
    # Create Discrete Gym Space
    if len(discrete_extrema) == 1:
        discrete_space = gym.spaces.Discrete(discrete_extrema[0])
    else:
        discrete_space = gym.spaces.MultiDiscrete(discrete_extrema)
    # Create Continuous Gym Space
    if continuous_extrema:
        low, high = zip(*continuous_extrema)
        continuous_space = gym.spaces.Box(np.array(low), np.array(high))
    # Create overall space
    if continuous_extrema and discrete_extrema:
        return gym.spaces.Tuple((discrete_space, continuous_space))
    elif discrete_extrema:
        return discrete_space
    elif continuous_extrema:
        return continuous_space
    else:
        raise NotImplementedError(
            "Received a space with no discrete or continuous components")


class OpenAIControl(OpenAI_pb2_grpc.OpenAIServicer):
    """GRPC Service to communicate with SCRIMMAGE Autonomy."""

    def __init__(self, actor_str):
        """Load up the python module that contains the Actor class."""
        actor_module_str, actor_class_str = actor_str.split(":")
        actor_module = importlib.import_module(actor_module_str)
        self.actor_class = getattr(actor_module, actor_class_str)
        self.actor = None

    def SendEnvironment(self, env, context):
        """Receive Environment proto and return acknowledgement."""
        # Convert to gym environment
        self.obs_space = create_space(env.observation_spaces)
        self.act_space = create_space(env.action_spaces)
        # Initialize Actor
        try:
            self.actor = self.actor_class(self.act_space, self.obs_space,
                                      env.params)
        # Try except is needed to catch any errors in initializing the Actor
        except Exception as e:
            print("There was an error: {}".format(e.message))
        return OpenAI_pb2.Empty()

    def GetAction(self, observation, context):
        """Receive Obs proto and send back an Action."""
        if self.actor is None:
            print("Actor is not initialized")
            return
        if isinstance(self.obs_space, gym.spaces.Tuple):
            obs = [observation.discrete, observation.continuous]
        elif isinstance(self.obs_space, gym.spaces.Box):
            obs = observation.continuous
        else:
            obs = observation.discrete

        action = self.actor.act(obs)
        if not isinstance(action, collections.Iterable):
            action = [action]

        if isinstance(self.act_space, gym.spaces.Box):
            action_proto = OpenAI_pb2.Action(continuous=action, done=False)
        elif isinstance(self.act_space, gym.spaces.Tuple):
            action_proto = OpenAI_pb2.Action(continuous=action.continuous,
                                             discrete=action.discrete, done=False)
        else:
            action_proto = OpenAI_pb2.Action(discrete=action, done=False)

        return action_proto

def main():
    parser = argparse.ArgumentParser(description="OpenAI GRPC Connection")
    parser.add_argument('--actor', type=str, required=True,
                        help='what actor to use for acting. '
                        'Input in the form module:Class (e.g., '
                        'load_model:NNModel) and should have the same basic '
                        'methods/properties as load_model:NNModel')
    parser.add_argument("--port", type=int, default=50051,
                        help="port to start scrimmage GRPC interface on")
    parser.add_argument("--ip", type=str, default="localhost",
                        help="IP Address to connect scrimmage GRPC interface to")
    args = parser.parse_args()

    port = args.port
    ip_address = args.ip
    connections = []
    server_thread = []
    # queue_names = ["env", "action", "action_response"]

    address = ip_address + ":" + str(port)
    # connections = {s: queue.Queue() for s in queue_names}
    server_thread = ServerThread(address, args.actor)
    server_thread.start()

    signal.signal(signal.SIGINT, server_thread.msg_stop)


if __name__ == "__main__":
    main()