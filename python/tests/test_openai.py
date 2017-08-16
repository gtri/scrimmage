"""Tests for openai interface."""
import os

import numpy as np
import gym


def test_openai():
    """Open single entity scenario and make sure it banks."""
    try:
        env = gym.make('scrimmage-v0')
    except gym.error.Error:
        mission_file = os.path.join(
            os.path.dirname(__file__), '..', '..',
            'missions', 'external_control.xml')

        gym.envs.register(
            id='scrimmage-v0',
            entry_point='scrimmage.external_control:ScrimmageEnv',
            max_episode_steps=1e9,
            reward_threshold=1e9,
            kwargs={"enable_gui": False, "mission_file": mission_file}
        )
        env = gym.make('scrimmage-v0')

    ROLL_IDX = 8
    YAW_IDX = 8

    # the observation is the state of the aircraft
    observation = env.reset()[0]
    for _ in range(100):
        action = env.action_space.sample()

        # the action is the desired state of the aircraft
        # we tell it to turn hard left
        action = observation
        action[1][YAW_IDX] += np.pi / 2
        observation = env.step(action)[0]

    env.env.close()
    ROLL_LIMIT = np.radians(30)
    assert abs(observation[1][ROLL_IDX]) > 0.9 * ROLL_LIMIT

if __name__ == '__main__':
    test_openai()
