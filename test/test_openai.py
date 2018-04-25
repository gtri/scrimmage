"""Example for running openai in scrimmage.

@file

@section LICENSE

Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)

This file is part of SCRIMMAGE.

  SCRIMMAGE is free software: you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.

@author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
@author Eric Squires <eric.squires@gtri.gatech.edu>
@date 31 July 2017
@version 0.1.0
@brief Brief file description.
@section DESCRIPTION
A Long description goes here.
"""

import numpy as np
import gym
import scrimmage


def test_openai():
    """Open single entity scenario and make sure it banks."""
    try:
        env = gym.make('scrimmage-v0')
    except gym.error.Error:
        mission_file = scrimmage.find_mission('rlsimple.xml')

        gym.envs.register(
            id='scrimmage-v0',
            entry_point='scrimmage.external_control:ScrimmageEnv',
            max_episode_steps=1e9,
            reward_threshold=1e9,
            kwargs={"enable_gui": False,
                    "mission_file": mission_file}
        )
        env = gym.make('scrimmage-v0')

    # the observation is the state of the aircraft
    obs = env.reset()
    total_reward = 0
    for i in range(200):

        action = 1 if i < 100 else 0
        obs, reward, done = env.step(action)[:3]
        total_reward += reward

        if done:
            break

    env.close()
    print("Total Reward: %2.2f" % total_reward)
    assert total_reward == 4

if __name__ == '__main__':
   test_openai()
