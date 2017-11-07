import numpy as np
import gym
import scrimmage


def test_openai_tutorial():
   """Open single entity scenario and make sure it banks."""
   try:
       env = gym.make('scrimmage-v0')
   except gym.error.Error:
       mission_file = scrimmage.find_mission('openai.xml')

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

       action = 1 if i < 100 else 2
       obs, reward, done = env.step(action)[:3]
       total_reward += reward

       if done:
           break

   env.env.close()
   print("Total Reward: %2.2f" % total_reward)
   assert total_reward > 0

if __name__ == '__main__':
   test_openai_tutorial()