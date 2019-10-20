import gym
import gym_raas
import numpy as np

env = gym.make('raaspendulum-v0')
env.reset()
for i in range(100):
    observation, reward, done, info = env.step( np.random.rand(1))
    env.render()
env.close()