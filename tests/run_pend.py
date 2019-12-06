import gym
import gym_raas
import numpy as np
import time
import matplotlib.pyplot as plt

env = gym.make("raaspendulum-v0")
env.reset()
obs = []
for i in range(50):
    observation, reward, done, info = env.step([1])
    time.sleep(0.01)
    obs.append(observation)

for i in range(50):
    observation, reward, done, info = env.step([-1])
    time.sleep(0.01)
    obs.append(observation)

obs = np.array(obs)
plt.plot(obs[:, 0])
plt.plot(obs[:, 1])
plt.plot(obs[:, 2])
plt.savefig("books_read.png")
