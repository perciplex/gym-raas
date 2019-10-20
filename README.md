# RaaS Gym Environments
These are the gym environments designed to be used with RaaS.

Currently, we have the pendulum-v0 implementation.

# Installation
```
cd gym_raas
pip install -e .
```

# Use

```
import gym
import gym_raas
env_pend = gym.make('gym_raas:pendulum-v0')
```
