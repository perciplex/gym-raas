# RaaS Gym Environments
These are the gym environments designed to be used with RaaS.

Currently, we have the pendulum-v0 implementation.

# Installation
```
cd raas-envs
pip install -e .
```

# Use

```
import gym
import raas_envs
env_pend = gym.make('pendulum-v0')
```
