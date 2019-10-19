from gym.envs.registration import register

register(id="pendulum-v0", entry_point="raas_envs.envs:PendulumEnv")
