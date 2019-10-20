from gym.envs.registration import register

register(id="raaspendulum-v0", entry_point="gym_raas.envs:PendulumEnv")
