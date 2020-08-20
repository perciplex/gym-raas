from gym.envs.registration import register

# Register the RaaS pendulum env
register(id="RaasPendulum-v0", entry_point="gym_raas.envs:PendulumEnv")

# Register the legacy name
register(id="raaspendulum-v0", entry_point="gym_raas.envs:PendulumEnv")
