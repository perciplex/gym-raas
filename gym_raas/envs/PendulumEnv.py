import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
import os

"""

Pendulum class. Continuous action space. Meant to recreate Pendulum-v0,
which is implemented here:

https://github.com/openai/gym/blob/master/gym/envs/classic_control/pendulum.py

Most of it is kept the same (reward structure, etc), except the part that has
to actually interface with the env.

"""


class PendulumEnv(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 30}
    # We should perhaps set hardware via an environment variable?
    def __init__(self, g=10.0, hardware=False):

        if "RAASPI" in os.environ:  # Check if RAASPI environment variable is set
            print("Hardware mode is active!")
            hardware = True

        self.hardware = hardware

        self.max_speed = 8
        self.max_torque = 2.0
        self.dt = 0.05
        self.g = g
        self.state = None
        high = np.array([1.0, 1.0, self.max_speed])
        self.action_space = spaces.Box(
            low=-self.max_torque, high=self.max_torque, shape=(1,), dtype=np.float32
        )
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        # Create Motor and Encoder object
        if self.hardware:
            import zmq

            context = zmq.Context()

            #  Socket to talk to server
            print("Connecting to motor driver server...")
            self.socket = context.socket(zmq.REQ)
            self.socket.connect("tcp://172.17.0.1:5555")

        else:
            self.viewer = None

            self.seed()
            # See comment in random() below about random initial conditions.

        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, u):
        g = self.g
        m = 1.0
        l = 1.0
        dt = self.dt

        u = np.clip(u, -self.max_torque, self.max_torque)[0]
        self.last_u = u  # for rendering

        if not self.hardware:
            th, thdot = self.state  # th := theta
            newthdot = (
                thdot
                + (-3 * g / (2 * l) * np.sin(th + np.pi) + 3.0 / (m * l ** 2) * u) * dt
            )
            newthdot = np.clip(
                newthdot, -self.max_speed, self.max_speed
            )  # pylint: disable=E1111
            newth = th + newthdot * dt

        elif self.hardware:
            print("Sending motor command for torque ".format(u))

            self.socket.send_pyobj(("Command", u))
            _ = self.socket.recv_pyobj()
            x, y, newthdot = self._get_obs()
            newth = np.arctan2(y, x)

        #costs = angle_normalize(th) ** 2 + 0.1 * thdot ** 2 + 0.001 * (u ** 2)
        costs = angle_normalize(newth) ** 2 + 0.1 * newthdot ** 2 + 0.001 * (u ** 2)
        self.state = np.array([newth, newthdot])
        return self._get_obs(), costs, False, {}

    def reset(self):
        # Currently, uses randomness for initial conditions. We could either
        # remove this aspect, or do something like create initial randomness by
        # doing a quick sequence of actions before starting the episode, that
        # would effectively start it in a random state.
        if not self.hardware:
            high = np.array([np.pi, 1])
            self.state = self.np_random.uniform(low=-high, high=high)
            self.last_u = None
            return self._get_obs()
        elif self.hardware:
            print("Sending motor command to stop")

            self.socket.send_pyobj(("Command", 0))
            _ = self.socket.recv_pyobj()

            return self._get_obs()

    def _get_obs(self):

        if not self.hardware:
            theta, thetadot = self.state
            return np.array([np.cos(theta), np.sin(theta), thetadot])
        elif self.hardware:
            print("Sending request for obs")
            self.socket.send_pyobj(("Poll", None))

            #  Get the reply.
            theta, thetadot = self.socket.recv_pyobj()

            # Do we want to clip the measurement?
            thetadot = np.clip(thetadot, -self.max_speed, self.max_speed)
            return np.array([np.cos(theta), np.sin(theta), thetadot])

    def render(self, mode="human"):
        # We need to figure out this path asset
        if not self.hardware:
            if self.viewer is None:
                from gym.envs.classic_control import rendering

                self.viewer = rendering.Viewer(500, 500)
                self.viewer.set_bounds(-2.2, 2.2, -2.2, 2.2)
                rod = rendering.make_capsule(1, 0.2)
                rod.set_color(0.8, 0.3, 0.3)
                self.pole_transform = rendering.Transform()
                rod.add_attr(self.pole_transform)
                self.viewer.add_geom(rod)
                axle = rendering.make_circle(0.05)
                axle.set_color(0, 0, 0)
                self.viewer.add_geom(axle)
                fname = os.path.join(os.path.dirname(__file__), "assets/clockwise.png")
                self.img = rendering.Image(fname, 1.0, 1.0)
                self.imgtrans = rendering.Transform()
                self.img.add_attr(self.imgtrans)

            self.viewer.add_onetime(self.img)
            self.pole_transform.set_rotation(self.state[0] + np.pi / 2)
            if self.last_u:
                self.imgtrans.scale = (-self.last_u / 2, np.abs(self.last_u) / 2)

            return self.viewer.render(return_rgb_array=mode == "rgb_array")

    def close(self):
        # We'd probably want something to destroy the connection/etc to the
        # lower level robot object?
        if not self.hardware:
            if self.viewer:
                self.viewer.close()
                self.viewer = None
        pass


def angle_normalize(x):
    return ((x + np.pi) % (2 * np.pi)) - np.pi
