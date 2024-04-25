import numpy as np

import gymnasium as gym
from gymnasium import spaces


class TrafficEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self, traffic_map, render_mode=None):
        self.traffic_map = traffic_map
        self.observation_space = self.traffic_map.get_observation_space()
        self.action_space = self.traffic_map.get_action_space()

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
