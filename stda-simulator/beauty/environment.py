import numpy as np
import scipy
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math



class Environment:
    """The class for the environment and its properties. Note that this includes the boat dimensions"""
    def __init__(self, env_params):
        for key, val in env_params['properties'].items():
            exec('self.' + key + '= val')
        for key, val in env_params['disturbances'].items():
            exec('self.' + key + '= val')

        self.time = 0
        self.step_counter = 0
        self.wind_x = self.wind_strength * math.cos(math.radians(self.wind_direction)) # change to degrees
        self.wind_y = self.wind_strength * math.sin(math.radians(self.wind_direction))