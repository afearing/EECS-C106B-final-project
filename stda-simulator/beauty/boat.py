import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math

class Boat:
    """The class for the boat state"""
    def __init__(self, boat_params):
        for key, val in boat_params['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
        self.true_sail_angle = False
    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)
    def calculate_true_sail_angle(self, env):
        self.true_sail_angle = np.sign(env.apparent_wind.apparent_angle) * abs(self.sail_angle)
        return self.true_sail_angle