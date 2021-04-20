import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math


class Controller:
    def __init__(self, controller_params):
        for key, val in controller_params.items():
            exec('self.' + key + '= val')
        self.rudder_angle = 0
        self.sail_angle = 0