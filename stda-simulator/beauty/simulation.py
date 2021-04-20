import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math


class Simulation:
    def __init(self, sim_params):
         for key, val in sim_params['simulator'].items():
            exec('self.' + key + '= val')
        self.N_steps = int(self.t_end * self.clockrate)
        self.time = 0
    def solve(self, boat, ref):
        return calculate_state_delta_
    def init_integrator(self, boat):
