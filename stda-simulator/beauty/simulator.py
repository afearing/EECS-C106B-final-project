import numpy as np
import scipy
from scipy import integrate
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math
from boat import *
from environment import *


class Simulator:
    def __init__(self, sim_params):
        for key, val in sim_params['simulator'].items():
            exec('self.' + key + '= val')
        self.environment_state = Environment(sim_params['environment'])
        self.boat = Boat(sim_params, self.environment_state)
        

    def simulate(self):
        return scipy.integrate.solve_ivp(fun=self.solve, t_span=(0,self.t_end), y0=self.boat.get_state(), method='RK45', ), self.environment_state # t_eval=np.linspace(0, self.t_end, self.stepsize))

    def solve(self, time, boat_state):
        self.environment_state.time = time
        self.environment_state.step_counter += 1
        self.boat.set_state(boat_state, self.environment_state)
        return self.boat.calculate_state_delta(self.environment_state)
