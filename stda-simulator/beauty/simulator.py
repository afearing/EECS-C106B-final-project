import numpy as np
import scipy
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math
from boat import *
from environment import *
from controller import *


class Simulator:
    def __init__(self, sim_params):
         for key, val in sim_params['simulator'].items():
            exec('self.' + key + '= val')
        self.environment_state = Environment(sim_params['environment'])
        self.boat = Boat(sim_params, self.environment_state)
        self.controller = Controller(self.stepsize, sim_params['controller'], self.boat, self.environment_state)
        

    def simulate(self):
        return scipy.integrate.solve_ivp(fun=self.solve, t_span=(0,self.t_end), y0=boat.get_state(), method='RK45', t_eval=np.linspace(0, self.t_end, self.stepsize)))

    def solve(self, time, boat_state):
        self.environment_state.time = time
        self.boat.set_state(boat_state)
        self.boat.calculate_forces(self.environment_state)

        # controller in here maybe
        return self.boat.calculate_state_delta(self.controller)

