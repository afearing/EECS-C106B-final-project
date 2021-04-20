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
    def __init(self, sim_params):
         for key, val in sim_params['simulator'].items():
            exec('self.' + key + '= val')
        self.environment_state = Environment(sim_params['environment'])
        self.boat = Boat(sim_params['boat'], self.environment_state)
        self.N_steps = int(self.t_end * self.clockrate)
        self.controller = Controller(sim_params['controller'])
        

    def simulate(self):
        for idx in range(self.N_steps):
            speed = self.boat.calculate_speed()
            drift = 

    def solve(self, time, boat_state):
        self.environment_state.time = time
        self.boat.set_state(boat_state)
        self.boat.calculate_forces(self.environment_state)
        return self.boat.calculate_state_delta(self.controller)

    def init_integrator(self, boat):
        integrator = scipy.integrate.ode(self.solve).set_integrator('dopri5')
        integrator.set_initial_value(boat.get_state(), 0)