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
        self.boat = Boat(sim_params['boat'], self.environment_state)
        self.controller = Controller(sim_params['controller'])


        self.N_steps = int(self.t_end * self.clockrate)
        # initialize data arrays. Might put these in boat but leave them in the Simulator class for now
        self.x = np.zeros((14, self.N_steps + 1))
        self.r = np.zeros(N_steps + 1)
        self.sail = np.zeros(N_steps + 1)
        self.t = np.zeros(N_steps + 1)
        self.x[:, 0] = self.boat.get_state()
        self.ref_heading = np.zeros(N_steps + 1)
        

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