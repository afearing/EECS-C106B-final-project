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
        self.controller = Controller(sim_params, self.boat, self.environment_state) # Yaw controller
        self.sail_angle_reference = Simulator.calculate_sail_angle_reference(self.boat)

    def simulate(self):
        times = None #np.array([])
        boat_states = None #np.array([])
        rudder_angle_references = None #np.array([])
        sail_angle_references = None #np.array([])

        num_steps = int(np.ceil(self.t_end / self.stepsize))
        for step_i in range(num_steps):
            time = step_i * self.stepsize
            # Compute control inputs (sail and rudder angles)
            self.rudder_angle_reference = self.controller.controll(self.boat, self.environment_state, time)
            if not step_i % int(self.boat.sail_sampletime / self.controller.sample_time):
                self.sail_angle_reference = Simulator.calculate_sail_angle_reference(self.boat)
            
            # Step stepsize seconds.
            result = scipy.integrate.solve_ivp(fun=self.solve, t_span=(time, time + self.stepsize), y0=self.boat.get_state(), method='RK45')
            self.boat.set_state(result.y[:, -1], self.environment_state)

            # Save data
            times = np.hstack((times, result.t)) if times is not None else result.t
            boat_states = np.hstack((boat_states, result.y)) if boat_states is not None else result.y

            new_rudders = self.rudder_angle_reference * np.ones((1, len(result.t)))
            rudder_angle_references = np.hstack((rudder_angle_references, new_rudders)) \
                if rudder_angle_references is not None else new_rudders

            new_sails = self.sail_angle_reference * np.ones((1, len(result.t)))
            sail_angle_references = np.hstack((sail_angle_references, new_sails)) \
                if sail_angle_references is not None else new_sails

        return times, boat_states, rudder_angle_references, sail_angle_references, self.environment_state

    def solve(self, time, boat_state):
        # print('{:.3f}, {}'.format(time, np.around(boat_state, 3)))
        # print(np.around(boat_state, 3))
        self.environment_state.time = time
        # self.environment_state.step_counter += 1
        self.boat.set_state(boat_state, self.environment_state)
        return self.boat.calculate_state_delta(self.environment_state, time,
            self.rudder_angle_reference, self.sail_angle_reference)

    def calculate_sail_angle_reference(boat):
        wind_angle = boat.forces.apparent_wind.apparent_angle
        wind_speed = boat.forces.apparent_wind.apparent_speed
        opt_aoa = math.sin(wind_angle)/ (math.cos(wind_angle) + 0.4 * math.cos(wind_angle)**2) * boat.sail_stretching / 4
        if abs(opt_aoa) > boat.stall_deg/180*math.pi:
            opt_aoa = np.sign(wind_angle) * boat.stall_deg/180*math.pi
        if wind_speed > boat.limit_wind_speed:
            fact = (boat.limit_wind_speed / wind_speed)**2
            opt_aoa *= fact
        return abs(np.clip(wind_angle - opt_aoa, -math.pi/2, math.pi/2))
    
