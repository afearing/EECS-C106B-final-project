import numpy as np
import scipy
from scipy import linalg
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math

class Controller:
    def __init__(self, sim_params, boat, env):
        self.sample_time = sim_params['simulator']['stepsize']
        
        for key, val in sim_params['boat']['controller'].items():
            exec('self.' + key + '= val')
        
        self.factor = boat.distance_cog_rudder * boat.rudder_blade_area * math.pi * env.water_density / boat.moi_z
        # initial values:
        self.summed_error = 0
        self.filtered_drift = 0


        # calculate controller parameters
        # approximated system modell, linearized, continous calculated (seems to be fine)
        A = np.array([  [0, 1, 0],
                        [0, -1/boat.yaw_timeconstant, 0],
                        [-1, 0, 0]])
        B = np.array([0, 1, 1])
        # weigth matrices for riccati design
        Q = np.diag([0.1, 1, 0.3])
        r = np.ones((1, 1))*30
        # calculating feedback
        P = scipy.linalg.solve_continuous_are(A, B[:, None], Q, r)
        K = np.sum(B[None, :] * P, axis = 1)/r[0, 0]
        self.KP = K[0]
        self.KD = K[1]
        self.KI = -K[2]

        # Desired path
        self.heading = Path(boat, env)
        self.rudder_angle = boat.rudder_angle # set initial value
        
    def controll(self, boat, env, time):
        drift_angle = boat.calculate_drift()
        speed = boat.calculate_speed()

        heading_error = self.heading.get_desired_heading(time) - boat.yaw
        # print(heading_error)
        # respect to periodicity of angle: maximum difference is 180 deg resp. pi
        while heading_error > math.pi:
            heading_error -= 2*math.pi
        while heading_error < -math.pi:
            heading_error += 2*math.pi

        # TODO: check this sample time number
        self.summed_error += self.sample_time * (heading_error -  drift_angle)
        # avoid high gains at low speed (singularity)
        if speed < self.speed_adaptation:
            speed = self.speed_adaptation

        factor2 = -1.0 / self.factor / speed**2 / math.cos(boat.roll)

        #control equation
        rudder_angle = factor2 * (self.KP * heading_error + self.KI * self.summed_error - self.KD * boat.yaw_rate)
        if abs(rudder_angle) > boat.max_rudder_angle:
            rudder_angle = np.clip(rudder_angle, -boat.max_rudder_angle, boat.max_rudder_angle)
            self.summed_error = (rudder_angle/factor2 - (self.KP * heading_error - self.KD * boat.yaw_rate)) / self.KI
        self.rudder_angle = rudder_angle
        # print(self.rudder_angle)
        return self.rudder_angle 


class Path:
    def __init__(self, boat, env):
        self.desired_heading = 0.0*math.pi # for testing
    def get_desired_heading(self, time):
        if time < 50:
            return 0
        elif time < 100:
            return np.pi/4
        else:
            return np.pi/2
        # return self.desired_heading