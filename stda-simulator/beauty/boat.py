import numpy as np
import scipy
from scipy import linalg
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math

from environment import *
from forces import *
from controller import *

class Boat:
    """The class for the boat state"""
    def __init__(self, sim_params, env):
        for key, val in sim_params['boat']['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
        for key, val in sim_params['boat']['boat_dimensions'].items():
            exec('self.' + key + '= val')
        ###
        # Invariants
        self.wave_impedance_invariant = (env.water_density / 2) * self.lateral_area
        # Hydrostatic force
        self.hydrostatic_eff_x        = self.height_bouyancy + (env.water_density / self.mass) * self.geometrical_moi_x
        self.hydrostatic_eff_y        = self.height_bouyancy + (env.water_density / self.mass) * self.geometrical_moi_y
        self.hydrostatic_invariant_z  =  -env.water_density * self.waterline_area * env.gravity
        self.gravity_force            = self.mass * env.gravity
        # Damping
        self.damping_invariant_x      = -self.mass / self.along_damping
        self.damping_invariant_y      = -self.mass / self.transverse_damping
        self.damping_invariant_z      = -.5 * self.damping_z * math.sqrt(env.water_density * self.waterline_area * env.gravity * self.mass)
        self.damping_invariant_yaw    = -(self.moi_z / self.yaw_timeconstant)
        self.damping_invariant_pitch  = -2 * self.pitch_damping * math.sqrt(self.moi_y * self.mass * env.gravity * self.hydrostatic_eff_y)
        self.damping_invariant_roll   = -2 * self.roll_damping * math.sqrt(self.moi_x * self.mass * env.gravity * self.hydrostatic_eff_x)
        # IDK
        self.distance_cog_keel_middle = self.distance_cog_keel_pressure_point - 0.7
        ### End Invariants

        self.forces = Forces(self, env)
        self.controller = Controller(sim_params, self, env)
        self.sail_angle_reference = self.calculate_sail_angle_reference()

    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)

    def calculate_drift(self):
        return np.arctan2(self.vel_y, self.vel_x)

    def calculate_forces(self, env):
        self.forces.calculate_forces(self, env)

    def calculate_state_delta(self, env, time):
        rudder_angle_reference = self.controller.controll(self, env, time)
        if not env.step_counter % int(self.sail_sampletime / self.controller.sample_time):
            self.sail_angle_reference = self.calculate_sail_angle_reference()
        sail_angle_reference = self.sail_angle_reference
        # print('rudder ang ref: ', rudder_angle_reference)


        delta_pos_x = self.vel_x * math.cos(self.yaw) - self.vel_y * math.sin(self.yaw)
        delta_pos_y = self.vel_y * math.cos(self.yaw) + self.vel_x * math.sin(self.yaw)
        delta_pos_z = self.vel_z
        delta_roll = self.roll_rate
        delta_pitch = self.pitch_rate * math.cos(self.roll) - self.yaw_rate * math.sin(self.roll)
        delta_yaw = self.yaw_rate * math.cos(self.roll) + self.pitch_rate * math.sin(self.roll)

        delta_vel_x = delta_yaw * self.vel_y + (self.forces.sail_force.x + self.forces.lateral_force.x + self.forces.rudder_force.x + self.forces.damping.x + self.forces.wave_impedance + self.forces.hydrostatic_force.x) / self.mass
        delta_vel_y = -delta_yaw * self.vel_x + ((self.forces.sail_force.y + self.forces.lateral_force.y + self.forces.rudder_force.y) * math.cos(self.roll) + self.forces.hydrostatic_force.y + self.forces.damping.y) / self.mass
        delta_vel_z = ((self.forces.sail_force.y + self.forces.lateral_force.y + self.forces.rudder_force.y) * math.sin(self.roll) + self.forces.hydrostatic_force.z - self.gravity_force + self.forces.damping.z) / self.mass

        delta_roll_rate = (self.forces.hydrostatic_force.z * self.forces.hydrostatic_force.y_hs - self.forces.sail_force.y * self.sail_pressure_point_height + self.forces.damping.roll)/self.moi_x
        delta_pitch_rate = (self.forces.sail_force.x * self.sail_pressure_point_height - self.forces.hydrostatic_force.z * self.forces.hydrostatic_force.x_hs * math.cos(self.roll) + self.forces.damping.pitch - (self.moi_x - self.moi_z) * self.roll_rate * self.yaw_rate) / self.moi_y
        delta_yaw_rate = (self.forces.damping.yaw - self.forces.rudder_force.y * self.distance_cog_rudder + self.forces.sail_force.y * self.distance_cog_sail_pressure_point + self.forces.sail_force.x * math.sin(self.forces.true_sail_angle) * self.distance_mast_sail_pressure_point + self.forces.lateral_force.y * (self.distance_cog_keel_pressure_point * (1-self.forces.lateral_force.separation) + self.distance_cog_keel_middle * self.forces.lateral_force.separation))/ self.moi_z

        delta_rudder = -2 * (self.rudder_angle - rudder_angle_reference)
        # print(self.rudder_angle, rudder_angle_reference, 'rudder ang rudder ang ref')
        delta_rudder = np.clip(delta_rudder, -self.max_rudder_speed, self.max_rudder_speed)
        # print(delta_rudder, self.max_rudder_speed, np.abs(delta_rudder) - self.max_rudder_speed)
        delta_sail = -0.1 * (self.forces.true_sail_angle - sail_angle_reference)
        delta_sail = np.clip(delta_sail, -self.max_sail_speed, self.max_sail_speed)
        delta = np.array([  delta_pos_x,     delta_pos_y,      delta_pos_z,
                            delta_roll,      delta_pitch,      delta_yaw,
                            delta_vel_x,     delta_vel_y,      delta_vel_z,
                            delta_roll_rate, delta_pitch_rate, delta_yaw_rate,
                            delta_rudder, delta_sail])
        return delta
        
    def set_state(self, boat_state, env):
        self.pos_x,          self.pos_y,          self.pos_z    = boat_state[0:3]
        self.roll,           self.pitch,          self.yaw      = boat_state[3:6]
        self.vel_x,          self.vel_y,          self.vel_z    = boat_state[6:9]
        self.roll_rate,      self.pitch_rate,     self.yaw_rate = boat_state[9:12]
        self.rudder_angle,   self.sail_angle               = boat_state[12:14]
        self.calculate_forces(env)
    
    def get_state(self):
        return np.array([   self.pos_x,          self.pos_y,          self.pos_z,   
                            self.roll,           self.pitch,          self.yaw,     
                            self.vel_x,          self.vel_y,          self.vel_z,   
                            self.roll_rate,      self.pitch_rate,     self.yaw_rate,
                            self.rudder_angle,   self.sail_angle])
                            
    def calculate_sail_angle_reference(self):
        wind_angle = self.forces.apparent_wind.apparent_angle
        wind_speed = self.forces.apparent_wind.apparent_speed
        opt_aoa = math.sin(wind_angle)/ (math.cos(wind_angle) + 0.4 * math.cos(wind_angle)**2) * self.sail_stretching / 4
        if abs(opt_aoa) > self.stall_deg/180*math.pi:
            opt_aoa = np.sign(wind_angle) * self.stall_deg/180*math.pi
        if wind_speed > self.limit_wind_speed:
            fact = (self.limit_wind_speed / wind_speed)**2
            opt_aoa *= fact
        return abs(np.clip(wind_angle - opt_aoa, -math.pi/2, math.pi/2))