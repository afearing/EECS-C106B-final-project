import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math


class WaveInfluence:
    def __init__(self, boat, env):
        self.calculate_wave_influence(boat, env)

    def calculate_wave_influence(self, boat, env):
        ''' Calculate how the waves influence the boat. 
        param time:     The simulation time [s]
        return: The influence of the waves on the boat
        '''
        frequency = math.sqrt((2 * math.pi * env.gravity)) / env.wave_length

        # wave vector components in cartesian coordinates
        wave_x = 2*math.pi / env.wave_length * math.cos(env.wave_direction)
        wave_y = 2*math.pi / env.wave_length * math.sin(env.wave_direction)
        
        factor = -env.wave_amplitude * math.cos(frequency * env.time - wave_x * boat.pos_x - wave_y * boat.pos_y)
        gradient_x = wave_x * factor
        gradient_y = wave_y * factor
        self.height = env.wave_amplitude * math.sin(frequency * env.time - wave_x * boat.pos_x - wave_y * boat.pos_y)
        self.gradient_x = gradient_x * math.cos(boat.yaw) + gradient_y * math.sin(boat.yaw)
        self.gradient_y = gradient_y * math.cos(boat.yaw) - gradient_x * math.sin(boat.yaw)
# 
        return self.height, self.gradient_x, self.gradient_y


class ApparentWind:
    def __init__(self, boat, env): # might change these to initialize in a sensible way
        self.calculate_apparent_wind(boat, env)

    def calculate_apparent_wind(self, boat, env):
        ''' Calculate the apparent wind on the boat. 

        return: The apparent wind on the boat
        '''
        transformed_x = env.wind_x * math.cos(boat.yaw) + env.wind_y * math.sin(boat.yaw)
        transformed_y = env.wind_x * -math.sin(boat.yaw) + env.wind_y * math.cos(boat.yaw)

        self.apparent_x = transformed_x - boat.vel_x
        self.apparent_y = transformed_y - boat.vel_y
        self.apparent_angle = math.atan2(-self.apparent_y, -self.apparent_x)
        self.apparent_speed = math.sqrt(self.apparent_x**2 + self.apparent_y**2)

        return (self.apparent_x, self.apparent_y, self.apparent_angle, self.apparent_speed)



class Damping:
    def __init__(self, boat, env):
        self.calculate_damping(boat, env)
    def calculate_damping(self, boat, env):
        self.x      = boat.damping_invariant_x * boat.vel_x
        self.y      = boat.damping_invariant_y *       boat.vel_y
        self.z      = boat.damping_invariant_z *       boat.vel_z
        self.roll   = boat.damping_invariant_roll *    boat.roll_rate
        self.pitch  = boat.damping_invariant_pitch *   boat.pitch_rate
        self.yaw    = boat.damping_invariant_yaw *     boat.yaw_rate
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


class HydrostaticForce:
    def __init__(self, boat, env):
        self.calculate_hydrostatic_force(boat, env)
    def calculate_hydrostatic_force(self, boat, env):
        force = boat.hydrostatic_invariant_z * (boat.pos_z - boat.wave_influence.height) + boat.gravity_force
        self.x = force * boat.wave_influence.gradient_x
        self.y = force * boat.wave_influence.gradient_y
        self.z = force
        self.x_hs = boat.hydrostatic_eff_y * math.sin(boat.pitch + math.atan(boat.wave_influence.gradient_x))
        self.y_hs = boat.hydrostatic_eff_x * -math.sin(boat.roll - math.atan(boat.wave_influence.gradient_y))
        return (self.x, self.y, self.z, self.x_hs, self.y_hs)





class RudderForce:
    def __init__():
        self.x = None
        self.y = None
    def calculate_rudder_force(self, boat, env):
        pressure = (env.water_density / 2) * boat.calculate_speed()**2
        self.x = -(((4 * math.pi) / boat.rudder_stretching) * boat.rudder_angle**2) * pressure * env.rudder_blade_area
        self.y = 2 * math.pi * pressure * boat.rudder_blade_area * boat.rudder_angle



class Boat:
    """The class for the boat state"""
    def __init__(self, boat_params, env):
        for key, val in boat_params['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
        for key, val in boat_params['boat_dimensions'].items():
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
        ###
        

        self.wave_influence = WaveInfluence(self, env)
        self.apparent_wind = ApparentWind(self, env)
        self.damping = Damping(self, env)
        self.hydrostatic_force = HydrostaticForce(self, env)
        self.wave_impedance = self.calculate_wave_impedance(env)
        self.true_sail_angle = self.calculate_true_sail_angle(env)

    def calculate_forces(self, env):
        self.wave_influence.calculate_wave_influence(self, env)
        self.apparent_wind.calculate_apparent_wind(self, env)
        self.damping.calculate_damping(self, env)
        self.hydrostatic_force.calculate_hydrostatic_force(self, env)
        self.wave_impedance = self.calculate_wave_impedance(env)
        self.true_sail_angle = self.calculate_true_sail_angle(env)
        
    def calculate_wave_impedance(self, env):
        return -np.sign(self.vel_x) * self.calculate_speed()**2 * (self.calculate_speed()/self.hull_speed)**2 * self.wave_impedance_invariant

    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)

    def calculate_true_sail_angle(self, env):
        return np.sign(self.apparent_wind.apparent_angle) * abs(self.sail_angle)