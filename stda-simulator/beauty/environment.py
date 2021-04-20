import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math



class WaveInfluence:
    def __init__(self):
        self.height = None
        self.gradient_x = None
        self.gradient_y = None

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
    def __init__(self): # might change these to initialize in a sensible way
        self.apparent_x      = None
        self.apparent_y      = None
        self.apparent_angle  = None
        self.apparent_speed  = None

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
    def __init__(self):
        self.x      =     None
        self.y      =     None
        self.z      =     None
        self.roll   =     None
        self.pitch  =     None
        self.yaw    =     None
    def calculate_damping(self, boat, env):
        self.x      = env.damping_invariant_x * boat.vel_x
        self.y      = env.damping_invariant_y *       boat.vel_y
        self.z      = env.damping_invariant_z *       boat.vel_z
        self.roll   = env.damping_invariant_roll *    boat.roll_rate
        self.pitch  = env.damping_invariant_pitch *   boat.pitch_rate
        self.yaw    = env.damping_invariant_yaw *     boat.yaw_rate
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

class RudderForce:
    def __init__():
        self.x = None
        self.y = None
    def calculate_rudder_force(self, boat, env):
        pressure = (env.water_density / 2) * boat.calculate_speed()**2
        self.x = -(((4 * math.pi) / env.rudder_stretching) * boat.rudder_angle**2) * pressure * env.rudder_blade_area
        self.y = 2 * math.pi * pressure * env.rudder_blade_area * boat.rudder_angle
class HydrostaticForce:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.x_hs = None
        self.y_hs = None
    def calculate_hydrostatic_force(self, boat, env):
        force = env.hydrostatic_invariant_z * (boat.pos_z - env.wave_influence.height) + env.gravity_force
        self.x = force * env.wave_influence.gradient_x
        self.y = force * env.wave_influence.gradient_y
        self.z = force
        self.x_hs = env.hydrostatic_eff_y * math.sin(boat.pitch + math.atan(env.wave_influence.gradient_x))
        self.y_hs = env.hydrostatic_eff_x * -math.sin(boat.roll - math.atan(env.wave_influence.gradient_y))
        return self.x, self.y, self.z, self.x_hs, self.y_hs, self



class Environment:
    """The class for the environment and its properties. Note that this includes the boat dimensions"""
    def __init__(self, env_params):
        for key, val in env_params['properties'].items():
            exec('self.' + key + '= val')
        for key, val in env_params['disturbances'].items():
            exec('self.' + key + '= val')
        for key, val in env_params['boat_dimensions'].items():
            exec('self.' + key + '= val')

        ###
        # Invariants
        self.wave_impedance_invariant = (self.water_density / 2) * self.lateral_area
        # Hydrostatic force
        self.hydrostatic_eff_x        = self.height_bouyancy + (self.water_density / self.mass) * self.geometrical_moi_x
        self.hydrostatic_eff_y        = self.height_bouyancy + (self.water_density / self.mass) * self.geometrical_moi_y
        self.hydrostatic_invariant_z  =  -self.water_density * self.waterline_area * self.gravity
        self.gravity_force            = self.mass * self.gravity
        # Damping
        self.damping_invariant_x      = -self.mass / self.along_damping
        self.damping_invariant_y      = -self.mass / self.transverse_damping
        self.damping_invariant_z      = -.5 * self.damping_z * math.sqrt(self.water_density * self.waterline_area * self.gravity * self.mass)
        self.damping_invariant_yaw    = -(self.moi_z / self.yaw_timeconstant)
        self.damping_invariant_pitch  = -2 * self.pitch_damping * math.sqrt(self.moi_y * self.mass * self.gravity * self.hydrostatic_eff_y)
        self.damping_invariant_roll   = -2 * self.roll_damping * math.sqrt(self.moi_x * self.mass * self.gravity * self.hydrostatic_eff_x)
        ###
        self.time = 0
        self.wind_x = self.wind_strength * math.cos(self.wind_direction)
        self.wind_y = self.wind_strength * math.sin(self.wind_direction)

        self.wave_influence = WaveInfluence()
        self.apparent_wind = ApparentWind()
        self.damping = Damping()
        self.hydrostatic_force = HydrostaticForce()
        self.wave_impedance = None


    def calculate_forces(self, boat):
        self.wave_influence.calculate_wave_influence(boat, self)
        self.damping.calculate_damping(boat, self)
        self.apparent_wind.calculate_apparent_wind(boat, self)
        self.hydrostatic_force.calculate_hydrostatic_force(boat, self)
        self.wave_impedance = -np.sign(boat.vel_x) * boat.calculate_speed()**2 * (boat.calculate_speed()/self.hull_speed)**2 * self.wave_impedance_invariant