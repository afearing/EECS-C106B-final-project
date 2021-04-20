import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math
from collections import namedtuple

# notes on implementation
# STATE = [x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_rate, pitch_rate, yaw_rate, rudder_angle, sail_angle] in R^14. All SI units, radians





class Boat:
    """The class for the boat state"""
    def __init__(self, boat_params):
        for key, val in boat_params['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)

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
        gradient_y = wave_y * facto
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
        self.apparent_angle = math.atan2(-apparent_y, -apparent_x)
        self.apparent_speed = math.sqrt(apparent_x**2 + apparent_y**2)

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
        
    def calculate_forces(self, boat):
        self.wave_influence.calculate_wave_influence(boat, self)
        self.damping.calculate_damping(boat, self)
        self.apparent_wind.calculate_apparent_wind(boat, self)



def main():
    # simulation parameters. Set them here. We can change it to a yaml file to load later.
    save = True
    with open('sim_config.yaml', 'r') as stream: #idek what this code does but it works https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python
        sim_params = yaml.safe_load(stream)
    # run the simulation
    Ts = sim_params['simulator']['stepper']['stepsize']
    boat_state = Boat(sim_params['boat']) #np.array(sim_params['initial_state'].values())
    environment_state = Environment(sim_params['environment'])
    simulate(boat_state, environment_state, save = save)



def simulate(boat_state, environment_state, save = False):
    

    return False




def solve(boat_state, boat_ref, env):

    # For force calulations needed values
    speed = boat_state.calculate_speed()
    env.calculate_forces(boat)
    # Force calculation


def calculate_wave_influence(boat, env):
    ''' Calculate how the waves influence the boat. 
    param time:     The simulation time                     [s]

    return: The influence of the waves on the boat
    '''
    frequency = math.sqrt((2 * math.pi * env.gravity) / env.wave_length)

    # wave vector components in cartesian coordinates
    wave_x = 2*math.pi / env.wave_length * math.cos(env.wave_direction)
    wave_y = 2*math.pi / env.wave_length * math.sin(env.wave_direction)

    factor = -env.wave_amplitude * math.cos(frequency * env.time - wave_x * boat.pos_x - wave_y * boat.pos_y)
    gradient_x = wave_x * factor
    gradient_y = wave_y * factor

    height = env.wave_amplitude * math.sin(frequency * env.time - wave_x * boat.pos_x - wave_y * boat.pos_y)
    gradient_x = gradient_x * math.cos(boat.yaw) + gradient_y * math.sin(boat.yaw)
    gradient_y = gradient_y * math.cos(boat.yaw) - gradient_x * math.sin(boat.yaw)
    return (height, gradient_x, gradient_y)



def calculate_apparent_wind(boat, env):
    ''' Calculate the apparent wind on the boat. 

    return: The apparent wind on the boat
    '''
    transformed_x = env.wind_x * math.cos(boat.yaw) + env.wind_y * math.sin(boat.yaw)
    transformed_y = env.wind_x * -math.sin(boat.yaw) + env.wind_y * math.cos(boat.yaw)

    apparent_x = transformed_x - boat.vel_x
    apparent_y = transformed_y - boat.vel_y
    apparent_angle = math.atan2(-apparent_y, -apparent_x)
    apparent_speed = math.sqrt(apparent_x**2 + apparent_y**2)

    return (apparent_x, apparent_y, apparent_angle, apparent_speed)

def calculate_damping(boat, env):
    ''' Calculate the damping. 
    
    return: The amount of damping applied to the boat
    '''
    return (env.damping_invariant_x *       boat.vel_x,
            env.damping_invariant_y *       boat.vel_y,
            env.damping_invariant_z *       boat.vel_z,
            env.damping_invariant_roll *    boat.roll_rate,
            env.damping_invariant_pitch *   boat.pitch_rate,
            env.damping_invariant_yaw *     boat.yaw_rate)

def calculate_hydrostatic_force(boat, env, wave_influence):
    ''' Calculate the hydrostatic force. 

    param wave_influence:   The influence of the waves on the boat     

    return: The force applied on the boat by the waves
    '''
    force = env.hydrostatic_invariant_z * (boat.pos_z - wave_influence[0]) + GRAVITY_FORCE
    
    return (force * wave_influence[1],
            force * wave_influence[2],
            force),
            (env.hydrostatic_eff_y * math.sin(boat.pitch + atan(wave_influence.gradient_x)), env.hydrostatic_eff_x * -sin(roll - atan(wave_influence.gradient_y)),)
            






if __name__ == '__main__':
    main()