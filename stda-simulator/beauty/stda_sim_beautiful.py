import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math

"""
notes on implementation
STATE = [x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_rate, pitch_rate, yaw_rate, rudder_angle, sail_angle] in R^14. All SI units, radians



"""

def main():
    # simulation parameters. Set them here. We can change it to a yaml file to load later.
    save = True
    with open('sim_config.yaml', 'r') as stream: #idek what this code does but it works https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python
        sim_params = yaml.safe_load(stream)
    # run the simulation
    print(sim_params, save = save)
    simulate(sim_params, save = save)



def simulate(sim_params, save = False):
    Ts = sim_params['simulator']['stepper']['stepsize']
    boat_state = np.array(sim_params['initial_state'].values())
    environment_parameters = sim_params['environment']


    return False




def solve(time, boat_state, boat_ref, environment_parameters):
    # State unpacking
    pos_x,     pos_y,      pos_z    = boat_state[0        : 3]
    roll,      pitch,      yaw      = boat_state[3        : 6]
    vel_x,     vel_y,      vel_z    = boat_state[6        : 9]
    roll_rate, pitch_rate, yaw_rate = boat_state[9        : 12]
    rudder_angle, sail_angle        = boat_state[12       : 15]
    speed = math.sqrt(vel_x**2 + vel_y**2)

    # For force calulations needed values
    speed = calculate_speed(boat_state)
    wave_influence = calculate_wave_influence(time, boat_state, environment_parameters)
    apparent_wind = calculate_apparent_wind(boat_state, environment_parameters)





def calculate_speed(boat_state):
    return np.linalg.norm(boat_state[3:5]) # sqrt(sumsq(x-y velocity))



def calculate_wave_influence(time, boat_state, environment_parameters):
    ''' Calculate how the waves influence the boat. 
    param time:     The simulation time                     [s]

    return: The influence of the waves on the boat
    '''
    g =                 environment_parameters['environment']['gravity']
    wave_length =       environment_parameters['disturbances']['wave_length']
    wave_direction =    environment_parameters['disturbances']['wave_direction']
    wave_amplitude =    environment_parameters['disturbances']['wave_amplitude']
    frequency = math.sqrt((2 * math.pi * g) / wave_length)

    # wave vector components in cartesian coordinates
    wave_x = 2*math.pi / wave_length * math.cos(wave_direction)
    wave_y = 2*math.pi / wave_length * math.sin(wave_direction)
    factor = -wave_amplitude * math.cos(frequency * time - wave_x * boat_state[0] = wave_y * boat_state[1])
    gradient_x = wave_x * factor
    gradient_y = wave_y * factor

    height = wave_amplitude * math.sin(frequency * time - wave_x * boat_state[0] = wave_y * boat_state[1])
    gradient_x = gradient_x * math.cos(boat_state[2]) + gradient_y * math.sin(boat_state[2])
    gradient_y = gradient_y * math.cos(boat_state[2]) - gradient_x * math.sin(boat_state[2])
    return (height, gradient_x, gradient_y)



if __name__ == '__main__':
    main()