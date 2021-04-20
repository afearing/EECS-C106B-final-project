import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math


# notes on implementation
# STATE = [x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_rate, pitch_rate, yaw_rate, rudder_angle, sail_angle] in R^14. All SI units, radians





class Boat:
    """The class for the boat state"""
    def __init__(self, boat_params):
        for key, val in boat_params['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
        for key, val in boat_params['dimensions'].items():
            exec('self.' + key + '= val')


    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)


class Environment:
    def __init__(self, env_params):
        for key, val in env_params['properties'].items():
            exec('self.' + key + '= val')
        for key, val in env_params['disturbances'].items():
            exec('self.' + key + '= val')

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




def solve(time, boat_state, boat_ref, environment_parameters):

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
    pos_x = boat_state[0]
    pos_y = boat_state[1]
    yaw   = boat_state[5]
    # wave vector components in cartesian coordinates
    wave_x = 2*math.pi / wave_length * math.cos(wave_direction)
    wave_y = 2*math.pi / wave_length * math.sin(wave_direction)
    factor = -wave_amplitude * math.cos(frequency * time - wave_x * pos_x - wave_y * pos_y)
    gradient_x = wave_x * factor
    gradient_y = wave_y * factor

    height = wave_amplitude * math.sin(frequency * time - wave_x * pos_x - wave_y * pos_y)
    gradient_x = gradient_x * math.cos(yaw) + gradient_y * math.sin(yaw)
    gradient_y = gradient_y * math.cos(yaw) - gradient_x * math.sin(yaw)
    return (height, gradient_x, gradient_y)

def calculate_apparent_wind(boat_state, environment_parameters):
    ''' Calculate the apparent wind on the boat. 
    param true_wind:    The true wind directions

    return: The apparent wind on the boat
    '''
    true_wind
    transformed_x = true_wind.x * cos(yaw) + true_wind.y * sin(yaw)
    transformed_y = true_wind.x * -sin(yaw) + true_wind.y * cos(yaw)

    apparent_x = transformed_x - vel_x
    apparent_y = transformed_y - vel_y
    apparent_angle = atan2(-apparent_y, -apparent_x)
    apparent_speed = sqrt(apparent_x**2 + apparent_y**2)

    return ApparentWind(x=apparent_x,
                        y=apparent_y,
                        angle=apparent_angle,
                        speed=apparent_speed)



if __name__ == '__main__':
    main()