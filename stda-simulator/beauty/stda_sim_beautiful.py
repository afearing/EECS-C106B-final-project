import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math
from collections import namedtuple
from environment import *
from boat import *

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
    
    solve(boat_state, 0, environment_state)
    return False




def solve(boat_state, boat_ref, env):

    # For force calulations needed values
    speed = boat_state.calculate_speed()
    env.calculate_forces(boat_state)
    # Force calculation




if __name__ == '__main__':
    main()