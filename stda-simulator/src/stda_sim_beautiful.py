import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io






def main():
    # simulation parameters. Set them here. We can change it to a yaml file to load later.
    save = True
    with open('sim_params_config.yaml', 'r') as stream: #idek what this code does but it works https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python
        sim_params = yaml.safe_load(stream)
    # run the simulation
    print(sim_params)






if __name__ == '__main__':
    main()