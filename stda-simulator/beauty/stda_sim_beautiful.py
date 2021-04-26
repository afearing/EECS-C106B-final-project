import numpy as np
import scipy
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math
from collections import namedtuple
from simulator import *

def main():
    # simulation parameters. Set them here. We can change it to a yaml file to load later.
    save = True
    with open('sim_config.yaml', 'r') as stream: #idek what this code does but it works https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python
        sim_params = yaml.safe_load(stream)
    # run the simulation
    sim = Simulator(sim_params)
    results, env =  sim.simulate()
    plot_maneuvers(results, env, save=True)


def plot_maneuvers(results, env, save = False):
    

    t = results.t

    ref_heading = 0.5*math.pi * np.ones(t.shape) # change this later to use actual heading not hard coded in


    states = results.y
    speeds = states[6:9]
    speed_axlabels = ['Time / s', 'Speed / m/s']
    speed_labels = ['Forward', 'Leeway', 'Vertical']
    fig_speed, tmp = plot_time_fig(time=t, data=speeds, labels=speed_labels, ax_labels=speed_axlabels)


    angles = states[3:6]
    angles_labels = ['Roll', 'Pitch']#, 'Heading']
    axlabels = ['Time / s', 'Angles / degree']
    scale = 180/math.pi
    fig_ang, tmp = plot_time_fig(time=t, data=angles[0:2], labels=angles_labels, ax_labels=axlabels, scale=scale)



    positions = states[0:3]
    fig_traj, ax_traj = plot_series(positions[0], positions[1], xlabel='$x_g$ / m', ylabel='$y_g$ / m')
    plot_arrows(positions[0], positions[1], angles[2], fig=fig_traj, ax=ax_traj, color='red')
    ax_traj.plot(positions[0], positions[1], 'xb', markersize=10)
    ax_traj.arrow(0, -25, env.wind_x, env.wind_y, head_width=1.5, head_length=3, width=.5, fc='green', ec='green')


    # heading plot
    heading_data = [angles[2] + 2*math.pi, ref_heading, np.arctan2(speeds[1], speeds[0])]
    heading_labels = ['Heading', 'Reference', 'Drift']
    fig_heading, ax_heading = plot_time_fig(time=t, data=heading_data, labels=heading_labels, ax_labels=axlabels, scale=scale, ticks='ang')
    
    
    if save: # pdfs for easy viewing
        fig_heading.savefig('figs/heading.pdf')
        fig_ang.savefig('figs/angles.pdf')
        fig_speed.savefig('figs/speeds.pdf')
        fig_traj.savefig('figs/trajectories.pdf')




def plot_time_fig(time, data, labels, ax_labels, scale=1., ticks=None):
    fig, ax = None, None
    for i, data_i in enumerate(data):
        fig, ax = plot_series(time, data_i*scale, label=labels[i], xlabel=ax_labels[0], ylabel=ax_labels[1], fig=fig, ax=ax, legend=True)
        
    if ticks is not None:
        if ticks == 'ang':
            ax.set_yticks(np.arange(9)*45)
    return fig, ax

def plot_series(x, y, fig=None, ax=None, N_subplot=1, n_subplot=1, title=None, xlabel=None, ylabel=None, label=None, legend=False):
    if fig is None:
        fig = plt.figure()
    if ax is None:
        ax = fig.add_subplot(N_subplot, 1, n_subplot)
    ax.plot(x, y, label=label)
    if not xlabel is None:
        ax.set_xlabel(xlabel)
    if not ylabel is None:
        ax.set_ylabel(ylabel)
    ax.grid(True)
    
    if legend:
        ax.legend()
    return fig, ax


def plot_arrows(x, y, directions, fig=None, ax=None, color=None):
    if fig is None:
        fig = plt.figure()
    if ax is None:
        ax = fig.add_subplot(N_subplot, 1, n_subplot)
    if color is None:
        color = 'k'
    
    length = np.mean(abs(x[1:]-x[:-1]))
    #length = .2
    
    for i in range(x.shape[0]):
        ax.arrow(x[i], y[i], length*np.cos(directions[i]), length*np.sin(directions[i]), head_width=.05, head_length=.1, fc=color, ec=color)



if __name__ == '__main__':
    main()