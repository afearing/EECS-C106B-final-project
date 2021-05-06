import numpy as np
import matplotlib.pyplot as plt
import yaml

from simulator import *
from controller import *
from path import *

# State description
POS_X,     POS_Y,      POS_Z,             \
ROLL,      PITCH,      YAW,               \
VEL_X,     VEL_Y,      VEL_Z,             \
ROLL_RATE, PITCH_RATE, YAW_RATE = range(12)

def main():
    print('Starting program...')

    save_figs = False

    with open('sim_config.yaml', 'r') as stream: #idek what this code does but it works https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python
        sim_params = yaml.safe_load(stream)

    sim = Simulator(sim_params)

    path = Path(sim)

    controller = Controller(sim_params, sim.boat, sim.env)

    sim.set_path(path)
    sim.set_controller(controller)

    times, boat_states, rudder_ang_refs, sail_ang_refs, yaw_desireds, plan_pos = sim.simulate()

    print('true wind dir: ', sim.env.true_wind.direction)
    print('yaw: ', np.average(np.rad2deg(boat_states.T[YAW, -20:-1])))
    print('yaw - true wind dir: ', np.rad2deg(np.average(boat_states.T[YAW, -20:-1]) - sim.env.true_wind.direction))
    print('speed: ', np.sqrt(sim.boat.vel_x**2 + sim.boat.vel_y**2))

    plots_manoevers(times, boat_states.T, rudder_ang_refs, sail_ang_refs, yaw_desireds, plan_pos, wind=sim.env.true_wind)
    plt.show()
    

def plots_manoevers(t, x, r, sail, ref_heading, planned_path, save=False, sail_force=None, keel_force=None, separation=None, wind=None, append=""):
    # plots
    
    print('time: ', t[-1])

    # controls = [np.rad2deg(sail), np.rad2deg(r)]
    # control_axlabels = ['Time', 'Angle (Deg)']
    # control_lables = ['Sail Angle', 'Rudder Angle']
    # fig_control, tmp = plot_time_fig(time=t, data=controls, labels=control_lables, ax_labels=control_axlabels)

    speeds = [x[VEL_X, :], x[VEL_Y, :], x[VEL_Z, :]]
    speed_axlabels = ['Time (s)', 'Speed (m/s)']
    speed_labels = ['Forward', 'Leeway', 'Vertical']
    
    fig_speed, tmp = plot_time_fig(time=t, data=speeds, labels=speed_labels, ax_labels=speed_axlabels, title='Speed vs Time')
    
    # angles = [x[ROLL, :], x[PITCH, :]]#, x[YAW, :]]
    # angles_labels = ['Roll', 'Pitch']#, 'Heading']
    axlabels = ['Time (s)', 'Angles (deg)']
    scale = 180/np.pi
    
    # fig_ang, tmp = plot_time_fig(time=t, data=angles, labels=angles_labels, ax_labels=axlabels, scale=scale)
    
    
    fig_traj, ax_traj = plot_series(x[POS_X, :], x[POS_Y, :], xlabel='$x_g$ (m)', ylabel='$y_g$ (m)', title='Boat Position')
    # plot_arrows(x[POS_X, :], x[POS_Y, :], x[YAW, :], fig=fig_traj, ax=ax_traj, color='red')
    ax_traj.plot(planned_path[:, POS_X], planned_path[:, POS_Y])
    ax_traj.plot(x[POS_X, 0], x[POS_Y, 0], 'xb', markersize=10)
    ax_traj.arrow(0, -25, wind.x, wind.y, head_width=1.5, head_length=3, width=.5, fc='green', ec='green')
    ax_traj.legend(['actual route', 'planned route', 'start', 'wind'])
    ax_traj.set_aspect('equal', adjustable='box')
    
    # heading plot
    heading_data = [x[YAW, :], ref_heading, np.arctan2(x[VEL_Y, :], x[VEL_X, :])]
    heading_labels = ['Heading', 'Reference', 'Drift']
    fig_heading, ax_heading = plot_time_fig(time=t, data=heading_data, labels=heading_labels, ax_labels=axlabels, scale=scale, ticks='ang', title='Heading vs Time')
    
    
    if save: 
        fig_heading.savefig('figs/heading'+append+'.eps')
        fig_ang.savefig('figs/angles'+append+'.eps')
        fig_speed.savefig('figs/speeds'+append+'.eps')
        fig_traj.savefig('figs/trajectories'+append+'.eps')

def plot_time_fig(time, data, labels, ax_labels, scale=1., ticks=None, title=None):
    fig, ax = None, None
    for i, data_i in enumerate(data):
        fig, ax = plot_series(time, data_i*scale, label=labels[i], xlabel=ax_labels[0], ylabel=ax_labels[1], fig=fig, ax=ax, legend=True, title=title)
        
    if ticks is not None:
        if ticks == 'ang':
            ax.set_yticks(np.arange(9)*45)
    return fig, ax

def plot_arrows(x, y, directions, fig=None, ax=None, color=None):
    if fig is None:
        fig = plt.figure()
    if ax is None:
        ax = fig.add_subplot(N_subplot, 1, n_subplot)
    if color is None:
        color = 'k'
    
    length = np.mean(abs(x[1:]-x[:-1]))
    
    for i in range(x.shape[0]):
        ax.arrow(x[i], y[i], length*np.cos(directions[i]), length*np.sin(directions[i]), head_width=.05, head_length=.1, fc=color, ec=color)
    
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
    if title is not None:
        ax.set_title(title)
    ax.grid(True)
    
    if legend:
        ax.legend()
    return fig, ax

if __name__ == '__main__':
    main()