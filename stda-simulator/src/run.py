
from heading_controller import heading_controller
from simulation import *
from numpy import *
import matplotlib.pyplot as plt
from sail_angle import sail_angle
from time import clock

deq = solve

def main():
    
    save=True
    
    scenario_1(save=save)
    
    plt.show()

def deq_sparse(time, state):
    # ignore pitch and heave oscillations (not really needed)
    diff = deq(time, state)
    diff[VEL_Z] = 0
    diff[PITCH_RATE] = 0
    diff[ROLL_RATE] = 0
    return diff
    
def init_data_arrays(n_states, N_steps, x0):
    x = zeros((n_states, N_steps+1))
    r = zeros(N_steps+1)
    sail = zeros(N_steps+1)
    t = zeros(N_steps+1)    
    x [:, 0] = x0
    
    ref_heading = zeros(N_steps+1)
    return x, t, r, sail, ref_heading
        
def init_integrator(x0, sampletime, sparse=False):
    # init integrator
    fun = deq if not sparse else deq_sparse
    integrator = ode(fun).set_integrator('dopri5')    
    integrator.set_initial_value(x0, 0)
    # init heading controller
    controller = heading_controller(sampletime, max_rudder_angle=15*pi/180)
    controller.calculate_controller_params(YAW_TIMECONSTANT)
    
    return integrator, controller

def scenario_1(save=False):
    n_states = 14
    wind = TrueWind(0, 5, 5, pi/2)
    environment[SAIL_ANGLE] = 0. /180 * pi
    environment[RUDDER_ANGLE] = 0
    environment[WAVE] = Wave(50, 0, 0)
    sim_wave = False
    if sim_wave:
        environment[WAVE] = Wave(length=100., amplitude=.5, direction=0)
        append = "_wave"
    else:
        append = ""
    environment[TRUE_WIND] = wind
    
    # simulation params
    t_end = 150.
    sampletime = .3
    sail_sampletime = 2.
    N_steps = int(t_end / sampletime)
    # initial values
    x0 = zeros(n_states)
    x0[VEL_X] = 0.
    x0[SAIL_STATE] = 48*pi/180
    
    x, t, r, sail, ref_heading = init_data_arrays(n_states, N_steps, x0)
    
    integrator, controller = init_integrator(x0, sampletime)
    
    ref_heading[int(40/sampletime):] = 1.2*pi

    ref_heading[int(90/sampletime) :] = .35 * pi

    sail_angle = None
    
    x, t, separation, keel, sail_force, sail, r = simulate(N_steps, x, t, r, sail, environment, controller, 
                                                            integrator, sampletime, sail_sampletime, ref_heading, wind, sail_angle)
    
    
    plots_manoevers(t, x, r, sail, ref_heading, save, sail_force=sail_force, keel_force=keel, separation=separation, wind=wind, append=append)


def smooth_reference(ref_heading, n_filter=5):
    ################ potential smoothing of heading reference
    smoothed = ref_heading.copy()
    N=ref_heading.shape[0]
    for i in range(N):
        ind_low = max(0, i-n_filter/2)
        ind_high = min(N, i+(n_filter-n_filter/2))
        smoothed[i] = np.mean(ref_heading[ind_low:ind_high])
        
    return smoothed

def comp_route(x, x2):
    fig_traj, ax_traj = plot_series(x[POS_X, :], x[POS_Y, :], xlabel='$x$ / m', ylabel='$y$ / m')
    fig_traj, ax_traj = plot_series(x2[POS_X, :], x2[POS_Y, :], xlabel='$x$ / m', ylabel='$y$ / m', fig=fig_traj, ax=ax_traj)
    

def plots_manoevers(t, x, r, sail, ref_heading, save, sail_force=None, keel_force=None, separation=None, wind=None, append=""):
    # plots
    
    speeds = [x[VEL_X, :], x[VEL_Y, :], x[VEL_Z, :]]
    speed_axlabels = ['Time / s', 'Speed / m/s']
    speed_labels = ['Forward', 'Leeway', 'Vertical']
    
    fig_speed, tmp = plot_time_fig(time=t, data=speeds, labels=speed_labels, ax_labels=speed_axlabels)
    
    angles = [x[ROLL, :], x[PITCH, :]]#, x[YAW, :]]
    angles_labels = ['Roll', 'Pitch']#, 'Heading']
    axlabels = ['Time / s', 'Angles / degree']
    scale = 180/pi
    
    fig_ang, tmp = plot_time_fig(time=t, data=angles, labels=angles_labels, ax_labels=axlabels, scale=scale)
    
    
    fig_traj, ax_traj = plot_series(x[POS_X, :], x[POS_Y, :], xlabel='$x_g$ / m', ylabel='$y_g$ / m')
    plot_arrows(x[POS_X, :], x[POS_Y, :], x[YAW, :], fig=fig_traj, ax=ax_traj, color='red')
    ax_traj.plot(x[POS_X, 0], x[POS_Y, 0], 'xb', markersize=10)
    ax_traj.arrow(0, -25, wind.x, wind.y, head_width=1.5, head_length=3, width=.5, fc='green', ec='green')
    #ax_traj.legend(['route', 'start', 'wind'])
    
    # heading plot
    heading_data = [x[YAW, :] + 2*pi, ref_heading, arctan2(x[VEL_Y, :], x[VEL_X, :])]
    heading_labels = ['Heading', 'Reference', 'Drift']
    fig_heading, ax_heading = plot_time_fig(time=t, data=heading_data, labels=heading_labels, ax_labels=axlabels, scale=scale, ticks='ang')
    
    
    if save: 
        fig_heading.savefig('figs/heading'+append+'.eps')
        fig_ang.savefig('figs/angles'+append+'.eps')
        fig_speed.savefig('figs/speeds'+append+'.eps')
        fig_traj.savefig('figs/trajectories'+append+'.eps')
        
    

def plot_time_fig(time, data, labels, ax_labels, scale=1., ticks=None):
    fig, ax = None, None
    for i, data_i in enumerate(data):
        fig, ax = plot_series(time, data_i*scale, label=labels[i], xlabel=ax_labels[0], ylabel=ax_labels[1], fig=fig, ax=ax, legend=True)
        
    if ticks is not None:
        if ticks == 'ang':
            ax.set_yticks(np.arange(9)*45)
    return fig, ax

def plot_leeways(t, separation, vx, vy):
    ax, fig = None, None
    fig, ax = plot_series(t, vx, xlabel='Time /s', label='Forward speed in $m/s$', ax=ax, fig=fig)
    fig, ax = plot_series(t, vy, xlabel='Time /s', label='Leeway in $m/s$', ax=ax, fig=fig)
    fig, ax = plot_series(t, separation, xlabel='Time /s', label='Flow separation factor', ax=ax, fig=fig)
    
    #ax.grid(True)
    ax.legend()
    ax.set_xlim([0, 3])
    ax.set_ylim([0, 1])
    return fig, ax
    
def simulate(N_steps, x, t, r, sail, environment, controller, integrator, sampletime, sail_sampletime, ref_heading, wind=None, predefined_sail_angle=None):
    t1 = clock()
    for i in range(N_steps):
        
        # rudder_angle calculation
        speed = sqrt(x[VEL_X, i]**2 + x[VEL_Y, i]**2) 
        drift = arctan2(x[VEL_Y, i], x[VEL_X, i])
        environment[RUDDER_ANGLE] = controller.controll(ref_heading[i], x[YAW, i], x[YAW_RATE, i], speed, x[ROLL, i], drift_angle=drift)
        #if t[i] < 80:
        r[i] = environment[RUDDER_ANGLE]
        #else:
            #r[i] = controller2.controll(ref_heading[i], x[YAW, i], x[YAW_RATE, i], speed, x[ROLL, i])#-.3 /180 *pi
            #environment[RUDDER_ANGLE] = r[i]
            
        if not predefined_sail_angle is None:
            sail[i] = predefined_sail_angle[i]
            environment[SAIL_ANGLE] = sail[i]
        else:
            if not i%int(sail_sampletime/sampletime):
                apparent_wind = calculate_apparent_wind(x[YAW, i], x[VEL_X, i], x[VEL_Y, i], wind)        
                environment[SAIL_ANGLE] = sail_angle(apparent_wind.angle, apparent_wind.speed, SAIL_STRETCHING)
                
                #print apparent_wind.angle/pi*180, wind.direction/pi*180, x[VEL_X, i], x[VEL_Y, i], wind.strength, apparent_wind.speed, x[YAW, i]/pi*180
                
                sail[i] = environment[SAIL_ANGLE]    
                
            else:
                sail[i] = sail[i-1]
            
    

        integrator.integrate(integrator.t+sampletime)
        t[i+1] = integrator.t
        x[:, i+1] = integrator.y
        #print 'time', t[i]
    print 'computation time', clock()-t1
    # forces
    sail_force = np.zeros((2, x.shape[1]))
    keel = np.zeros((2, x.shape[1]))
    separation = np.zeros((x.shape[1]))
    for i in range(x.shape[1]):
        apparent_wind = calculate_apparent_wind(x[YAW, i], x[VEL_X, i], x[VEL_Y, i], wind)
        force = calculate_sail_force(x[ROLL, i], apparent_wind, np.sign(apparent_wind.angle)*sail[i])
        sail_force[:, i] = [force.x, force.y]
        speed = np.sqrt(x[VEL_X, i]**2+ x[VEL_Y, i]**2)
        lateral_force, lateral_separation     = calculate_lateral_force(x[VEL_X, i], x[VEL_Y, i], x[YAW, i], speed)
        keel[:, i] = [lateral_force.x, lateral_force.y]
        
        separation[i] = lateral_separation
        
    return x, t, separation, keel, sail_force, sail, r

def plot_arrows(x, y, directions, fig=None, ax=None, color=None):
    if fig is None:
        fig = plt.figure()
    if ax is None:
        ax = fig.add_subplot(N_subplot, 1, n_subplot)
    if color is None:
        color = 'k'
    
    length = mean(abs(x[1:]-x[:-1]))
    
    for i in range(x.shape[0]):
        ax.arrow(x[i], y[i], length*cos(directions[i]), length*sin(directions[i]), head_width=.05, head_length=.1, fc=color, ec=color)
    
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
    
if __name__ == "__main__":
    main()
