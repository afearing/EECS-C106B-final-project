import scipy
import scipy.integrate

import numpy as np
from environment import *
from boat import *
from path import *
import forces

class Simulator:
    def __init__(self, sim_params, controller = None, path = None):
        for key, val in sim_params['simulator'].items():
            exec('self.' + key + '= val')
        
        self.controller = controller
        self.path = path
        self.env = Environment(sim_params['environment'])
        self.boat = Boat(sim_params, self.env)

    def set_controller(self, controller):
        self.controller = controller
    
    def set_path(self, path):
        self.path = path
        
    def simulate(self):
        num_steps = int(np.ceil(self.t_end / self.stepsize))
        print('num steps: ', num_steps)

        times = np.zeros((num_steps + 1, 1))
        boat_states = np.zeros((num_steps + 1, 14))
        boat_states[0] = self.boat.get_state()
        rudder_ang_refs = np.zeros((num_steps + 1, 1))
        sail_ang_refs = np.zeros((num_steps + 1, 1))
        yaw_desireds = np.zeros((num_steps + 1, 1))


        for step_i in range(num_steps):
            print('iteration: ', step_i)
            time = step_i * self.stepsize
            
            # Compute rudder control input
            if self.controller is not None:
                yaw_desired = self.path.yaw(time)
                yaw_desireds[step_i] = yaw_desired

                self.rudder_ang_ref = self.controller.rudder_control(time, yaw_desired, self.boat, self.env)

                # Compute sail control input
                if not step_i % int(self.boat.sail_sampletime / self.stepsize):
                    self.sail_angle_ref = self.controller.sail_control(time, self.boat, self.env)
            else:
                self.rudder_ang_ref = 0
                self.sail_angle_ref = 0
            
            # Integrate to next time step
            sol = scipy.integrate.solve_ivp(self.__solve, (time, time+self.stepsize), self.boat.get_state())
            times[step_i + 1] = sol.t[-1]
            boat_states[step_i + 1] = sol.y[:, -1]
            rudder_ang_refs[step_i + 1] = self.rudder_ang_ref
            sail_ang_refs[step_i + 1] = self.sail_angle_ref

            self.boat.set_state(boat_states[step_i + 1])
            if self.boat.yaw < -np.pi:
                self.boat.yaw += 2*np.pi
            elif self.boat.yaw > np.pi:
                self.boat.yaw -= 2*np.pi

        return times, boat_states, rudder_ang_refs, sail_ang_refs, yaw_desireds

    def __solve(self, time, boat_state):
        self.boat.set_state(boat_state)

        speed = np.sqrt(self.boat.vel_x**2 + self.boat.vel_y**2)
        wave_influence = forces.calculate_wave_influence(self.boat.pos_x, self.boat.pos_y,
            self.boat.yaw, self.env.wave, time, self.env.gravity)
        apparent_wind = forces.calculate_apparent_wind(self.boat.yaw,
            self.boat.vel_x, self.boat.vel_y, self.env.true_wind)
        
        true_sail_angle = np.sign(apparent_wind.angle) * np.abs(self.boat.sail_angle)

        damping = forces.calculate_damping(self.boat.vel_x, self.boat.vel_y, self.boat.vel_z,
            self.boat.roll_rate, self.boat.pitch_rate, self.boat.yaw_rate, self.boat)
        hydrostatic_force, x_hs, y_hs = forces.calculate_hydrostatic_force(self.boat.pos_z,
            self.boat.roll, self.boat.pitch, wave_influence, self.boat)
        
        wave_impedance = forces.calculate_wave_impedance(self.boat.vel_x, speed, self.boat)
        rudder_force = forces.calculate_rudder_force(speed, self.boat.rudder_angle, self.boat, self.env)
        lateral_force, lateral_separation = forces.calculate_lateral_force(self.boat.vel_x,
            self.boat.vel_y, self.boat.roll, speed, self.boat, self.env)
        sail_force = forces.calculate_sail_force(self.boat.roll, apparent_wind, true_sail_angle, self.boat, self.env)

        dpos_x = self.boat.vel_x * np.cos(self.boat.yaw) - self.boat.vel_y * np.sin(self.boat.yaw)
        dpos_y = self.boat.vel_y * np.cos(self.boat.yaw) + self.boat.vel_x * np.sin(self.boat.yaw)
        dpos_z = self.boat.vel_z
        droll  = self.boat.roll_rate
        dpitch = self.boat.pitch_rate * np.cos(self.boat.roll) - self.boat.yaw_rate * np.sin(self.boat.roll)
        dyaw   = self.boat.yaw_rate * np.cos(self.boat.roll) + self.boat.pitch_rate * np.sin(self.boat.roll)

        dvel_x = dyaw * self.boat.vel_y + (sail_force.x + lateral_force.x + rudder_force.x + damping.x + wave_impedance + hydrostatic_force.x) / self.boat.mass
     
        dvel_y = -dyaw * self.boat.vel_x + \
                    ((sail_force.y + lateral_force.y + rudder_force.y) * np.cos(self.boat.roll) + \
                    hydrostatic_force.y + \
                    damping.y) / self.boat.mass

        dvel_z = ((sail_force.y + lateral_force.y + rudder_force.y) * np.sin(self.boat.roll) + \
                    hydrostatic_force.z - self.boat.gravity_force + damping.z) / self.boat.mass

                    

        droll_rate  = (hydrostatic_force.z * y_hs
                            - sail_force.y * self.boat.sail_pressure_point_height
                            + damping.roll) / self.boat.moi_x

        dpitch_rate = (sail_force.x * self.boat.sail_pressure_point_height
                            - hydrostatic_force.z * x_hs * np.cos(self.boat.roll)
                            + damping.pitch
                            - (self.boat.moi_x - self.boat.moi_z) * self.boat.roll_rate * self.boat.yaw_rate) / self.boat.moi_y
        
        dyaw_rate   = (damping.yaw
                            #+ hydrostatic_force.z * hydrostatic_force.x * sin(roll)
                            - rudder_force.y * self.boat.distance_cog_rudder
                            + sail_force.y * self.boat.distance_cog_sail_pressure_point
                            + sail_force.x * np.sin(true_sail_angle) * self.boat.distance_mast_sail_pressure_point
                            + lateral_force.y * (self.boat.distance_cog_keel_pressure_point * (1-lateral_separation)\
                                                + self.boat.distance_cog_keel_middle * lateral_separation))/ self.boat.moi_z
    

        drudder = - 2 * (self.boat.rudder_angle - self.rudder_ang_ref)
        drudder = np.clip(drudder, -self.boat.max_rudder_speed, self.boat.max_rudder_speed)
        
        dsail = - .1 * (self.boat.sail_angle - self.sail_angle_ref)
        dsail = np.clip(dsail, -self.boat.max_sail_speed, self.boat.max_sail_speed)

        diff = np.array([
            dpos_x, dpos_y, dpos_z,
            droll, dpitch, dyaw,
            dvel_x, dvel_y, dvel_z,
            droll_rate, dpitch_rate, dyaw_rate,
            drudder, dsail
        ])

        return diff