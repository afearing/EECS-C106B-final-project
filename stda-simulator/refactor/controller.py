from scipy.linalg import solve_continuous_are
import numpy as np

from forces import calculate_apparent_wind

class Controller:
    def __init__(self, sim_params, boat, env):
        self.sample_time = sim_params['simulator']['stepsize']
        
        for key, val in sim_params['boat']['controller'].items():
            exec('self.' + key + '= val')

        self.summed_error = 0

        self.factor = boat.distance_cog_rudder * boat.rudder_blade_area * np.pi * env.water_density / boat.moi_z

        A = A=np.array([[0, 1,        0],\
                        [0, -1./boat.yaw_timeconstant, 0],\
                        [-1, 0,       0]])
        B = np.array([0, 1, 1])
        
        # weigth matrices for riccati design
        Q = np.diag([1E-1, 1, 0.3])
        r = np.ones((1,1))*30
        
        # calculating feedback
        P = solve_continuous_are(A,B[:, None],Q,r)
        K = np.sum(B[None, :] * P, axis=1)/r[0, 0]
        
        self.KP = K[0]
        self.KD = K[1]
        self.KI = -K[2]

    def rudder_control(self, time, yaw_desired, boat, env):
        # calculate error, difference and sum of error
        heading_error = yaw_desired - boat.yaw
        speed = np.sqrt(boat.vel_x**2 + boat.vel_y**2)
        drift_angle = np.arctan2(boat.vel_y, boat.vel_x)
        
        #print 'heading_error', heading_error/pi*180
        # respect to periodicity of angle: maximum difference is 180 deg resp. pi
        while heading_error > np.pi:
            heading_error -= 2*np.pi
        while heading_error < -np.pi:
            heading_error += 2*np.pi   
            
        self.summed_error += self.sample_time * (heading_error - drift_angle)
        
        # avoid high gains at low speed (singularity)
        if speed < self.speed_adaptation:
            speed = self.speed_adaptation
        
        factor2 = -1. / self.factor / speed**2 / np.cos(boat.roll) 
        
        # control equation
        rudder_angle = factor2 * (self.KP * heading_error + self.KI * self.summed_error - self.KD * boat.yaw_rate)
        
        # rudder restriction and anti-windup
        if np.abs(rudder_angle) > boat.max_rudder_angle:
            rudder_angle = np.sign(rudder_angle) * boat.max_rudder_angle
            self.summed_error = (rudder_angle/factor2 - (self.KP * heading_error - self.KD * boat.yaw_rate)) / self.KI
        
        return rudder_angle

    def sail_control(self, time, boat, env):
        apparent_wind = calculate_apparent_wind(boat.yaw, boat.vel_x, boat.vel_y, env.true_wind)
        wind_angle = apparent_wind.angle
        wind_speed = apparent_wind.speed
        opt_aoa = np.sin(wind_angle) / (np.cos(wind_angle) + .4 * np.cos(wind_angle)**2) * boat.sail_stretching / 4
    
        if np.abs(opt_aoa) > boat.stall_deg/180.*np.pi:
            opt_aoa = np.sign(wind_angle) * boat.stall_deg/180.*np.pi
        # heading controllability at high wind speeds:
        if wind_speed > boat.limit_wind_speed:
            fact = (boat.limit_wind_speed / wind_speed)**2
            opt_aoa *= fact
        
        return np.abs(np.clip(wind_angle - opt_aoa, -np.pi/2, np.pi/2))