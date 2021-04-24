import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math





class ApparentWind:
        '''
    inputs:
        boat:
            vel_x
            vel_y
            yaw
        environment:
            wind_x
            wind_y
    '''
    def __init__(self, boat, env): # might change these to initialize in a sensible way
        self.calculate_apparent_wind(boat, env)

    def calculate_apparent_wind(self, boat, env):
        ''' Calculate the apparent wind on the boat. 

        return: The apparent wind on the boat
        '''
        transformed_x = env.wind_x * math.cos(boat.yaw) + env.wind_y * math.sin(boat.yaw)
        transformed_y = env.wind_x * -math.sin(boat.yaw) + env.wind_y * math.cos(boat.yaw)

        self.apparent_x = transformed_x - boat.vel_x
        self.apparent_y = transformed_y - boat.vel_y
        self.apparent_angle = math.atan2(-self.apparent_y, -self.apparent_x)
        self.apparent_speed = math.sqrt(self.apparent_x**2 + self.apparent_y**2)

        return (self.apparent_x, self.apparent_y, self.apparent_angle, self.apparent_speed)



class Damping:
    '''
    inputs:
        boat:
            vel_x
            vel_y
            vel_z
            roll_rate
            pitch_rate
            yaw_rate
            invariants
    '''
    def __init__(self, boat):
        self.calculate_damping(boat)
    def calculate_damping(self, boat):
        self.x      = boat.damping_invariant_x * boat.vel_x
        self.y      = boat.damping_invariant_y *       boat.vel_y
        self.z      = boat.damping_invariant_z *       boat.vel_z
        self.roll   = boat.damping_invariant_roll *    boat.roll_rate
        self.pitch  = boat.damping_invariant_pitch *   boat.pitch_rate
        self.yaw    = boat.damping_invariant_yaw *     boat.yaw_rate
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

class WaveInfluence: 
    '''
    Used only by HydrostaticForce
    inputs:
        boat:
            pos_x
            pos_y
            yaw
        environment:
            gravity
            wave_length
            wave_direction
            wave_amplitude
            time
    '''

    def __init__(self, boat, env):
        self.calculate_wave_influence(boat, env)

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
        gradient_y = wave_y * factor
        self.height = env.wave_amplitude * math.sin(frequency * env.time - wave_x * boat.pos_x - wave_y * boat.pos_y)
        self.gradient_x = gradient_x * math.cos(boat.yaw) + gradient_y * math.sin(boat.yaw)
        self.gradient_y = gradient_y * math.cos(boat.yaw) - gradient_x * math.sin(boat.yaw)
# 
        return self.height, self.gradient_x, self.gradient_y

class HydrostaticForce:
    '''
    inputs:
        boat:
            pos_z
            roll
            pitch
        environment:
            wind_x
            wind_y
    '''
    def __init__(self, boat, env):
        self.wave_influence = WaveInfluence(boat, env)
        self.calculate_hydrostatic_force(boat, env)
        
    def calculate_hydrostatic_force(self, boat, env):
        self.wave_influence.calculate_wave_influence(boat, env)
        force = boat.hydrostatic_invariant_z * (boat.pos_z - self.wave_influence.height) + boat.gravity_force
        self.x = force * self.wave_influence.gradient_x
        self.y = force * self.wave_influence.gradient_y
        self.z = force
        self.x_hs = boat.hydrostatic_eff_y * math.sin(boat.pitch + math.atan(self.wave_influence.gradient_x))
        self.y_hs = boat.hydrostatic_eff_x * -math.sin(boat.roll - math.atan(self.wave_influence.gradient_y))
        return (self.x, self.y, self.z, self.x_hs, self.y_hs)


class RudderForce:
    '''
    inputs:
        boat:
            speed
            various dimensions
        environment:
            water_density
    '''
    def __init__(self, boat, env):
        self.calculate_rudder_force(boat, env)

    def calculate_rudder_force(self, boat, env):
        pressure = (env.water_density / 2) * boat.calculate_speed()**2
        self.x = -(((4 * math.pi) / boat.rudder_stretching) * boat.rudder_angle**2) * pressure * boat.rudder_blade_area
        self.y = 2 * math.pi * pressure * boat.rudder_blade_area * boat.rudder_angle
        return self.x, self.y


class LateralForce:
    '''
    inputs:
        boat:
            speed
            vel_y
            vel_x
            various dimensions
        environment:
            water_density
    '''
    def __init__(self, boat, env ):
        self.calculate_lateral_force(boat, env)

    def calculate_lateral_force(self, boat, env):
        ''' Calculate the lateral force. 
        return: The force applied to the lateral plane of the boat
        '''
        pressure = (env.water_density / 2) * boat.calculate_speed()**2 * math.cos(boat.roll)**2
        friction = 2.66 * math.sqrt(env.water_viscosity / (boat.calculate_speed() * boat.keel_length)) if boat.calculate_speed() else 0
        #     aoa :           angle of attack
        # eff_aoa : effective angle of attack
        aoa = math.atan2(boat.vel_y, boat.vel_x)
        eff_aoa = aoa
        if aoa < -math.pi / 2:
            eff_aoa = math.pi + aoa
        elif aoa > math.pi / 2:
            eff_aoa = -math.pi + aoa
        separation = 1- math.exp(-((abs(eff_aoa))/(math.pi/180*25))**2)
        tmp = -(friction + (4 * math.pi * eff_aoa**2 * separation) / boat.keel_stretching)
        separated_transverse_force = -np.sign(aoa) * pressure * boat.sail_area * math.sin(aoa)**2

        self.x = (1 - separation) * (tmp * math.cos(aoa) + 2 * math.pi * eff_aoa * math.sin(aoa)) * pressure * boat.lateral_area
        self.y = (1 - separation) * (tmp * math.sin(aoa) - 2 * math.pi * eff_aoa * math.cos(aoa)) * pressure * boat.lateral_area + separation * separated_transverse_force
        self.separation = separation

class SailForce:
    ''' Calculate the force that is applied to the sail. 
    inputs:
        apparent_wind
        true_sail_angle
        boat
        environment
    return: The force applied on the sail by the wind
    '''
    def __init__(self, boat, env ):
        self.calculate_sail_force(boat, env)
    def calculate_sail_force(self,apparent_wind, true_sail_angle boat, env):
        aoa = apparent_wind.apparent_angle - true_sail_angle
        if aoa * true_sail_angle < 0:
            aoa = 0
        if aoa < math.pi / 2:
            eff_aoa = math.pi + aoa
        else:
            eff_aoa = -math.pi + aoa
        pressure = (env.air_density / 2) * apparent_wind.apparent_speed**2 * math.cos(boat.roll * math.cos(true_sail_angle))**2
        friction = 3.55 * math.sqrt(env.air_viscosity / (apparent_wind.apparent_speed * boat.sail_length)) if apparent_wind.apparent_speed else 0
        separation = 1 - math.exp(-((abs(eff_aoa))/(math.pi/180*25))**2)

        propulsion = (2 * math.pi * eff_aoa * math.sin(apparent_wind.apparent_angle) - (friction + (4 * math.pi * eff_aoa**2 * separation) / boat.sail_stretching) * math.cos  (apparent_wind.apparent_angle)) * boat.sail_area * pressure

        transverse_force = (-2 * math.pi * eff_aoa * math.cos(apparent_wind.apparent_angle) - (friction + (4 * math.pi * eff_aoa**2 * separation) / boat.sail_stretching) *    math.sin(apparent_wind.apparent_angle)) * boat.sail_area * pressure

        separated_propulsion = np.sign(aoa) * pressure * boat.sail_area * math.sin(aoa)**2 * math.sin(true_sail_angle)
        separated_transverse_force = -np.sign(aoa) * pressure * boat.sail_area * math.sin(aoa)**2 * math.cos(true_sail_angle)
        self.x = (1 - separation) * propulsion + separation * separated_propulsion
        self.y =(1 - separation) * transverse_force + separation * separated_transverse_force
        return self.x, self.y


class Forces:
    '''
    inputs:
        boat:
        environment:
    '''
    def __init__(self, boat, env):

        # objects
        self.apparent_wind = ApparentWind(boat, env)
        self.damping = Damping(boat)
        self.hydrostatic_force = HydrostaticForce(boat, env)
        self.rudder_force = RudderForce(boat, env)
        self.lateral_force = LateralForce(boat, env)

        # floats
        self.wave_impedance = boat.calculate_wave_impedance(boat)
        self.true_sail_angle = boat.calculate_true_sail_angle(boat, apparent_wind)

        # more objects
        self.sail_force = SailForce(self.apparent_wind, self.true_sail_angle, boat, env)

        
    def calculate_forces(self, boat, env):
        # objects
        self.apparent_wind.calculate_apparent_wind(boat, env)
        self.damping.calculate_damping(self, boat)
        self.hydrostatic_force.calculate_hydrostatic_force(boat, env)
        self.rudder_force.calculate_rudder_force(boat, env)  
        self.lateral_force.calculate_lateral_force(boat, env)

        #floats
        self.wave_impedance = boat.calculate_wave_impedance(boat)
        self.true_sail_angle = self.calculate_true_sail_angle(env)
        
        # more objects
        self.sail_force.calculate_sail_force(self.apparent_wind, self.true_sail_angle, boat, env)

    def calculate_wave_impedance(self, boat):
        return -np.sign(boat.vel_x) * boat.calculate_speed()**2 * (boat.calculate_speed()/boat.hull_speed)**2 * boat.wave_impedance_invariant
    def calculate_true_sail_angle(self, boat, apparent_wind):
        return np.sign(apparent_wind.apparent_angle) * abs(boat.sail_angle)


class Controller:
    def __init__(self, sample_time, controller_params, boat, env):
        self.sample_time = sample time
        self.factor = boat.distance_cog_rudder * boat.rudder_blade_area * math.pi * env.water_density / boat.moi_z
        for key, val in controller_params.items():
            exec('self.' + key + '= val')
        # initial values:
        self.summed_error = 0
        self.filtered_drift = 0


        # calculate controller parameters
        # approximated system modell, linearized, continous calculated (seems to be fine)
        A = np.array([  [0, 1, 0],
                        [0, -1/boat.yaw_time_constant, 0],
                        [-1, 0, 0]])
        B = np.array([0, 1, 1])
        # weigth matrices for riccati design
        Q = np.diag([0.1, 1, 0.3])
        r = np.ones((1, 1))*30
        # calculating feedback
        P = scipy.linalg.solve_continuous_are(A, B[:, None], Q, r)
        K = np.sum(B[None, :] * P, axis = 1)/r[0, 0]
        self.KP = K[0]
        self.KD = K[1]
        self.KI = -K[2]

        # Desired path
        self.desired_path = Path(boat, env)
        
    def controll(self, desired_heading, drift_angle, boat, env):
        heading_error = desired_heading - heading_error
        # respect to periodicity of angle: maximum difference is 180 deg resp. pi
        while heading_error > math.pi:
            heading_error -= 2*math.pi
        while heading_error < -math.pi:
            heading_error += 2*math.pi
        self.summed_error += self.sample_time * (heading_error - drift_angle)
        # avoid high gains at low speed (singularity)
        if boat.calculate_speed() < self.speed_adaption:
            boat.calculate_speed() = self.speed_adaption
        
        factor2 = -1.0 / self.factor / boat.calculate_speed()**2 / math.cos(boat.roll)

        #control equation
        rudder_angle = factor2 * (self.KP * heading_error + self.KI * self.summed_error - self.KD * boat.yaw_rate)
        if abs(rudder_angle) > boat.max_rudder_angle:
            rudder_angle = np.clip(rudder_angle, -boat.max_rudder_angle, boat.max_rudder_angle)
            self.summed_error = (rudder_angle/factor2 - (self.KP * heading_error - self.KD * boat.yaw_rate)) / self.KI
        return rudder_angle 


class Path:
    def __init__(self, boat, env):
        self.desired_heading = 0.5*math.pi # for testing
    def get_desired_heading(self):
        return self.desired_heading


class Boat:
    """The class for the boat state"""
    def __init__(self, sim_params, env):
        for key, val in sim_params['boat']['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
        for key, val in sim_params['boat']['boat_dimensions'].items():
            exec('self.' + key + '= val')
        ###
        # Invariants
        self.wave_impedance_invariant = (env.water_density / 2) * self.lateral_area
        # Hydrostatic force
        self.hydrostatic_eff_x        = self.height_bouyancy + (env.water_density / self.mass) * self.geometrical_moi_x
        self.hydrostatic_eff_y        = self.height_bouyancy + (env.water_density / self.mass) * self.geometrical_moi_y
        self.hydrostatic_invariant_z  =  -env.water_density * self.waterline_area * env.gravity
        self.gravity_force            = self.mass * env.gravity
        # Damping
        self.damping_invariant_x      = -self.mass / self.along_damping
        self.damping_invariant_y      = -self.mass / self.transverse_damping
        self.damping_invariant_z      = -.5 * self.damping_z * math.sqrt(env.water_density * self.waterline_area * env.gravity * self.mass)
        self.damping_invariant_yaw    = -(self.moi_z / self.yaw_timeconstant)
        self.damping_invariant_pitch  = -2 * self.pitch_damping * math.sqrt(self.moi_y * self.mass * env.gravity * self.hydrostatic_eff_y)
        self.damping_invariant_roll   = -2 * self.roll_damping * math.sqrt(self.moi_x * self.mass * env.gravity * self.hydrostatic_eff_x)
        ### End Invariants

        # forces
        self.forces = Forces(self, env)

    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)

    def calculate_drift(self):
        return math.arctan2(self.vel_y, self.vel_x)


    def calculate_state_delta(self, ref):
        delta_pos_x = self.vel_x * math.cos(self.yaw) - self.vel_y * math.sin(self.yaw)
        delta_pos_y = self.vel_y * math.cos(self.yaw) - self.vel_x * math.sin(self.yaw)
        delta_pos_z = self.vel_z
        delta_roll = self.roll_rate
        delta_pitch = pitch_rate * math.cos(self.roll) - self.yaw_rate * math.sin(roll)
        delta_yaw = self.yaw_rate * math.cos(self.roll) + self.pitch_rate * math.sin(self.roll)

        delta_vel_x = delta_yaw * self.vel_y + (self.sail_force.x + self.lateral_force.x + self.rudder_force.x + self.damping.x + self.wave_impedance + self.hydrostatic_force.x) / self.mass
        delta_vel_y = -delta_yaw * self.vel_x + ((self.sail_force.y + self.lateral_force.y + self.rudder_force.y) * math.cos(self.roll) + self.hydrostatic_force.y + self.damping.y) / self.mass
        delta_vel_z = ((self.sail_force.y + self.lateral_force.y + self.rudder_force.y) * math.sin(self.roll) + self.hydrostatic_force.z - self.gravity_force + self.damping.z) / self.mass

        delta_roll_rate = (self.hydrostatic_force.z * self.hydrostatic_force.y_hs - self.sail_force.y * self.sail_pressure_point_height + self.damping.roll)/self.moi_x
        delta_pitch_rate = (self.sail_force.x * self.sail_pressure_point_height - self.hydrostatic_force.z * self.hydrostatic_force.x_hs * math.cos(self.roll) + self.damping.pitch - (self.moi_x - self.moi_z) * self.roll_rate * self.yaw_rate) / self.moi_y
        delta_yaw_rate = (self.damping.yaw - self.rudder_force.y * self.distance_cog_rudder + self.sail_force.y * self.distance_cog_sail_pressure_point + self.sail_force.x * math.sin(self.true_sail_angle) * self.distance_mast_sail_pressure_point + self.lateral_force.y * (self.distance_cog_keel_pressure_point * (1-self.lateral_force.lateral_separation) + self.distance_cog_keel_middle * self.lateral_force.lateral_separation))/ self.moi_z

        delta_rudder = -2 * (self.rudder_angle - ref.rudder_angle)
        delta_rudder = np.clip(delta_rudder, -self.max_rudder_speed, self.max_rudder_speed)
        delta_sail = -0.1 * (self.true_sail_angle - ref.sail_angle)
        delta_sail = np.clip(delta_sail, -self.max_sail_speed, self.max_sail_speed)
        delta = np.array([  delta_pos_x,     delta_pos_y,      delta_pos_z,
                            delta_roll,      delta_pitch,      delta_yaw,
                            delta_vel_x,     delta_vel_y,      delta_vel_z,
                            delta_roll_rate, delta_pitch_rate, delta_yaw_rate]))
        return delta
        
        def set_state(self, boat_state, env):
            self.pos_x,          self.pos_y,          self.pos_z    = boat_state[0:3]
            self.roll,           self.pitch,          self.yaw      = boat_state[3:6]
            self.vel_x,          self.vel_y,          self.vel_z    = boat_state[6:9]
            self.roll_rate,      self.pitch_rate,     self.yaw_rate = boat_state[9:12]
            self.rudder_angle,   self.sail_angle               = boat_state[12:14]
            self.calculate_forces(env)
        
        def get_state(self):
            return np.array([   self.pos_x,          self.pos_y,          self.pos_z,   
                                self.roll,           self.pitch,          self.yaw,     
                                self.vel_x,          self.vel_y,          self.vel_z,   
                                self.roll_rate,      self.pitch_rate,     self.yaw_rate,
                                self.rudder_angle,   self.sail_angle])