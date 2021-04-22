import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import yaml
import io
import math


class WaveInfluence:
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


class ApparentWind:
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
    def __init__(self, boat, env):
        self.calculate_damping(boat, env)
    def calculate_damping(self, boat, env):
        self.x      = boat.damping_invariant_x * boat.vel_x
        self.y      = boat.damping_invariant_y *       boat.vel_y
        self.z      = boat.damping_invariant_z *       boat.vel_z
        self.roll   = boat.damping_invariant_roll *    boat.roll_rate
        self.pitch  = boat.damping_invariant_pitch *   boat.pitch_rate
        self.yaw    = boat.damping_invariant_yaw *     boat.yaw_rate
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


class HydrostaticForce:
    def __init__(self, boat, env):
        self.calculate_hydrostatic_force(boat, env)
    def calculate_hydrostatic_force(self, boat, env):
        force = boat.hydrostatic_invariant_z * (boat.pos_z - boat.wave_influence.height) + boat.gravity_force
        self.x = force * boat.wave_influence.gradient_x
        self.y = force * boat.wave_influence.gradient_y
        self.z = force
        self.x_hs = boat.hydrostatic_eff_y * math.sin(boat.pitch + math.atan(boat.wave_influence.gradient_x))
        self.y_hs = boat.hydrostatic_eff_x * -math.sin(boat.roll - math.atan(boat.wave_influence.gradient_y))
        return (self.x, self.y, self.z, self.x_hs, self.y_hs)


class RudderForce:
    def __init__(self, boat, env):
        self.calculate_rudder_force(boat, env)

    def calculate_rudder_force(self, boat, env):
        pressure = (env.water_density / 2) * boat.calculate_speed()**2
        self.x = -(((4 * math.pi) / boat.rudder_stretching) * boat.rudder_angle**2) * pressure * boat.rudder_blade_area
        self.y = 2 * math.pi * pressure * boat.rudder_blade_area * boat.rudder_angle
        return self.x, self.y



class LateralForce:
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

    return: The force applied on the sail by the wind
    '''
    def __init__(self, boat, env ):
        self.calculate_sail_force(boat, env)
    def calculate_sail_force(self, boat, env):
        aoa = boat.apparent_wind.apparent_angle - boat.true_sail_angle
        if aoa * boat.true_sail_angle < 0:
            aoa = 0
        if aoa < math.pi / 2:
            eff_aoa = math.pi + aoa
        else:
            eff_aoa = -math.pi + aoa
        pressure = (env.air_density / 2) * boat.apparent_wind.apparent_speed**2 * math.cos(boat.roll * math.cos(boat.true_sail_angle))**2
        friction = 3.55 * math.sqrt(env.air_viscosity / (boat.apparent_wind.apparent_speed * boat.sail_length)) if boat.apparent_wind.apparent_speed else 0
        separation = 1 - math.exp(-((abs(eff_aoa))/(math.pi/180*25))**2)

        propulsion = (2 * math.pi * eff_aoa * math.sin(boat.apparent_wind.apparent_angle) - (friction + (4 * math.pi * eff_aoa**2 * separation) / boat.sail_stretching) * math.cos  (boat.apparent_wind.apparent_angle)) * boat.sail_area * pressure

        transverse_force = (-2 * math.pi * eff_aoa * math.cos(boat.apparent_wind.apparent_angle) - (friction + (4 * math.pi * eff_aoa**2 * separation) / boat.sail_stretching) *    math.sin(boat.apparent_wind.apparent_angle)) * boat.sail_area * pressure

        separated_propulsion = np.sign(aoa) * pressure * boat.sail_area * math.sin(aoa)**2 * math.sin(boat.true_sail_angle)
        separated_transverse_force = -np.sign(aoa) * pressure * boat.sail_area * math.sin(aoa)**2 * math.cos(boat.true_sail_angle)
        self.x = (1 - separation) * propulsion + separation * separated_propulsion
        self.y =(1 - separation) * transverse_force + separation * separated_transverse_force


class Controller:
    def __init__(self, sample_time, controller_params):
        self.sample_time = sample time
        self.factor
        for key, val in controller_params.items():
            exec('self.' + key + '= val')
        self.rudder_angle = 0
        self.sail_angle = 0













class Boat:
    """The class for the boat state"""
    def __init__(self, boat_params, env):
        for key, val in boat_params['initial_state'].items():
            exec('self.' + key + '= val') # this is supposed to load all the parameters as variables https://stackoverflow.com/questions/18090672/convert-dictionary-entries-into-variables-python
        for key, val in boat_params['boat_dimensions'].items():
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
        ###
        self.max_rudder_speed = math.pi/30
        self.max_sail_speed = math.pi/10



        self.wave_influence = WaveInfluence(self, env)
        self.apparent_wind = ApparentWind(self, env)
        self.true_sail_angle = self.calculate_true_sail_angle(env)
        self.damping = Damping(self, env)
        self.hydrostatic_force = HydrostaticForce(self, env)
        self.wave_impedance = self.calculate_wave_impedance(env)
        self.rudder_force = RudderForce(self, env)
        self.lateral_force = LateralForce(self, env)
        self.sail_force = SailForce(self, env)
    def calculate_forces(self, env):
        self.wave_influence.calculate_wave_influence(self, env)
        self.apparent_wind.calculate_apparent_wind(self, env)
        self.true_sail_angle = self.calculate_true_sail_angle(env)
        self.damping.calculate_damping(self, env)
        self.hydrostatic_force.calculate_hydrostatic_force(self, env)
        self.wave_impedance = self.calculate_wave_impedance(env)
        self.rudder_force.calculate_rudder_force(self, env)     
        self.lateral_force.calculate_lateral_force(self, env)   
        self.sail_force.calculate_sail_force(self, env)

    def calculate_wave_impedance(self, env):
        return -np.sign(self.vel_x) * self.calculate_speed()**2 * (self.calculate_speed()/self.hull_speed)**2 * self.wave_impedance_invariant

    def calculate_speed(self):
        return math.sqrt(self.vel_x**2 + self.vel_y**2)

    def calculate_true_sail_angle(self, env):
        return np.sign(self.apparent_wind.apparent_angle) * abs(self.sail_angle)

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
        
        def set_state(self, boat_state):
            self.pos_x,          self.pos_y,          self.pos_z    = boat_state[0:3]
            self.roll,           self.pitch,          self.yaw      = boat_state[3:6]
            self.vel_x,          self.vel_y,          self.vel_z    = boat_state[6:9]
            self.roll_rate,      self.pitch_rate,     self.yaw_rate = boat_state[9:12]
            self.rudder_angle,   self.sail_angle               = boat_state[12:14]
        
        def get_state(self):
            return np.array([   self.pos_x,          self.pos_y,          self.pos_z,   
                                self.roll,           self.pitch,          self.yaw,     
                                self.vel_x,          self.vel_y,          self.vel_z,   
                                self.roll_rate,      self.pitch_rate,     self.yaw_rate,
                                self.rudder_angle,   self.sail_angle])