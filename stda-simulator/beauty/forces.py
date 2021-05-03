import numpy as np
import scipy
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
    def __init__(self, boat, env):
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
    def __init__(self, apparent_wind, true_sail_angl, boat, env):
        self.calculate_sail_force(apparent_wind, true_sail_angl, boat, env)

    def calculate_sail_force(self, apparent_wind, true_sail_angle, boat, env):
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
        self.wave_impedance = self.calculate_wave_impedance(boat)
        self.true_sail_angle = self.calculate_true_sail_angle(boat, self.apparent_wind)

        # more objects
        self.sail_force = SailForce(self.apparent_wind, self.true_sail_angle, boat, env)

        
    def calculate_forces(self, boat, env):
        # objects
        self.apparent_wind.calculate_apparent_wind(boat, env)
        self.damping.calculate_damping(boat)
        self.hydrostatic_force.calculate_hydrostatic_force(boat, env)
        self.rudder_force.calculate_rudder_force(boat, env)  
        self.lateral_force.calculate_lateral_force(boat, env)

        #floats
        self.wave_impedance = self.calculate_wave_impedance(boat)
        self.true_sail_angle = self.calculate_true_sail_angle(boat, self.apparent_wind)
        
        # more objects
        self.sail_force.calculate_sail_force(self.apparent_wind, self.true_sail_angle, boat, env)

    def calculate_wave_impedance(self, boat):
        return -np.sign(boat.vel_x) * boat.calculate_speed()**2 * (boat.calculate_speed()/boat.hull_speed)**2 * boat.wave_impedance_invariant
    def calculate_true_sail_angle(self, boat, apparent_wind):
        return np.sign(apparent_wind.apparent_angle) * abs(boat.sail_angle)