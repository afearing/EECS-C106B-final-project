import numpy as np
from collections import namedtuple
from environment import *

# Structured data
# Forces
RudderForce      = namedtuple('RudderForce', 'x, y')
LateralForce     = namedtuple('LateralForce', 'x, y')
SailForce        = namedtuple('SailForce', 'x, y')
HydrostaticForce = namedtuple('HydrostaticForce', 'x, y, z')
Damping          = namedtuple('Damping', 'x, y, z, yaw, pitch, roll')

def calculate_wave_influence(pos_x, pos_y, yaw, wave, time, gravity):
    ''' Calculate how the waves influence the boat. 
    
    param pos_x:    The boats position on the x-axis        [m]  
    param pos_y:    The boats position on the y-axis        [m]
    param yaw:      The heading of the boat                 [radians]        
    param wave:     The direction and length of the waves
    param time:     The simulation time                     [s]

    return: The influence of the waves on the boat
    '''
    frequency = np.sqrt((2 * np.pi * gravity) / wave.length)

    k = WaveVector(x=2 * np.pi / wave.length * np.cos(wave.direction),
                   y=2 * np.pi / wave.length * np.sin(wave.direction))

    factor = -wave.amplitude * np.cos(frequency * time - k.x * pos_x - k.y * pos_y)
    gradient_x = k.x * factor
    gradient_y = k.y * factor

    return WaveInfluence(
        height=wave.amplitude * np.sin(frequency * time - k.x * pos_x - k.y * pos_y),
        gradient_x=gradient_x * np.cos(yaw) + gradient_y * np.sin(yaw),
        gradient_y=gradient_y * np.cos(yaw) - gradient_x * np.sin(yaw))

def calculate_apparent_wind(yaw, vel_x, vel_y, true_wind):
    ''' Calculate the apparent wind on the boat. 

    param yaw:          The heading of the boat [radians]
    param vel_x:        The velocity along the x-axis [m/s]
    param vel_y:        The velocity along the y-axis [m/s]
    param true_wind:    The true wind directions

    return: The apparent wind on the boat
    '''
    transformed_x = true_wind.x * np.cos(yaw) + true_wind.y * np.sin(yaw)
    transformed_y = true_wind.x * -np.sin(yaw) + true_wind.y * np.cos(yaw)

    apparent_x = transformed_x - vel_x
    apparent_y = transformed_y - vel_y
    apparent_angle = np.arctan2(-apparent_y, -apparent_x)
    apparent_speed = np.sqrt(apparent_x**2 + apparent_y**2)

    return ApparentWind(x=apparent_x,
                        y=apparent_y,
                        angle=apparent_angle,
                        speed=apparent_speed)

def calculate_damping(vel_x, vel_y, vel_z, roll_rate, pitch_rate, yaw_rate, boat):
    ''' Calculate the damping. 
    
    param vel_x:        The velocity along the x-axis           [m/s]  
    param vel_y:        The velocity along the y-axis           [m/s]  
    param vel_z:        The velocity along the z-axis           [m/s]  
    param roll_rate:    The rate of change to the roll angle    [radians/s]
    param pitch_rate:   The rate of change to the pitch angle   [radians/s]
    param yaw_rate:     The rate of change to the yaw angle     [radians/s]

    return: The amount of damping applied to the boat
    '''
    return Damping(
        x=boat.damping_invariant_x * vel_x,
        y=boat.damping_invariant_y * vel_y,
        z=boat.damping_invariant_z * vel_z,
        roll=boat.damping_invariant_roll * roll_rate,
        pitch=boat.damping_invariant_pitch * pitch_rate,
        yaw=boat.damping_invariant_yaw * yaw_rate)

def calculate_hydrostatic_force(pos_z, roll, pitch, wave_influence, boat):
    ''' Calculate the hydrostatic force. 
    
    param pos_z:            The boats position on the z-axis        [m]  
    param roll:             The roll angle of the boat              [radians]
    param pitch:            The pitch angle of the boat             [radians]
    param wave_influence:   The influence of the waves on the boat     

    return: The force applied on the boat by the waves
    '''
    force = boat.hydrostatic_invariant_z * (pos_z - wave_influence.height) + boat.gravity_force
    
    return HydrostaticForce(
        x=force * wave_influence.gradient_x,
        y=force * wave_influence.gradient_y,
        z=force), \
            boat.hydrostatic_eff_y * np.sin(pitch + np.arctan(wave_influence.gradient_x)), \
            boat.hydrostatic_eff_x * -np.sin(roll - np.arctan(wave_influence.gradient_y)),

def calculate_wave_impedance(vel_x, speed, boat):
    ''' Calculate the wave impedance. 
    
    param vel_x: The velocity along the x-axis  [m/s]    
    param speed: The total speed of the boat    [m/s]

    return: The force applied to the rudder of the boat
    '''
    return -np.sign(vel_x) * speed**2 * (speed / boat.hull_speed)**2 * boat.wave_impedance_invariant

def calculate_rudder_force(speed, rudder_angle, boat, env):
    ''' Calculate the force that is applied to the rudder. 
   
    param speed:        The total speed of the boat [m/s]
    param rudder_angle: The angle of the rudder     [radians]

    return: The force applied to the rudder of the boat
    '''
    pressure = (env.water_density / 2) * speed**2
    return RudderForce(
        x=-(((4 * np.pi) / boat.rudder_stretching) * rudder_angle**2) * pressure * boat.rudder_blade_area,
        y=2 * np.pi * pressure * boat.rudder_blade_area * rudder_angle)

def calculate_lateral_force(vel_x, vel_y, roll, speed, boat, env):
    ''' Calculate the lateral force. 
    
    param vel_x:        The velocity along the x-axis   [m/s]
    param vel_y:        The velocity along the y-axis   [m/s]
    param roll:         The roll angle of the boat      [radians]
    param speed:        The total speed of the boat     [m/s]

    return: The force applied to the lateral plane of the boat
    '''
    pressure = (env.water_density / 2) * speed**2 * np.cos(roll)**2

    friction = 2.66 * np.sqrt(env.water_viscosity / (speed * boat.keel_length)) \
               if speed != 0                                        \
               else 0

    #     aoa :           angle of attack
    # eff_aoa : effective angle of attack
    eff_aoa = aoa = np.arctan2(vel_y, vel_x)
    if aoa < -np.pi / 2:
        eff_aoa = np.pi + aoa
    elif aoa > np.pi / 2:
        eff_aoa = -np.pi + aoa
    
    separation = 1-np.exp(-((np.abs(eff_aoa))/(np.pi/180*25))**2)

    # Identical calculation for x and y
    tmp = -(friction + (4 * np.pi * eff_aoa**2 * separation) / boat.keel_stretching)
    
    separated_transverse_force = -np.sign(aoa) * pressure * boat.sail_area * np.sin(aoa)**2
    
    return LateralForce(
        x=(1 - separation) * (tmp * np.cos(aoa) + 2 * np.pi * eff_aoa * np.sin(aoa)) * pressure * boat.lateral_area,
        y=(1 - separation) * (tmp * np.sin(aoa) - 2 * np.pi * eff_aoa * np.cos(aoa)) * pressure * boat.lateral_area + separation * separated_transverse_force)\
            , separation

def calculate_sail_force(roll, wind, sail_angle, boat, env):
    ''' Calculate the force that is applied to the sail. 
    
    param roll:         The roll angle of the boat [radians]
    param wind:         The apparent wind on the boat  
    param sail_angle:   The angle of the main sail [radians]

    return: The force applied on the sail by the wind
    '''
    # aoa : angle of attack
    aoa = wind.angle - sail_angle
    if aoa * sail_angle < 0:
        aoa = 0
    # eff_aoa : effective angle of attack
    eff_aoa = aoa
    if aoa < -np.pi / 2:
        eff_aoa = np.pi + aoa
    elif aoa > np.pi / 2:
        eff_aoa = -np.pi + aoa

    pressure = (env.air_density / 2) * wind.speed**2 * np.cos(roll * np.cos(sail_angle))**2

    friction = 3.55 * np.sqrt(env.air_viscosity / (wind.speed * boat.sail_length)) \
               if wind.speed != 0                                      \
               else 0
    
    separation = 1-np.exp(-((np.abs(eff_aoa))/(np.pi/180*25))**2)
    
    propulsion = (2 * np.pi * eff_aoa * np.sin(wind.angle)                                       \
                  -(friction + (4 * np.pi * eff_aoa**2 * separation) / boat.sail_stretching) * np.cos(wind.angle)) \
                  * boat.sail_area * pressure

    transverse_force = (-2 * np.pi * eff_aoa * np.cos(wind.angle)                                      \
                        -(friction + (4 * np.pi * eff_aoa**2 * separation) / boat.sail_stretching) * np.sin(wind.angle)) \
                        * boat.sail_area * pressure

    separated_propulsion = np.sign(aoa) * pressure * boat.sail_area * np.sin(aoa)**2 * np.sin(sail_angle)
    separated_transverse_force = -np.sign(aoa) * pressure * boat.sail_area * np.sin(aoa)**2 * np.cos(sail_angle)
    
    return SailForce(
        x=(1 - separation) * propulsion + separation * separated_propulsion,
        y=(1 - separation) * transverse_force + separation * separated_transverse_force)