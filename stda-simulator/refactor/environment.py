import scipy
import numpy as np

from collections import namedtuple

# Structured data
# Environment
TrueWind         = namedtuple('TrueWind', 'x, y, strength, direction')
ApparentWind     = namedtuple('ApparentWind', 'x, y, angle, speed')
Wave             = namedtuple('Wave', 'length, direction, amplitude')
WaveVector       = namedtuple('WaveVector', 'x, y')
WaveInfluence    = namedtuple('WaveInfluence', 'height, gradient_x, gradient_y')

class Environment:
    """The class for the environment and its properties. Note that this includes the boat dimensions"""
    def __init__(self, env_params):
        for key, val in env_params['properties'].items():
            exec('self.' + key + '= val')
        for key, val in env_params['disturbances'].items():
            exec('self.' + key + '= val')
        
        self.wind_x = self.wind_strength * np.cos(np.deg2rad(self.wind_direction)) # change to degrees
        self.wind_y = self.wind_strength * np.sin(np.deg2rad(self.wind_direction))

        self.true_wind = TrueWind(
            x = self.wind_x, y = self.wind_y,
            strength = self.wind_strength, direction = self.wind_direction
        )

        self.wave = Wave(
            length = self.wave_length,
            direction = self.wave_direction,
            amplitude = self.wave_amplitude
        )
