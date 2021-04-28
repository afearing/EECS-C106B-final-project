import numpy as np 

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
        self.damping_invariant_z      = -.5 * self.damping_z * np.sqrt(env.water_density * self.waterline_area * env.gravity * self.mass)
        self.damping_invariant_yaw    = -(self.moi_z / self.yaw_timeconstant)
        self.damping_invariant_pitch  = -2 * self.pitch_damping * np.sqrt(self.moi_y * self.mass * env.gravity * self.hydrostatic_eff_y)
        self.damping_invariant_roll   = -2 * self.roll_damping * np.sqrt(self.moi_x * self.mass * env.gravity * self.hydrostatic_eff_x)
        # IDK
        self.distance_cog_keel_middle = self.distance_cog_keel_pressure_point - 0.7
        ### End Invariants
        
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
                            