import numpy as np

class Path:
    def yaw(self, time):
        if time < 40:
            return 0.0
        elif time < 90:
            return np.pi/4
        # elif time < 100:
        #     return np.pi/4
        else:
            return np.pi/2
        # return 0