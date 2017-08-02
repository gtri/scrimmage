import numpy as np
import math
import scrimmage as sc

class Straight(object):
    def init(self, params):
        self.speed = float(params['speed'])
        self.state = sc.State()
        self.desired_state = sc.State()
        self.contacts = dict()
    def step_autonomy(self, t, dt):
        # Move in the forward direction.
        desired = sc.State()
        desired.vel = np.array([self.speed, 0, 0])
        self.desired_state = desired
        
        return True
