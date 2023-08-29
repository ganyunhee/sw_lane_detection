import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time


class LateralController:
    '''
    Lateral control using the Stanley controller

    functions:
        stanley 

    init:
        gain_constant (default=5)
        damping_constant (default=0.5)
    '''


    def __init__(self, gain_constant=5, damping_constant=0.6):

        self.gain_constant = gain_constant
        self.damping_constant = damping_constant
        self.previous_steering_angle = 0


    def stanley(self, waypoints, speed):
        '''
        ##### TODO #####
        one step of the stanley controller with damping
        args:
            waypoints (np.array) [2, num_waypoints]
            speed (float)
        '''
        
        # CMNT : get path segment and car trajectory
        trajectory_len = ( (waypoints[0][0] - waypoints[0][1])**2
                      + (waypoints[1][0] - waypoints[1][1])**2) ** 1/2  #pi/2

        trajectory = [-(waypoints[1][0] - waypoints[1][1]) / trajectory_len,
                      -(waypoints[0][0] - waypoints[0][1]) / trajectory_len]

        # derive orientation error as the angle of the first path segment to the car orientation
        # CMNT : orientation or heading error -- used to line up the vehicle with the path
        
        heading_err = trajectory[1]
        
        # derive cross track error as distance between desired waypoint at spline parameter equal zero ot the car position
        # CMNT : cross-track error -- used to measure the distance from the front to the nearest point in th trajectory
        
        cross_track_err = waypoints[0][0] - 48
        
        # prevent division by zero by adding a small epsilon 
        # CMNT : softening constant epsilon or ds
        
        softened_speed = speed + 1e-9 # add epsilon to speed
        
        # derive stanley control law
        # CMNT : steering angle = heading error + arctan(k * cross-track error / speed)
        #           --> delta = heading_err + np.arctan2(self.gain_constant * cross_track_err, speed)
        
        delta = 1/2 * heading_err + np.arctan2(self.gain_constant * cross_track_err, softened_speed)  
        
        # derive damping
        # CMNT : steering angle = steering command - damping*(steering command - previous steering command)
        
        delta = delta - self.damping_constant*(delta - self.previous_steering_angle)
        
        # clip to the maximum steering angle (0.4) and rescale the steering action space
    
        max_steer = 0.4
        delta = np.clip(delta, -max_steer, max_steer) / max_steer
        
        return delta

        
        """
        crosstrack error : waypoints의 맨 처음값 - (48,0)
                            initial value of waypoints : (48, 0)
        heading error : waypoints의 첫 번째 인덱스 값과 그 다음 인덱스에 해당되는 값 사용
                        use values pertaining to first index and next index
        """





