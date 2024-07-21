import numpy as np


class LateralController:
    '''
    Lateral control using the Stanley controller

    functions:
        stanley 

    init:
        gain_constant (default=5)
        damping_constant (default=0.5)
    '''


    def __init__(self, gain_constant=5, damping_constant=0.5):

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

        # derive orientation error as the angle of the first path segment to the car orientation
        error = waypoints[:, 1] - waypoints[:, 0] 
        orientation_error = np.arctan(error[0]/error[1])
        
        # derive stanley control law
        # derive cross track error as distance between desired waypoint at spline parameter equal zero at the car position
        # prevent division by zero by adding as small epsilon 
        # derive damping
        car_position = np.array([48,0])
        first_point = waypoints[:, 0]
        dx = first_point[0] - car_position[0]
        dy = first_point[1] - car_position[1]
        cross_track_error = np.sum(np.sqrt(dx**2 + dy **2))
        
        angle = np.arctan(self.gain_constant * cross_track_error / (speed + 1e-8))
        if angle > np.pi:
            angle -= 2.0 * np.pi
        elif angle < -np.pi:
            angle += 2.0 * np.pi
        
        if dx > 0:
            damping_sc = orientation_error + angle
        else:
            damping_sc = orientation_error - angle
        
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        damping_sc = np.clip(damping_sc, -0.4, 0.4)
        
        damp = self.damping_constant * (damping_sc - self.previous_steering_angle)
        steering_angle = damping_sc - damp
        self.previous_steering_angle = steering_angle
            
        return steering_angle






