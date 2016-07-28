#!/usr/bin/python
 
import numpy as np
import math
 
class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=1 # kp > 0
        self.ka=1.2# ka > kp
        self.kb=0 # kb < 0
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
       
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        delta_x = (goal[0] - state[0]).item()
        delta_y = (goal[1] - state[1]).item()
        theta = state[2].item()

        rho = math.sqrt(delta_x ** 2 + delta_y ** 2)
        self.alpha = np.arctan2(delta_y, delta_x) - theta

        if self.alpha > (3 * math.pi / float(2)):
            self.alpha = self.alpha - 2 * math.pi 
        elif self.alpha < -(3 * math.pi / float(2)):
            self.alpha = self.alpha + 2 * math.pi

        self.alpha = max(-math.pi/2 , min(math.pi/2 , self.alpha))
            
        
        beta = -theta - self.alpha

        v = min(self.kp * rho, self.MAX_SPEED)
        omega_sign  = (self.ka * self.alpha + self.kb * beta) > 0
        omega = min(abs(self.ka * self.alpha + self.kb * beta), self.MAX_OMEGA)

        done = rho < 0.164
        # done = rho < 0.164
        # done = rho < 0.2
        # return (v, self.ka * self.alpha + self.kb * beta, done)
        return (v, omega if omega_sign else -omega, done)
 
