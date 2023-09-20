import numpy as np
from math import sin, cos

from ../hbird_common/hbird_msgs/msg/State.msg import Waypoint, State

from hbird_msgs.msg import Waypoint, State  #need to figure this out, file is in a different directory


class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, hbparams, pid_gains, dt=0):
        """
        Inputs:
        - hbparams (HBarams dataclass):             model parameter class for the drone
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params = hbparams

        ### CARLO LOOK HERE AT LINE 30: uncomment it and the sim breaks

        # set control gains here
        ### Control Gains ###
        # translational                    ---P---
        # self.kp_x = pid_gains.kp_x ### THIS BREAKS SOMETHING



        # self.kp_y = pid_gains.kp_y
        # self.kp_z = pid_gains.kp_z
        # #                                  ---I---
        # self.ki_x = pid_gains.ki_x
        # self.ki_y = pid_gains.ki_y
        # self.ki_z = pid_gains.ki_z
        # #                                  ---D---
        # self.kd_x = pid_gains.kd_x
        # self.kd_y = pid_gains.kd_y
        # self.kd_z = pid_gains.kd_z
        
        # # rotational                       ---P---
        # self.kp_phi = pid_gains.kp_phi     
        # self.kp_theta = pid_gains.kp_theta
        # self.kp_psi = pid_gains.kp_psi
        # #                                  ---I---
        # self.ki_phi = pid_gains.ki_phi
        # self.ki_theta = pid_gains.ki_theta
        # self.ki_psi = pid_gains.ki_psi
        # #                                  ---D---
        # self.kd_p = pid_gains.kd_p
        # self.kd_q = pid_gains.kd_q
        # self.kd_r = pid_gains.kd_r

        ### STATE MEMBERS

        self.integral = State()
        self.integral.x = 0.0
        self.integral.y = 0.0
        self.integral.z = 0.0
        self.integral.w = 0.0

        self.prev_error = State()
        self.prev_error.x = 0.0
        self.prev_error.y = 0.0
        self.prev_error.z = 0.0
        self.prev_error.w = 0.0

        self.error = State()
        self.error.x = 0.0
        self.error.y = 0.0
        self.error.z = 0.0
        self.error.w = 0.0

        ### PHYSICAL PARAMETERS

        self.m = 1.0    # mass of the drone
        self.I = 1.0    # moment of rotational inertia about z axis 
        self.g = 9.81   # gravity



    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        """
        U = np.array([0.,0.,0.,0.])

        # your code here

        

        return U