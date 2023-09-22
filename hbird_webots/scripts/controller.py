import numpy as np
from math import sin, cos

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

        ### Control Gains ###
        # translational --P--
        self.kp_x = pid_gains['kp_x']
        self.kp_y = pid_gains['kp_y']
        self.kp_z = pid_gains['kp_z']
        #               --I--
        self.ki_x = pid_gains['ki_x']
        self.ki_y = pid_gains['ki_y']
        self.ki_z = pid_gains['ki_z']
        #               --D--
        self.kd_x = pid_gains['kd_x']
        self.kd_y = pid_gains['kd_y']
        self.kd_z = pid_gains['kd_z']

        # rotational    --P--
        self.kp_phi = pid_gains['kp_phi']
        self.kp_theta = pid_gains['kp_theta']
        self.kp_psi = pid_gains['kp_psi']
        #               --I--
        self.ki_phi = pid_gains['ki_phi']
        self.ki_theta = pid_gains['ki_theta']
        self.ki_psi = pid_gains['ki_psi']
        #               --D--
        self.kd_p = pid_gains['kd_p']
        self.kd_q = pid_gains['kd_q']
        self.kd_r = pid_gains['kd_r']

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}
        """

        x_dd_d = 0.0
        y_dd_d = 0.0
        z_dd_d = 0.0

        error_x_d = 0.0 - state.velocity.x
        error_x = setpoint.position.x - state.position.x
        error_y_d = 0.0 - state.velocity.y
        error_y = setpoint.position.y - state.position.y
        error_z_d = 0.0 - state.velocity.z
        error_z = setpoint.position.z - state.position.z
        
        x_dd_c = x_dd_d + (self.kd_x * error_x_d) + (self.kp_x * error_x)
        y_dd_c = y_dd_d + (self.kd_y * error_y_d) + (self.kp_y * error_y)
        z_dd_c = z_dd_d + (self.kd_z * error_z_d) + (self.kp_z * error_z)
        
        psi_T = setpoint.heading

        phi_d = (1/ self.params.g) * ((x_dd_c * sin(psi_T)) - (y_dd_c * cos(psi_T)))
        theta_d = (1/ self.params.g) * ((x_dd_c * cos(psi_T)) + (y_dd_c * sin(psi_T)))
        psi_d = setpoint.heading

        p_d_d = 0.0
        q_d_d = 0.0
        r_d_d = 0.0

        error_p = 0.0 - state.angular_velocity.x
        error_phi = phi_d - state.orientation.x
        error_q = 0.0 - state.angular_velocity.y
        error_theta = theta_d - state.orientation.y
        error_r = 0.0 - state.angular_velocity.z
        error_psi = psi_d - state.orientation.z

        u1 = self.params.mass * (z_dd_c + self.params.g)
        u2 = p_d_d + (self.kd_p * error_p) + (self.kp_phi * error_phi)
        u3 = q_d_d + (self.kd_q * error_q) + (self.kp_theta * error_theta)
        u4 = r_d_d + (self.kd_r * error_r) + (self.kp_psi * error_psi)

        U = np.array([u1, u2, u3, u4])

        print(state.position)

        return U