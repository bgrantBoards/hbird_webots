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

    def compute_commands(self, setpoint:Waypoint, state:State):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}
        """

        # compute current cartesian positional error
        error_x = setpoint.position.x - state.position.x
        error_y = setpoint.position.y - state.position.y
        error_z = setpoint.position.z - state.position.z
        
        # use PID control to compute linear accelerations
        x_dd = self.kd_x + (self.kp_x * error_x)
        y_dd = self.kd_y + (self.kp_y * error_y)
        z_dd = self.kd_z + (self.kp_z * error_z)

        # compute angular position setpoints (derived from linear accelerations)
        psi_T = setpoint.heading
        phi_set = (1/ self.params.g) * ((x_dd * sin(psi_T)) - (y_dd * cos(psi_T)))
        theta_set = (1/ self.params.g) * ((x_dd * cos(psi_T)) + (y_dd * sin(psi_T)))
        psi_set = setpoint.heading
        
        # compute current angular error
        # error_p = 0.0 - state.angular_velocity.x
        error_phi = phi_set - state.orientation.x
        # error_q = 0.0 - state.angular_velocity.y
        error_theta = theta_set - state.orientation.y
        # error_r = 0.0 - state.angular_velocity.z
        error_psi = psi_set - state.orientation.z


        u1 = self.params.mass * (z_dd + self.params.g)                       # Z-axis   thrust
        # u2 = p_d_d + (self.kd_p * error_p) + (self.kp_phi * error_phi)       # X-axis?  thrust
        # u3 = q_d_d + (self.kd_q * error_q) + (self.kp_theta * error_theta)   # Y-axis?  thrust
        # u4 = r_d_d + (self.kd_r * error_r) + (self.kp_psi * error_psi)       # yaw?
        # u2 = self.params.I * ((self.kd_p * error_phi) + (self.kp_phi * error_phi)     )   # X-axis?  thrust
        # u3 = self.params.I * ((self.kd_q * error_theta) + (self.kp_theta * error_theta) )   # Y-axis?  thrust
        # u4 = self.params.I * ((self.kd_r * error_psi) + (self.kp_psi * error_psi)     )   # yaw?
        u2 = ((self.kd_p * error_phi) + (self.kp_phi * error_phi)     )     # X-axis?  thrust
        u3 = ((self.kd_q * error_theta) + (self.kp_theta * error_theta) )   # Y-axis?  thrust
        u4 = ((self.kd_r * error_psi) + (self.kp_psi * error_psi)     )     # yaw?

        # u4 = 1

        U = np.array([u1, u2, u3, u4])

        print(f"errors: x {round(error_x, 3)} y: {round(error_y, 3)} z: {round(error_z, 3)}")
        '''
        print(f"errors: x {round(error_x, 3)} y: {round(error_y, 3)} z: {round(error_z, 3)}\n\
                thrust: x {round(u2, 3)} y: {round(u3, 3)} z {round(u1, 3)} yaw {round(u4, 3)}")
        '''


        return U