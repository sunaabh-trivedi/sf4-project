"""
Computing the optimal (Kalman) filter estimate for altitude based on measured accelerometer and barometric pressure readings.
"""

import numpy as np

class Kalman():

    def __init__(self, x_dim, z_dim):

        self.delta_t = 0.02 # Based on 50 Hz sampling freq
        
        # TODO: Measure/estimate variances

        # Variance of external acceleration inputs (empirical)
        external_acc_var = 1
        # Variance of accelerometer bias noise (also empirical)
        acc_bias_var = 1

        self.acc_real = 0 # Estimated acceleration
        self.acc_measured = 0

        """
        x[0] = Altitude
        x[1] = Climb rate
        x[2] = Acceleration bias
        """

        self.x = np.zeros(x_dim) # x_dim vector
        self.P = np.eye(x_dim) # x_dim x x_dim Identity matrix
        self.Q = np.eye(x_dim)
    
        # State transition matrix, based on Newtonian kinematics
        self.F = np.array([[1, self.delta_t, -0.5*self.delta_t**2], 
                  [0, 1, -self.delta_t], 
                  [0, 0, 1]])
        
        # Process noise covariance matrix, from residual in Newtonian equations
        self.Q = np.array([[0.25 * self.delta_t**4 *external_acc_var, 0.5 * self.delta_t**3 * external_acc_var, 0], 
                  [0.5 * self.delta_t**3*external_acc_var, self.delta_t**2 * external_acc_var, 0], 
                  [0, 0, acc_bias_var]])
        
        # Measurement function, picks out the first element (altitude) of the state vector
        self.H = np.array([1, 0, 0])

        # Measurement (altitude) noise variance (empirical)
        self.R = np.array([1])

        self.I = np.eye(x_dim) # Identity matrix


    def predict(self, acc_measured, F = None, Q = None):

        """
        Brief: Predict the prior estimate of altitude based on the physical model (Newtonian)
        Params: x - Kalman state (incl. altitude, climbing rate, acceleration bias)
                F - State transition matrix (physical model)
                P - State covariance matrix (updated through physical process)
                Q - Process noise covariance matrix
        """

        F = self.F
        Q = self.Q

        # Compute the prior covariance matrix, P_{k, k-1}
        self.P = np.dot(np.dot(F, self.P), F.T) + Q

        # Compute the prior state variable, x_{k, k-1}
        # This accounts for the measured acceleration from the accelerometer

        self.acc_real = acc_measured - self.x[2]
        self.x[1] = self.x[1] + self.acc_real*self.delta_t
        self.x[0] = self.x[0] + self.x[1]*self.delta_t

        return self.x, self.P


    def update(self, z, R = None, H = None):
        """
        Brief: Get the posterior estimate of altitude based on measurement, z
        Params: z - Measurement of altitude based on barometric pressure reading
                R - Measurement noise covariance matrix
                H - Measurement function, which maps the system state to the observation (will just be [1, 0, 0] in this case as we only get altitude measurements over the serial link)
        """

        R = self.R
        H = self.H

        # Compute the Innovation
        y = z - np.dot(H, self.x)

        # Compute the Innovation covariance
        S = np.dot(np.dot(H, self.P), H.T) + R

        # Compute the Kalman gain
        if np.isscalar(S):
            S_inv = 1/S
        else:
            S_inv = np.linalg.inv(S)

        K = np.dot(np.dot(self.P, H.T), S_inv)

        # Compute the posterior estimate of state (altitude)
        self.x = self.x + y*K.T

        # Compute the posterior estimate of the state covariance matrix
        I_KH = (self.I - K @ H) 
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T)+ np.dot(np.dot(K, R), K.T)

        return self.x, self.P