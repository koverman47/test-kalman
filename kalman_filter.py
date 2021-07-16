#!/usr/bin/env python

import rospy
import numpy as np
from numpy import dot, zeros, eye
from std_msgs.msg import Float32
from time import time
from copy import deepcopy
from test_kalman.msg import State

class KalmanFilter():
    def __init__(self):
        self.Mu_ = np.array([[0], [0.5]])           # predicted state 
        self.Sigma_ = np.array([[0, 0],[0, 0.01]])  # predicted covariance
        self.Mu = np.array([[0], [0.5]])            # updated state
        self.Sigma = np.array([[0, 0],[0, 0.01]])   # updated covariance

        self.R = np.array([[0.1, 0],[0, 0.1]])      # process uncertainty
        self.A = np.array([[1, 0],[0, 0]])          # state transition model
        self.B = None                               # control input model
        self.C = np.array([[1, 0],[0, 1]])          # sensor model
        self.Q = np.array([[10, 0],[0, 10]])        # sensor uncertainty
        self.K = np.zeros((2, 2))                   # kalman gain
        self.I = np.eye(2)


    def predict(self, u=None, B=None):
        # mu_t_ = A*mu_t-1 + B*u
        if B is not None and u is not None:
            self.Mu_ = dot(self.A, self.Mu) + dot(B, u)
        else:
            self.Mu_ = dot(self.A, self.Mu)
        
        # Sigma_t = A*Sigma_t-1*A' + R
        self.Sigma_ = dot(dot(self.A, self.Sigma), self.A.T) + self.R

        return(self.Mu_, self.Sigma_)


    def update(self, z=None):
        # No new obervation
        if z is None:
            self.Mu = self.Mu_.copy()
            self.Sigma = self.Sigma_.copy()
            return(self.Mu, self.Sigma)

        # K_t = Sigma_t*C.T*(C*Sigma_t*C.T+Q)^-1
        self.K = dot(dot(self.Sigma_, self.C.T), np.linalg.inv(dot(self.C, dot(self.Sigma_, self.C.T)) + self.Q))

        # mu_t = mu_t + K_t(z_t - C*mu_t)
        self.Mu = self.Mu_+dot(self.K, (z - dot(self.C, self.Mu_)))

        # Sigma_t = (I - K_t*C)*Sigma_t
        self.Sigma = dot((self.I - dot(self.K, self.C)), self.Sigma_)

        return(self.Mu, self.Sigma)


        

if __name__ == "__main__":
    global t1, t2, u
    t1 = time()
    t2 = time()
    estimated_state = State()
    u = 0.

    def callback(data):
        global t1, t2, u
        u = data.data
        t2 = time()
        dt = t2 - t1
        t1 = t2

        B = np.array([[dt],[1]])
        kf.predict(u, B)
        (mu, sigma) = kf.update()
        (estimated_state.position, estimated_state.velocity) = mu

        pub.publish(estimated_state)

        rospy.loginfo("Estimate Position : %f" % estimated_state.position)
        rospy.loginfo("Estimate Velocity : %f" % estimated_state.velocity)

    kf = KalmanFilter()

    rospy.init_node('kalman_filter_node')
    pub = rospy.Publisher('state_estimate', State, queue_size=1)
    rospy.Subscriber('dynamics', Float32, callback)
    
    rospy.spin()
    
    
    
