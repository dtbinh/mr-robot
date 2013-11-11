#!/usr/bin/env python
import roslib; roslib.load_manifest('kf_lab')
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState
import tf
from tf.transformations import euler_from_quaternion
import numpy
import math


class KFExample:
    def __init__(self,name):
        self.name = name
        self.position_stddev = 0.50 # [m]
        self.ref_frame = "/world"
        self.pose_frame = "/bubbleRob"
        rospy.init_node('kf_example')
        self.ref_frame = rospy.get_param("~ref_frame",self.ref_frame)
        self.pose_frame = rospy.get_param("~pose_frame",self.pose_frame)
        self.position_stddev = rospy.get_param("~position_stddev_m",self.position_stddev)
        rospy.loginfo("Starting KF Example" )
        self.listener = tf.TransformListener()
        self.array = Float64MultiArray()
        self.array.layout = MultiArrayLayout()
        self.array.layout.data_offset = 0
        self.array.layout.dim = [MultiArrayDimension('data',4,4)]
        self.array.data = [0.0] * 4

        # Defines
        self.deltaT = 1/5 # deltaT = 1/rospy.rate(in hz)
        self.P = numpy.eye(4) # initial P_0

        # State is x, y, vx, vy
        self.state = numpy.vstack([0.0, 0.0, 0.0, 0.0])

    # Filter measurement Z = numpy.vstack([x,y])
    def filter(self,Z):
        # Implement kalman filter here
        deltaT = 1 # TODO What is the value of deltaT? 
        var = self.position_stddev**2
        A = numpy.mat([
            [1, 0, self.deltaT, 0],
            [0, 1, 0, self.deltaT],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
	
	U=self.linearVelocity # TODO How to retrieve that ?
	k= # TODO parameter to update the predicted velocity with the input
	theta=atan(Z[1,3]/Z[1,4])

	B = numpy.vstack([0.0, 0.0, k*cos(theta), k*sin(theta)])

        Q = numpy.mat([
            [var, 0, 0, 0],
            [0, var, 0, 0],
            [0, 0, var, 0],
            [0, 0, 0, var]])
        R = numpy.mat([
            [var, 0, 0, 0],
            [0, var, 0, 0],
            [0, 0, var, 0],
            [0, 0, 0, var]])
        P_ = numpy.zeros((2,2))
        # Prediction stage
        I = numpy.eye(4)
        self.state = I * self.state
        
        X_ = numpy.zeros((4,1))
        X_ = A * self.state + B * U

        P_ = A*self.P*A.T()+Q
        # Update stage
        F = numpy.mat([
            [1,0],
            [0,1],
            [0,0],
            [0,0]])
        
        # Measurement Update
        H = numpy.zeros((2,4))
        K = numpy.zeros((4,2))
        H[0,0] = 1
        H[1,1] = 1
        # Update the Kalman Gain
        T = H*P_*H.T() + R
        K = P_*H.T()/numpy.linalg.inv(T)
        # Update the a posteriori state
        X = X_ + K*(Z-H*X_)
        # Update P
        self.P = (numpy.eye(4)-K*H)*P_

        # self.state = F*Z
        self.state = X
    
        # Now publish the result
        self.array.data = [self.state[i] for i in range(4)]
        self.pub.publish(self.array)

    def run(self):
        rate = rospy.Rate(5) # 5hz
        # Required for TF to initialise
        rospy.sleep(1.0)
        self.pub = rospy.Publisher("~state",Float64MultiArray)
        while not rospy.is_shutdown():
            t = rospy.Time.now()
            self.listener.waitForTransform(self.pose_frame,self.ref_frame, t, rospy.Duration(1.0))
            ((x,y,z),rot) = self.listener.lookupTransform(self.pose_frame,self.ref_frame, t)
            euler = euler_from_quaternion(rot)
            Z = numpy.vstack([x,y])
            self.filter(Z)
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = KFExample("kf_example") 
        rd.run()
    except rospy.ROSInterruptException:
        pass
