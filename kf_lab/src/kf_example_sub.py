#!/usr/bin/env python
import roslib; roslib.load_manifest('kf_lab')
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState
import tf
from tf.transformations import euler_from_quaternion
import numpy
import message_filters


class KFExample:
    def __init__(self,name):
        self.name = name
        self.position_stddev = 0.50 # [m]
        self.ref_frame = "/world"
        self.pose_frame = "/bubblerob"
        rospy.init_node('kf_example')
        self.ref_frame = rospy.get_param("~ref_frame",self.ref_frame)
        self.pose_frame = rospy.get_param("~pose_frame",self.pose_frame)
        self.position_stddev = rospy.get_param("~position_stddev_m",self.position_stddev)
        rospy.loginfo("Starting KF Example" )
        # Creating an Float64MultiArray to export the results
        self.array = Float64MultiArray()
        self.array.layout = MultiArrayLayout()
        self.array.layout.data_offset = 0
        self.array.layout.dim = [MultiArrayDimension('data',4,4)]
        self.array.data = [0.0] * 4

        # State is x, y, vx, vy
        self.state = numpy.vstack([0.0, 0.0, 0.0, 0.0])

    # joint = [Left Motor State, Right Motor State]
    def joint_cb(self, left, right):
		t = rospy.Time.now()
        self.listener.waitForTransform(self.pose_frame,self.ref_frame, t, rospy.Duration(1.0))
        ((x,y,z),rot) = self.listener.lookupTransform(self.pose_frame,self.ref_frame, t)
			
		oldRot=sys.maxint
		if oldRot==sys.maxint:
			deltaTheta=0
			oldRot=rot
		else:
			deltaTheta=rot-oldRot
			oldRot=rot
		
		# http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
		deltaThetaL=left.velocity
		deltaThetaR=right.velocity
		
        # Implement kalman filter here
        # Prediction stage
		var = self.position_stddev**2
        A = numpy.eye(5)
			
        I = numpy.eye(5)
		
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
        
        X_ = numpy.zeros((5,1))
        X_ = A * self.state	# Let's ignore the input to simplify things

        P_ = A*self.P*A.T()+Q

        # Update stage
        wheel_radius = 0.04
        
		# Measurement Update
        H = numpy.zeros((2,5))
        K = numpy.zeros((5,2))
        H[0,0] = 1/deltaThetaL
        H[1,0] = 1/deltaThetaR
		H[0,2] = -(2*self.state[0]+wheel_radius*deltaTheta)/(4*deltaThetaL*deltaThetaL)
		H[1,3] = (2*self.state[0]-wheel_radius*deltaTheta)/(4*deltaThetaR*deltaThetaR)
		H[0,4] = (2*self.state[0]-wheel_radius*deltaTheta)/(4*deltaThetaR*deltaThetaR)
		H[1,4] = wheel_radius/(2*deltaThetaL)
		H[2,4] = -wheel_radius/(2*deltaThetaR)
		
        # Update the Kalman Gain
        T = H*P_*H.T() + R
        K = P_*H.T()/numpy.linalg.inv(T)
        # Update the a posteriori state
        X = X_ + K*(Z-H*X_)
        # Update P
        self.P = (numpy.eye(4)-K*H)*P_

        # Update self.state
		self.state = X
    
        # Now publish the result
        self.array.data = [self.state[i] for i in range(5)]
        self.pub.publish(self.array)



    def run(self):
        timeout = True
        rate = rospy.Rate(2)
        rospy.sleep(1.0)
        # Now subscribe to the wheel encoder, but create a time synchronizer to
        # receive left and right at the same time
        self.pub = rospy.Publisher("~state",Float64MultiArray)
        self.left_joint_sub = message_filters.Subscriber("/vrep/leftWheelEncoder", JointState)
        self.right_joint_sub = message_filters.Subscriber("/vrep/rightWheelEncoder", JointState)
        self.ts = message_filters.TimeSynchronizer([self.left_joint_sub,self.right_joint_sub], 10)
        self.ts.registerCallback(self.joint_cb)
			
        # Now just wait...
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = KFExample("kf_example") 
        rd.run()
    except rospy.ROSInterruptException:
        pass

