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
        # Implement kalman filter here
        # Prediction stage
        I = numpy.eye(4)
        self.state = I * self.state

        # Update stage
        wheel_radius = 0.04
        Z = numpy.vstack([left.velocity[0]*wheel_radius, 
            right.velocity[0]*wheel_radius])
        F = numpy.mat([
            [0,0],
            [0,0],
            [1,0],
            [0,1]])
        self.state = F * Z
    
        # Now publish the result
        self.array.data = [self.state[i] for i in range(4)]
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

